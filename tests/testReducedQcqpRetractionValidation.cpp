/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testReducedQcqpRetractionValidation.cpp
 * @brief Validate reduced QCQP retraction inputs and results.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/ReducedQcqpRetractor.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace gtsam;
using namespace gtdynamics;

namespace {
constexpr Key kX = 1, kY = 2;

QuadraticConstraint exactModel(const NonlinearInequalityConstraint& factor) {
  return dynamic_cast<const QuadraticInequalityConstraintFactor&>(factor)
      .quadraticConstraint();
}

IEConstraintManifold makeManifold(
    const Values& values, const NonlinearEqualityConstraints& equalities = {},
    const NonlinearInequalityConstraints& inequalities = {},
    ReducedQcqpRetractor::InequalityModel model = exactModel,
    const IERetractorParams::shared_ptr& metric =
        std::make_shared<IERetractorParams>(),
    const std::optional<KeyVector>& priorKeys = {}) {
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedQcqpRetractor>(
          ReducedQcqpRetractor::AffineEqualities{}, std::move(model), metric,
          ReducedQcqpRetractor::Settings{}, priorKeys));
  return IEConstraintManifold(
      params, std::make_shared<NonlinearEqualityConstraints>(equalities),
      std::make_shared<NonlinearInequalityConstraints>(inequalities), values);
}

Values scalarVector(double x) {
  Values values;
  values.insert(kX, Vector(Vector1(x)));
  return values;
}

NonlinearInequalityConstraints upperBound(double upper) {
  NonlinearInequalityConstraints constraints;
  constraints.push_back(
      QuadraticConstraint(kX, Matrix::Zero(1, 1), Vector1(0.5), upper,
                          QuadraticConstraint::Sense::LessEqual)
          .createInequalityFactor());
  return constraints;
}
}  // namespace

// The skew-symmetric part of Q contributes neither to x'Qx nor its derivative.
// A nonzero target catches an erroneous affine cross term Q*x rather than
// 0.5*(Q+Q')*x when shifting the reduced coordinates.
TEST(ReducedQcqpValidation, AsymmetricQuadraticProjection) {
  Values values;
  values.insert(kX, Vector(Vector2(0.5, 0)));
  Matrix2 Q;
  Q << 1, 3, -3, 1;
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      QuadraticConstraint::LessEqual(kX, Q, 1).createInequalityFactor());
  const auto start = makeManifold(values, {}, inequalities);
  VectorValues delta;
  delta.insert(kX, Vector2(1.5, 1));
  const auto result = start.retract(delta);
  const Vector expected = Vector2(2, 1) / std::sqrt(5.0);
  EXPECT(assert_equal(expected, result.values().at<Vector>(kX), 1e-6));
  EXPECT(result.activeIndices().exists(0));
}

// A zero-dimensional affine manifold has exactly one point. No optimization
// can change that point, even if the ambient trial target is different.
TEST(ReducedQcqpValidation, FullyFixedAffineManifold) {
  const Values values = scalarVector(2);
  NonlinearEqualityConstraints equalities;
  equalities.push_back(
      LinearConstraint::Equal(
          JacobianFactor(kX, Matrix::Identity(1, 1), Vector1(2)))
          .createEqualityFactor());
  const auto start = makeManifold(values, equalities, upperBound(3));
  EXPECT_LONGS_EQUAL(0, start.eBasis()->dim());
  VectorValues delta;
  delta.insert(kX, Vector1(7));
  const auto result = start.retract(delta);
  EXPECT(assert_equal(values, result.values(), 1e-12));
  EXPECT(result.activeIndices().empty());
  IndexSet impossibleBoundary;
  impossibleBoundary.insert(0);
  CHECK_EXCEPTION(start.retract(delta, impossibleBoundary), std::runtime_error);
}

// Omitting priors is legitimate only if the retained priors still measure
// every independent equality-manifold direction.
TEST(ReducedQcqpValidation, PartialPriorMetric) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(0)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(
      LinearConstraint::Equal(JacobianFactor(
          kX, Matrix::Identity(1, 1), kY, Matrix::Identity(1, 1), Vector1(0)))
          .createEqualityFactor());
  auto metric = std::make_shared<IERetractorParams>();
  const auto start = makeManifold(values, equalities, {}, exactModel, metric,
                                  KeyVector{kX});
  VectorValues delta;
  delta.insert(kX, Vector1(2));
  delta.insert(kY, Vector1(0));
  const auto result = start.retract(delta);
  EXPECT_DOUBLES_EQUAL(2, result.values().at<Vector>(kX)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(-2, result.values().at<Vector>(kY)(0), 1e-7);

  const auto unconstrained = makeManifold(values, {}, {}, exactModel, metric,
                                          KeyVector{kX});
  CHECK_EXCEPTION(unconstrained.retract(delta), std::invalid_argument);
}

TEST(ReducedQcqpValidation, RejectsNonfiniteQuadraticModels) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  for (int badCoefficient = 0; badCoefficient < 3; ++badCoefficient) {
    const auto start = makeManifold(
        scalarVector(0), {}, upperBound(1),
        [=](const NonlinearInequalityConstraint&) {
          Matrix Q = Matrix::Zero(1, 1);
          Vector a = Vector1(0.5);
          double b = 1;
          if (badCoefficient == 0) Q(0, 0) = nan;
          if (badCoefficient == 1) a(0) = infinity;
          if (badCoefficient == 2) b = nan;
          return QuadraticConstraint(kX, Q, a, b,
                                      QuadraticConstraint::Sense::LessEqual);
        });
    CHECK_EXCEPTION(start.retract(start.values().zeroVectors()),
                    std::invalid_argument);
  }
}

// Acceptance checks the original factor, independently of its supplied QCQP
// model: a mistaken looser model must not silently introduce infeasible values.
TEST(ReducedQcqpValidation, OriginalInequalityValidation) {
  const auto start = makeManifold(
      scalarVector(0), {}, upperBound(1),
      [](const NonlinearInequalityConstraint&) {
        return QuadraticConstraint(kX, Matrix::Zero(1, 1), Vector1(0.5), 2,
                                    QuadraticConstraint::Sense::LessEqual);
      });
  VectorValues delta;
  delta.insert(kX, Vector1(1.5));
  CHECK_EXCEPTION(start.retract(delta), std::runtime_error);
}

TEST(ReducedQcqpValidation, InvalidMetricConfiguration) {
  auto metric = std::make_shared<IERetractorParams>();
  metric->useVaryingSigma = true;
  CHECK_EXCEPTION(makeManifold(scalarVector(0), {}, {}, exactModel, metric),
                  std::invalid_argument);
}

TEST(ReducedQcqpValidation, RejectsNonfiniteTrial) {
  const auto start = makeManifold(scalarVector(0));
  VectorValues delta;
  delta.insert(kX, Vector1(std::numeric_limits<double>::infinity()));
  CHECK_EXCEPTION(start.retract(delta), std::invalid_argument);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
