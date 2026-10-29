/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testReducedALRetractionValidation.cpp
 * @brief Validate reduced AL retraction inputs and results.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QuadraticConstraint.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace gtsam;
using namespace gtdynamics;

namespace {
constexpr Key kX = 1, kY = 2;

IEConstraintManifold makeManifold(
    const Values &values, const NonlinearEqualityConstraints &equalities = {},
    const NonlinearInequalityConstraints &inequalities = {},
    const IERetractorParams::shared_ptr &metric =
        std::make_shared<IERetractorParams>(),
    const std::optional<KeyVector> &priorKeys = {}) {
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->affineEqualities = true;
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedALRetractor>(
          metric, ReducedALRetractor::Settings{}, priorKeys));
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

// Original quadratic factors remain supported without a conversion callback.
// Their own Jacobian handles the skew-symmetric part of Q.
TEST(ReducedALValidation, AsymmetricQuadraticProjection) {
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
TEST(ReducedALValidation, FullyFixedAffineManifold) {
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
TEST(ReducedALValidation, PartialPriorMetric) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(0)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(kX, Matrix::Identity(1, 1), kY,
                                          Matrix::Identity(1, 1), Vector1(0)))
                           .createEqualityFactor());
  auto metric = std::make_shared<IERetractorParams>();
  const auto start =
      makeManifold(values, equalities, {}, metric, KeyVector{kX});
  VectorValues delta;
  delta.insert(kX, Vector1(2));
  delta.insert(kY, Vector1(0));
  const auto result = start.retract(delta);
  EXPECT_DOUBLES_EQUAL(2, result.values().at<Vector>(kX)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(-2, result.values().at<Vector>(kY)(0), 1e-7);

  const auto unconstrained =
      makeManifold(values, {}, {}, metric, KeyVector{kX});
  CHECK_EXCEPTION(unconstrained.retract(delta), std::invalid_argument);
}

TEST(ReducedALValidation, InvalidMetricConfiguration) {
  auto metric = std::make_shared<IERetractorParams>();
  metric->useVaryingSigma = true;
  CHECK_EXCEPTION(makeManifold(scalarVector(0), {}, {}, metric),
                  std::invalid_argument);
}

TEST(ReducedALValidation, RejectsNonfiniteTrial) {
  const auto start = makeManifold(scalarVector(0));
  VectorValues delta;
  delta.insert(kX, Vector1(std::numeric_limits<double>::infinity()));
  CHECK_EXCEPTION(start.retract(delta), std::invalid_argument);
}

namespace {
// Deliberately nonquadratic and multi-key. There is no polynomial model or
// original equality wrapper for the retractor to rely on.
class ExponentialBound : public NonlinearInequalityConstraint {
 public:
  explicit ExponentialBound(double sigma = 1, bool invalidJacobian = false)
      : NonlinearInequalityConstraint(constrainedNoise(Vector1(sigma)),
                                      KeyVector{kX, kY}),
        invalidJacobian_(invalidJacobian) {}
  Vector unwhitenedExpr(const Values &values,
                        OptionalMatrixVecType H = nullptr) const override {
    const double exponential =
        std::exp(values.at<Vector>(kX)(0) + values.at<Vector>(kY)(0));
    if (H) {
      (*H)[0] = (*H)[1] = Matrix::Constant(1, 1, exponential);
      if (invalidJacobian_)
        (*H)[1](0, 0) = std::numeric_limits<double>::quiet_NaN();
    }
    return Vector1(exponential - std::exp(1.0));
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ExponentialBound>(*this);
  }

 private:
  bool invalidJacobian_;
};
}  // namespace

// exp(x+y)<=e has the same half-space projection, but is not a QCQP expression.
// Changing the original sigma must preserve the solution and its original
// whitened feasibility. Both key Jacobians contribute to the reduced row.
TEST(ReducedALValidation, NonquadraticMultiKeyProjectionAndWhitening) {
  for (double sigma : {0.01, 1.0, 100.0}) {
    Values values = scalarVector(0);
    values.insert(kY, Vector(Vector1(0)));
    NonlinearInequalityConstraints inequalities;
    inequalities.emplace_shared<ExponentialBound>(sigma);
    const auto start = makeManifold(values, {}, inequalities);
    VectorValues delta;
    delta.insert(kX, Vector1(2));
    delta.insert(kY, Vector1(1));
    const auto result = start.retract(delta);
    EXPECT_DOUBLES_EQUAL(1, result.values().at<Vector>(kX)(0), 1e-6);
    EXPECT_DOUBLES_EQUAL(0, result.values().at<Vector>(kY)(0), 1e-6);
    EXPECT(result.evaluateInequalityViolation() <= 1e-8);
    EXPECT(result.activeIndices().exists(0));
  }
}

// Reduction must chain both original key blocks, including the dependent key.
// A blocked nonlinear inequality is an equality even for an interior target.
TEST(ReducedALValidation, NonquadraticBoundaryOnAffineManifold) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(0)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(kX, Matrix::Constant(1, 1, -2), kY,
                                          Matrix::Identity(1, 1), Vector1(0)))
                           .createEqualityFactor());
  NonlinearInequalityConstraints inequalities;
  inequalities.emplace_shared<ExponentialBound>();
  const auto start = makeManifold(values, equalities, inequalities);
  IndexSet blocking;
  blocking.insert(0);
  const auto result = start.retract(values.zeroVectors(), blocking);
  EXPECT_DOUBLES_EQUAL(1.0 / 3, result.values().at<Vector>(kX)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(2.0 / 3, result.values().at<Vector>(kY)(0), 1e-7);
  EXPECT(result.evaluateEqualityViolation() <= 1e-8);
  EXPECT(std::abs(inequalities.at(0)->whitenedExpr(result.values())(0)) <=
         1e-8);
}

TEST(ReducedALValidation, RejectsNonfiniteOriginalEvaluation) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(0)));
  NonlinearInequalityConstraints inequalities;
  inequalities.emplace_shared<ExponentialBound>();
  const auto start = makeManifold(values, {}, inequalities);
  VectorValues delta = values.zeroVectors();
  delta.at(kX)(0) = 1000;  // exp overflows only at the target.
  CHECK_EXCEPTION(start.retract(delta), std::invalid_argument);
  inequalities.resize(0);
  inequalities.emplace_shared<ExponentialBound>(1.0, true);
  const auto invalid = makeManifold(values, {}, inequalities);
  // An infeasible trial exercises the Jacobian path; a feasible zero trial
  // legitimately requires no derivative evaluation or AL solve.
  delta.at(kX)(0) = 2;
  CHECK_EXCEPTION(invalid.retract(delta), std::invalid_argument);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
