/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testReducedALRetractor.cpp
 * @brief Test reduced AL retraction and cone projections.
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <stdexcept>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
namespace affine_projection {

constexpr Key kX = 1, kY = 2;

IEConstraintManifold manifold(
    const Values &values, const NonlinearEqualityConstraints &equalities = {},
    const NonlinearInequalityConstraints &inequalities = {},
    const IERetractorParams::shared_ptr &metric =
        std::make_shared<IERetractorParams>(),
    const TangentSpaceBasisCreator::shared_ptr &basis =
        std::make_shared<OrthonormalBasisCreator>(),
    const ReducedALRetractor::Settings &settings = {}) {
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->affineEqualities = true;
  params->equalityBasisCreator = basis;
  params->retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedALRetractor>(metric, settings));
  return IEConstraintManifold(
      params, std::make_shared<NonlinearEqualityConstraints>(equalities),
      std::make_shared<NonlinearInequalityConstraints>(inequalities), values);
}

NonlinearInequalityConstraint::shared_ptr bound(
    double value, QuadraticConstraint::Sense sense) {
  return QuadraticConstraint(kX, Matrix::Zero(1, 1), Vector1(0.5), value, sense)
      .createInequalityFactor();
}

Values scalarVector(double value) {
  Values values;
  values.insert(kX, Vector(Vector1(value)));
  return values;
}

// A retraction fixes an already feasible point when its tangent step is zero.
TEST(ReducedALRetractor, ZeroStepIdentity) {
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(0, QuadraticConstraint::Sense::GreaterEqual));
  const auto start = manifold(scalarVector(1.25), {}, inequalities);
  const auto result = start.retract(start.values().zeroVectors());
  EXPECT(assert_equal(start.values(), result.values(), 1e-12));
  EXPECT(result.activeIndices().empty());
}

// Feasibility is enough only when the point minimizes the projection cost.
TEST(ReducedALRetractor, FeasibleTrialSkipsAL) {
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(2, QuadraticConstraint::Sense::LessEqual));
  const auto start = manifold(scalarVector(0), {}, inequalities);
  VectorValues delta;
  delta.insert(kX, Vector1(1));
  IERetractionInfo info;
  const auto result = start.retract(delta, {}, &info);
  EXPECT_DOUBLES_EQUAL(1, result.values().at<Vector>(kX)(0), 0);
  EXPECT_LONGS_EQUAL(1, info.numFeasibleTrials);
  EXPECT_LONGS_EQUAL(0, info.numALSolveCalls);
  EXPECT(start.eBasis() == result.eBasis());
  IERetractorParams metric;
  EXPECT(start.projectionMetric(metric) == result.projectionMetric(metric));

  // A feasible interior point cannot satisfy a request to land on the face.
  IERetractionInfo forced;
  IndexSet blocking;
  blocking.insert(0);
  const auto boundary = start.retract(delta, blocking, &forced);
  EXPECT_DOUBLES_EQUAL(2, boundary.values().at<Vector>(kX)(0), 1e-8);
  EXPECT_LONGS_EQUAL(1, forced.numALSolveCalls);
  EXPECT_LONGS_EQUAL(0, forced.numFeasibleTrials);

  delta.at(kX)(0) = 3;
  IERetractionInfo outside;
  start.retract(delta, {}, &outside);
  EXPECT_LONGS_EQUAL(1, outside.numALSolveCalls);
}

TEST(ReducedALRetractor, AffineMetricReuseAndInvalidation) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(0)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(JacobianFactor(
      kX, Matrix11::Constant(-2), kY, Matrix11::Identity(), Vector1(0)))
      .createEqualityFactor());
  auto weights = IERetractorParams::createVaryingSigmas(LevenbergMarquardtParams());
  weights->metricSigmas->insert(kX, Vector1(1));
  weights->metricSigmas->insert(kY, Vector1(2));
  const auto start = manifold(values, equalities, {}, weights);
  auto first = start.projectionMetric(*weights);
  EXPECT(first == start.projectionMetric(*weights));
  VectorValues delta;
  delta.insert(kX, Vector1(2));
  delta.insert(kY, Vector1(0));
  IERetractionInfo info;
  const auto projected = start.retract(delta, {}, &info);
  EXPECT_LONGS_EQUAL(0, info.numALSolveCalls);
  EXPECT_LONGS_EQUAL(1, info.numFeasibleProjections);
  EXPECT_DOUBLES_EQUAL(1, projected.values().at<Vector>(kX)(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(2, projected.values().at<Vector>(kY)(0), 1e-12);
  EXPECT(first == projected.projectionMetric(*weights));

  weights->metricSigmas->at(kY)(0) = 1;
  auto changed = start.projectionMetric(*weights);
  EXPECT(first != changed);
  EXPECT_DOUBLES_EQUAL(0.4,
      start.retract(delta).values().at<Vector>(kX)(0), 1e-12);
  // A returned immutable metric remains valid after another metric replaces it.
  EXPECT_DOUBLES_EQUAL(1, first->recover(first->project(delta)).at(kX)(0), 1e-12);
  auto partial = start.projectionMetric(*weights, KeyVector{kX});
  EXPECT(partial != changed);
  EXPECT_DOUBLES_EQUAL(2, partial->recover(partial->project(delta)).at(kX)(0), 1e-12);
  weights->scaleVaryingSigma = true;
  weights->priorSigma = 3;
  auto scaled = start.projectionMetric(*weights, KeyVector{kX});
  EXPECT(scaled != partial);
  EXPECT_DOUBLES_EQUAL(2, scaled->recover(scaled->project(delta)).at(kX)(0), 1e-12);
  // Independent components must not share caches merely because keys match.
  const auto independent = manifold(values, equalities, {}, weights);
  EXPECT(independent.projectionMetric(*weights) != start.projectionMetric(*weights));
}

class CircleEquality : public NoiseModelFactor1<Vector> {
 public:
  CircleEquality() : NoiseModelFactor1<Vector>(noiseModel::Unit::Create(1), kX) {}
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<CircleEquality>(*this);
  }
  Vector evaluateError(const Vector &x, OptionalMatrixType H = nullptr) const override {
    if (H) *H = 2 * x.transpose();
    return Vector1(x.squaredNorm() - 1);
  }
};

// The default must still relinearize a curved equality manifold at each state.
TEST(ReducedALRetractor, NonlinearManifoldRebuildsBasisAndMetric) {
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<BarrierRetractorCreator>();
  auto equalities = std::make_shared<NonlinearEqualityConstraints>();
  equalities->push_back(std::make_shared<ZeroCostConstraint>(
      std::make_shared<CircleEquality>()));
  Values a, b;
  a.insert(kX, Vector(Vector2(1, 0)));
  b.insert(kX, Vector(Vector2(0, 1)));
  IEConstraintManifold start(params, equalities,
      std::make_shared<NonlinearInequalityConstraints>(), a);
  IERetractorParams metric;
  auto first = start.projectionMetric(metric);
  const auto next = start.createWithNewValues(b);
  EXPECT(start.eBasis() != next.eBasis());
  EXPECT(first != next.projectionMetric(metric));
  EXPECT_DOUBLES_EQUAL(0, start.eBasis()->recoverJacobian(kX)(0, 0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0, next.eBasis()->recoverJacobian(kX)(1, 0), 1e-12);
}

// Both inequality senses preserve their affine offsets and activate a new face.
TEST(ReducedALRetractor, BoundSignsAndNewActiveConstraint) {
  for (bool lower : {false, true}) {
    const double sign = lower ? -1.0 : 1.0;
    NonlinearInequalityConstraints inequalities;
    inequalities.push_back(
        bound(sign, lower ? QuadraticConstraint::Sense::GreaterEqual
                          : QuadraticConstraint::Sense::LessEqual));
    const auto start = manifold(scalarVector(0), {}, inequalities);
    EXPECT(start.activeIndices().empty());
    VectorValues delta;
    delta.insert(kX, Vector1(2 * sign));
    const auto result = start.retract(delta);
    EXPECT_DOUBLES_EQUAL(sign, result.values().at<Vector>(kX)(0), 2e-8);
    EXPECT(result.activeIndices().exists(0));
    EXPECT(result.evaluateInequalityViolation() <= 1e-8);
  }
}

// Large trial magnitudes must not stall on cancellation in the AL merit cost.
TEST(ReducedALRetractor, LargeTrialProjection) {
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(0, QuadraticConstraint::Sense::LessEqual));
  const auto start = manifold(scalarVector(0), {}, inequalities);
  for (double trial : {1000.0, 3000.0, 1e6}) {
    VectorValues delta;
    delta.insert(kX, Vector1(trial));
    const auto result = start.retract(delta);
    EXPECT_DOUBLES_EQUAL(0, result.values().at<Vector>(kX)(0), 1e-8);
    EXPECT(result.activeIndices().exists(0));
  }
}

// Selecting a boundary must override a trial that would stay in the interior.
TEST(ReducedALRetractor, BlockingFaceAndMoveToBoundary) {
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(0, QuadraticConstraint::Sense::GreaterEqual));
  const auto start = manifold(scalarVector(1), {}, inequalities);
  IndexSet blocking;
  blocking.insert(0);
  VectorValues delta;
  delta.insert(kX, Vector1(2));
  const auto result = start.retract(delta, blocking);
  EXPECT_DOUBLES_EQUAL(0, result.values().at<Vector>(kX)(0), 1e-8);
  EXPECT(result.activeIndices().exists(0));
  const auto boundary = start.moveToBoundary(blocking);
  EXPECT_DOUBLES_EQUAL(0, boundary.values().at<Vector>(kX)(0), 1e-8);
  EXPECT(boundary.activeIndices().exists(0));
}

// Elimination and whitening must preserve the metric, including dependent keys.
TEST(ReducedALRetractor, WeightedAffineEliminationProjection) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(1)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(kX, Matrix::Constant(1, 1, -2), kY,
                                          Matrix::Identity(1, 1), Vector1(1)))
                           .createEqualityFactor());
  auto metric = IERetractorParams::createVaryingSigmas();
  metric->metricSigmas->insert(kX, Vector1(1));
  metric->metricSigmas->insert(kY, Vector1(2));
  auto basis = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector &) { return KeyVector{kX}; });
  const auto start = manifold(values, equalities, {}, metric, basis);
  EXPECT_LONGS_EQUAL(1, start.eBasis()->dim());
  VectorValues delta;
  delta.insert(kX, Vector1(2));
  delta.insert(kY, Vector1(0));
  const auto result = start.retract(delta);
  // min (x-2)^2 + (y-1)^2/4, subject to y=2x+1.
  EXPECT_DOUBLES_EQUAL(1, result.values().at<Vector>(kX)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(3, result.values().at<Vector>(kY)(0), 1e-7);
  EXPECT(result.evaluateEqualityViolation() < 1e-12);

  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(0.5, QuadraticConstraint::Sense::LessEqual));
  const auto constrained =
      manifold(values, equalities, inequalities, metric, basis);
  const auto onFace = constrained.retract(delta);
  EXPECT_DOUBLES_EQUAL(0.5, onFace.values().at<Vector>(kX)(0), 2e-8);
  EXPECT_DOUBLES_EQUAL(2, onFace.values().at<Vector>(kY)(0), 4e-8);
  EXPECT(onFace.activeIndices().exists(0));
  EXPECT(onFace.evaluateEqualityViolation() < 1e-12);
}

// A null-space step cannot correct an infeasible affine anchor.
TEST(ReducedALRetractor, RejectsInfeasibleAffineAnchor) {
  Values values = scalarVector(0);
  values.insert(kY, Vector(Vector1(0)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(kX, Matrix::Constant(1, 1, -2), kY,
                                          Matrix::Identity(1, 1), Vector1(1)))
                           .createEqualityFactor());
  const auto start = manifold(values, equalities);
  CHECK_EXCEPTION(start.retract(values.zeroVectors()), std::invalid_argument);
}

// The affine contract must not silently apply Euclidean arithmetic to Pose3.
TEST(ReducedALRetractor, RejectsNonEuclideanValues) {
  Values values;
  values.insert(kX, Pose3());
  const auto start = manifold(values);
  CHECK_EXCEPTION(start.retract(values.zeroVectors()), std::invalid_argument);
}

// A requested nonexistent face is an input error.
TEST(ReducedALRetractor, RejectsInvalidBlockingIndex) {
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(0, QuadraticConstraint::Sense::GreaterEqual));
  const auto values = scalarVector(1);
  const auto start = manifold(values, {}, inequalities);
  IndexSet invalid;
  invalid.insert(1);
  CHECK_EXCEPTION(start.retract(values.zeroVectors(), invalid),
                  std::invalid_argument);
}

// Scalar and fixed-size Euclidean values retain their original dynamic types.
TEST(ReducedALRetractor, EuclideanValueTypes) {
  Values values;
  values.insert(kX, 2.0);
  values.insert(kY, Vector3(1, 2, 3));
  const auto start = manifold(values);
  VectorValues delta;
  delta.insert(kX, Vector1(0.5));
  delta.insert(kY, Vector3(-1, 2, -3));
  const auto result = start.retract(delta);
  EXPECT_DOUBLES_EQUAL(2.5, result.values().at<double>(kX), 1e-7);
  EXPECT(assert_equal(Vector3(0, 4, 0), result.values().at<Vector3>(kY), 1e-7));
}

}  // namespace affine_projection
/* ************************************************************************* */

namespace elliptic_projection {

constexpr Key kForce = 3;

// Use the original norm constraint in both CMC and the reduced AL solve.
class Cone : public NonlinearInequalityConstraint {
 public:
  explicit Cone(double mu)
      : NonlinearInequalityConstraint(constrainedNoise(Vector1(1)),
                                      KeyVector{kForce}),
        mu_(Vector1(mu)) {}

  explicit Cone(const Vector2 &mu)
      : NonlinearInequalityConstraint(constrainedNoise(Vector1(1)),
                                      KeyVector{kForce}),
        mu_(mu) {}

  Vector unwhitenedExpr(const Values &values,
                        OptionalMatrixVecType H = nullptr) const override {
    const Vector &force = values.at<Vector>(kForce);
    const Vector scaled = force.tail(mu_.size()).cwiseQuotient(mu_);
    const double norm = scaled.norm();
    if (H) {
      Matrix row = Matrix::Zero(1, force.size());
      row(0, 0) = -1;
      if (norm > 0)
        row.rightCols(mu_.size()) =
            (scaled.cwiseQuotient(mu_) / norm).transpose();
      (*H)[0] = row;
    }
    return Vector1(norm - force(0));
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<Cone>(*this);
  }

  NonlinearEqualityConstraint::shared_ptr createEqualityConstraint()
      const override;

 private:
  Vector mu_;
};

class ConeBoundary : public NonlinearEqualityConstraint {
 public:
  explicit ConeBoundary(const Cone &cone)
      : NonlinearEqualityConstraint(constrainedNoise(Vector1(1)), cone.keys()),
        cone_(cone) {}
  Vector unwhitenedError(const Values &values,
                         OptionalMatrixVecType H = nullptr) const override {
    return cone_.unwhitenedExpr(values, H);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<ConeBoundary>(*this);
  }

 private:
  Cone cone_;
};

NonlinearEqualityConstraint::shared_ptr Cone::createEqualityConstraint() const {
  return std::make_shared<ConeBoundary>(*this);
}

IEConstraintManifold manifold(const Vector2 &force, double mu = 1) {
  Values values;
  values.insert(kForce, Vector(force));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      QuadraticConstraint(kForce, Matrix2::Zero(), Vector2(0.5, 0), 0,
                          QuadraticConstraint::Sense::GreaterEqual)
          .createInequalityFactor());
  inequalities.emplace_shared<Cone>(mu);
  return affine_projection::manifold(values, {}, inequalities);
}

// The anisotropic cone projection has a known answer away from its apex.
TEST(ReducedALRetractor, EllipticAnalyticalProjection) {
  const auto start = manifold(Vector2(1, 0), 0.5);
  VectorValues delta;
  delta.insert(kForce, Vector2(0, 2));
  const auto result = start.retract(delta);
  EXPECT(assert_equal(Vector(Vector2(1.6, 0.8)),
                      result.values().at<Vector>(kForce), 1e-6));
  EXPECT(result.activeIndices().exists(1));
  EXPECT(!result.activeIndices().exists(0));
  EXPECT(result.evaluateInequalityViolation() <= 1e-8);
}

// Construct a curved anisotropic-cone projection from its known KKT normal.
TEST(ReducedALRetractor, ThreeDimensionalEllipticProjection) {
  Values values;
  values.insert(kForce, Vector(Vector3(1, 0, 0)));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      QuadraticConstraint(kForce, Matrix3::Zero(), Vector3(0.5, 0, 0), 0,
                          QuadraticConstraint::Sense::GreaterEqual)
          .createInequalityFactor());
  inequalities.emplace_shared<Cone>(Vector2(0.5, 0.25));
  const auto start = affine_projection::manifold(values, {}, inequalities);
  // At f=(2,.6,.4), grad(norm(t/mu)-n)=(-1,1.2,3.2).
  // Target=f+0.5*grad therefore has this exact metric projection.
  VectorValues delta;
  delta.insert(kForce, Vector3(0.5, 1.2, 2));
  const auto result = start.retract(delta);
  EXPECT(assert_equal(Vector(Vector3(2, 0.6, 0.4)),
                      result.values().at<Vector>(kForce), 1e-6));
  EXPECT(result.activeIndices().exists(1));
  EXPECT(!result.activeIndices().exists(0));
  EXPECT(result.evaluateInequalityViolation() <= 1e-8);
}

// A negative normal target must project to the apex, not a negative sheet.
TEST(ReducedALRetractor, ConeRejectsNegativeNormalSheet) {
  const auto start = manifold(Vector2(1, 0));
  for (double targetNormal : {-1.0, -1e-4}) {
    VectorValues delta;
    delta.insert(kForce, Vector2(targetNormal - 1, 0));
    const auto result = start.retract(delta);
    EXPECT(assert_equal(Vector(Vector2::Zero()),
                        result.values().at<Vector>(kForce), 2e-8));
    EXPECT(result.values().at<Vector>(kForce)(0) >= -1e-8);
  }
}

// A polar-cone target projects to the nonsmooth apex. The original Jacobian
// convention may still prevent AL convergence: failure must be explicit.
TEST(ReducedALRetractor, ApexHasExplicitAlConvergenceFailure) {
  const auto start = manifold(Vector2(1, 0));
  VectorValues delta;
  delta.insert(kForce, Vector2(-2, 0.5));
  bool failed = false;
  try {
    start.retract(delta);
  } catch (const std::runtime_error &error) {
    failed = true;
    const std::string message = error.what();
    EXPECT(message.find("AL failed") != std::string::npos);
  }
  EXPECT(failed);
}

// A zero trial at the nonsmooth apex must preserve the feasible anchor.
TEST(ReducedALRetractor, ZeroForceZeroStepIdentity) {
  const auto start = manifold(Vector2::Zero());
  const auto result = start.retract(start.values().zeroVectors());
  EXPECT(assert_equal(start.values(), result.values(), 1e-12));
  EXPECT(result.activeIndices().exists(0));
  EXPECT(result.activeIndices().exists(1));
}

}  // namespace elliptic_projection
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
