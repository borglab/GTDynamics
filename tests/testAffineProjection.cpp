/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file testAffineProjection.cpp
 * @brief Test shared projection geometry, distance semantics, and reduced
 * penalty retraction.
 * @author Frank Dellaert
 */
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/AffineProjection.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/nonlinear/factorTesting.h>
using namespace gtsam;
using namespace gtdynamics;

namespace {
constexpr Key kX = 1, kY = 2;
IEConstraintManifold line(const IERetractor::shared_ptr &retractor,
                          bool affine = true) {
  Values anchor;
  anchor.insert(kX, Vector(Vector1(0)));
  anchor.insert(kY, Vector(Vector1(1)));
  auto equalities = std::make_shared<NonlinearEqualityConstraints>();
  equalities->push_back(
      LinearConstraint::Equal(JacobianFactor(kX, Matrix11::Constant(-2), kY,
                                             Matrix11::Identity(), Vector1(1)))
          .createEqualityFactor());
  auto inequalities = std::make_shared<NonlinearInequalityConstraints>();
  // The inequality concerns kY, which is eliminated from the reduced
  // coordinates.
  inequalities->push_back(
      QuadraticConstraint(kY, Matrix::Zero(1, 1), Vector1(0.5), 3,
                          QuadraticConstraint::Sense::LessEqual)
          .createInequalityFactor());
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->affineEqualities = affine;
  params->equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector &) { return KeyVector{kX}; });
  params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  return IEConstraintManifold(params, equalities, inequalities, anchor);
}
IERetractorParams::shared_ptr weights() {
  auto p = IERetractorParams::createVaryingSigmas();
  p->metricSigmas->insert(kX, Vector1(1));
  p->metricSigmas->insert(kY, Vector1(2));
  p->feasibleThreshold = 1e-8;
  p->lmParams.maxIterations = 200;
  p->lmParams.absoluteErrorTol = p->lmParams.relativeErrorTol = 1e-14;
  return p;
}
VectorValues trial(double x, double y) {
  VectorValues d;
  d.insert(kX, Vector1(x));
  d.insert(kY, Vector1(y));
  return d;
}
} // namespace

TEST(AffineProjection, SharedPriorSemantics) {
  auto p = weights();
  const auto m = line(std::make_shared<BarrierRetractor>(p));
  const Values target = m.values().retract(trial(2, 4));
  // Compare with the existing helpers, including repeated and missing keys.
  for (bool varying : {false, true})
    for (bool scaled : {false, true}) {
      p->useVaryingSigma = varying;
      p->scaleVaryingSigma = scaled;
      p->priorSigma = 3;
      for (const KeyVector &keys :
           {KeyVector{kX, kY}, KeyVector{kX, kX}, KeyVector{kY}}) {
        NonlinearFactorGraph old, shared;
        if (!varying)
          AddGeneralPriors(target, keys, p->priorSigma, old);
        else if (scaled)
          AddGeneralPriors(target, keys, *p->metricSigmas, old, p->priorSigma);
        else
          AddGeneralPriors(target, keys, *p->metricSigmas, old);
        p->addPriors(target, keys, shared);
        EXPECT_DOUBLES_EQUAL(old.error(m.values()), shared.error(m.values()),
                             1e-12);
        EXPECT(assert_equal(old.linearize(m.values())->hessian().first,
                            shared.linearize(m.values())->hessian().first,
                            1e-12));
      }
    }
  p->metricSigmas->erase(kY);
  const auto precision = p->priorPrecisions(target, KeyVector{kX, kY});
  EXPECT(!precision.exists(kY));
}

TEST(AffineProjection, ObjectiveAndOriginalConstraintEquivalence) {
  auto p = weights();
  const auto m = line(std::make_shared<BarrierRetractor>(p));
  const auto delta =
      trial(4, 1); // deliberately not tangent: cost has a constant offset
  for (bool whiten : {false, true}) {
    p->whitenAffineCoordinates = whiten;
    AffineProjection projection(m, delta, *p, {}, {}, 1e-8, 1e-7, false);
    const auto problem = projection.buildProblem({});
    NonlinearFactorGraph ambient;
    p->addPriors(m.values().retract(delta), m.values().keys(), ambient);
    Values zero;
    zero.insert(AffineProjection::kReduced, Vector(Vector1(0)));
    for (double v : {-0.8, 0.3, 2.0}) {
      Values values;
      values.insert(AffineProjection::kReduced, Vector(Vector1(v)));
      const Values recovered = projection.recover(Vector1(v));
      EXPECT_DOUBLES_EQUAL(0, m.eConstraints()->violationNorm(recovered),
                           1e-12);
      EXPECT_DOUBLES_EQUAL(
          ambient.error(recovered) -
              ambient.error(projection.recover(Vector1(0))),
          problem.costs().error(values) - problem.costs().error(zero), 1e-11);
      EXPECT(assert_equal(m.iConstraints()->at(0)->whitenedExpr(recovered),
                          problem.iConstraints().at(0)->whitenedExpr(values),
                          1e-12));
      EXPECT(internal::testFactorJacobians(
          "shared projection objective",
          dynamic_cast<const NoiseModelFactor &>(*problem.costs().at(0)),
          values, 1e-5, 1e-7));
      const auto &constraint = problem.iConstraints().at(0);
      std::vector<Matrix> jacobians(1);
      constraint->unwhitenedExpr(values, &jacobians);
      Values plus = values, minus = values;
      plus.update(AffineProjection::kReduced, Vector(Vector1(v + 1e-5)));
      minus.update(AffineProjection::kReduced, Vector(Vector1(v - 1e-5)));
      const Matrix numerical = (constraint->unwhitenedExpr(plus) -
                                constraint->unwhitenedExpr(minus)) /
                               2e-5;
      EXPECT(assert_equal(numerical, jacobians[0], 1e-7));
    }
    const auto metric = m.projectionMetric(*p);
    EXPECT(metric == m.projectionMetric(*p));
    p->whitenAffineCoordinates = !whiten;
    EXPECT(metric != m.projectionMetric(*p));
    // A coordinate change preserves the ambient projection.
    EXPECT(assert_equal(
        metric->recover(metric->project(delta)),
        m.projectionMetric(*p)->recover(m.projectionMetric(*p)->project(delta)),
        1e-12));
  }
}

TEST(AffineProjection, ThreeSolversAndBlockingFaces) {
  for (int backend : {0, 1, 2})
    for (bool whiten : {false, true}) {
      auto p = weights();
      p->whitenAffineCoordinates = whiten;
      p->useReducedCoordinates = backend == 1;
      IERetractor::shared_ptr retractor =
          backend == 2 ? std::static_pointer_cast<IERetractor>(
                             std::make_shared<ReducedALRetractor>(p))
                       : std::make_shared<BarrierRetractor>(p);
      const auto m = line(retractor);
      IERetractionInfo info;
      const auto result = m.retract(trial(3, 6), {}, &info);
      EXPECT_DOUBLES_EQUAL(1, result.values().at<Vector>(kX)(0), 1e-7);
      EXPECT_DOUBLES_EQUAL(3, result.values().at<Vector>(kY)(0), 1e-7);
      EXPECT(result.activeIndices().exists(0));
      EXPECT(result.eBasis() == m.eBasis());
      if (backend == 1)
        EXPECT_LONGS_EQUAL(1, info.numPenaltySolveCalls);
      // A feasible interior trial must still land on a requested boundary face.
      IndexSet blocking;
      blocking.insert(0);
      const auto forced = m.retract(trial(.25, .5), blocking);
      EXPECT_DOUBLES_EQUAL(3, forced.values().at<Vector>(kY)(0), 1e-7);
      const auto boundary = retractor->moveToBoundary(&m, blocking);
      EXPECT_DOUBLES_EQUAL(3, boundary.values().at<Vector>(kY)(0), 1e-7);
    }
}

TEST(AffineProjection, ReducedPenaltyShortcutsAndFallback) {
  auto p = weights();
  p->useReducedCoordinates = true;
  auto retractor = std::make_shared<BarrierRetractor>(p);
  const auto m = line(retractor);
  IERetractionInfo trialInfo, projectionInfo;
  m.retract(trial(.1, .2), {}, &trialInfo);
  EXPECT_LONGS_EQUAL(1, trialInfo.numFeasibleTrials);
  EXPECT_LONGS_EQUAL(0, trialInfo.numPenaltySolveCalls);
  m.retract(trial(.2, 0), {}, &projectionInfo);
  EXPECT_LONGS_EQUAL(1, projectionInfo.numFeasibleProjections);
  EXPECT_LONGS_EQUAL(0, projectionInfo.numPenaltySolveCalls);
  // An un-certified component must keep the existing ambient path.
  const auto ambient = line(retractor, false);
  IERetractionInfo fallback;
  const auto result = ambient.retract(trial(3, 6), {}, &fallback);
  EXPECT_DOUBLES_EQUAL(3, result.values().at<Vector>(kY)(0), 1e-7);
  EXPECT_LONGS_EQUAL(0, fallback.numPenaltySolveCalls);
  EXPECT(ambient.eBasis() != result.eBasis());
  // Selecting only kX still measures all independent directions on this line.
  auto partial = std::make_shared<BarrierRetractor>(p, KeyVector{kX});
  EXPECT_DOUBLES_EQUAL(
      3, line(partial).retract(trial(3, 6)).values().at<Vector>(kY)(0), 1e-7);
}

namespace {
class Circle : public NoiseModelFactor1<Vector> {
public:
  Circle() : NoiseModelFactor1<Vector>(noiseModel::Unit::Create(1), kX) {}
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<Circle>();
  }
  Vector evaluateError(const Vector &x,
                       OptionalMatrixType H = nullptr) const override {
    if (H)
      *H = 2 * x.transpose();
    return Vector1(x.squaredNorm() - 1);
  }
};
} // namespace
TEST(AffineProjection, NonlinearEqualityKeepsAmbientRecovery) {
  auto p = weights();
  p->useVaryingSigma = false;
  p->priorSigma = 1;
  p->useReducedCoordinates = true;
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<BarrierRetractorCreator>(p);
  NonlinearFactorGraph factors;
  factors.emplace_shared<Circle>();
  auto equalities = std::make_shared<NonlinearEqualityConstraints>(
      NonlinearEqualityConstraints::FromCostGraph(factors));
  Values values;
  values.insert(kX, Vector(Vector2(1, 0)));
  IEConstraintManifold m(params, equalities,
                         std::make_shared<NonlinearInequalityConstraints>(),
                         values);
  VectorValues delta;
  delta.insert(kX, Vector2(0, .2));
  IERetractionInfo info;
  const auto result = m.retract(delta, {}, &info);
  EXPECT_DOUBLES_EQUAL(1, result.values().at<Vector>(kX).norm(), 1e-7);
  EXPECT_LONGS_EQUAL(0, info.numPenaltySolveCalls);
  EXPECT(m.eBasis() != result.eBasis());
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
