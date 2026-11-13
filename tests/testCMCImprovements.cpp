/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testCMCImprovements.cpp
 * @brief Verify cone scaling, projection failure handling and history
 * retention.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/constrained/TangentCone.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/PoseContactConstraintSpec.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <atomic>
using namespace gtsam;
using namespace gtdynamics;

/* ************************************************************************* */
// Large gradients and differently scaled rows describe the same homogeneous
// cone; projection must preserve both its solution and its blocking faces.
TEST(CMCImprovements, ScaledConeProjection) {
  LinearInequalityConstraints constraints;
  const Matrix22 A{{1e6, -2e6}, {-2e-6, 1e-6}};
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, A, Vector2::Zero()));
  const auto [active, result] =
      TangentCone(constraints).project(Vector2(1e9, 0));
  EXPECT(assert_equal(Vector2(.8e9, .4e9), result, 1e-4));
  EXPECT(result(0) - 2 * result(1) < 1e-4);
}

/* ************************************************************************* */
namespace projection_retry {
class LimitedRetractor : public IERetractor {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<LimitedRetractor>(*this);
  }

 public:
  std::shared_ptr<std::atomic<size_t>> rejected =
      std::make_shared<std::atomic<size_t>>(0);
  IECMPoint retract(
      const IECMPoint* point, const VectorValues& delta,
      const std::optional<IndexSet>& blocking = {},
      IERetractionInfo* info = nullptr) const override {
    if (delta.at(1).norm() > .2) {
      ++*rejected;
      throw RetractionFailure("Synthetic projection convergence failure");
    }
    return point->manifold()->point(point->bindings(), point->values().retract(delta));
  }
};
// Failed projections reject the trial and increase damping. Bounded details
// preserve all scalar callbacks while retaining at most one previous iterate.
TEST(CMCImprovements, RejectProjectionAndRetry) {
  Values values;
  values.insert(Key(1), .1);
  NonlinearFactorGraph cost;
  cost.emplace_shared<PriorFactor<double>>(1, 1., noiseModel::Unit::Create(1));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(ScalarExpressionInequalityConstraint::LeqZero(
      Double_(0.) - Double_(Key(1)), 1.));
  const auto retractor = std::make_shared<LimitedRetractor>();
  const auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
  manifoldParams->equalityBasisCreator =
      std::make_shared<OrthonormalBasisCreator>();
  manifoldParams->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  IELMParams params;
  params.historyWindow = 1;
  params.lmParams.maxIterations = 5;
  size_t observations = 0;
  params.iterationCallback = [&observations](const IELMState&) {
    ++observations;
  };
  const IELMOptimizer optimizer(params, manifoldParams);
  const auto result = optimizer.optimize(cost, {}, inequalities, values);
  EXPECT(retractor->rejected->load() > 0);
  EXPECT(result.at<double>(1) > .1);
  EXPECT(result.at<double>(1) <= 1.);
  EXPECT(inequalities.violationNorm(result) < 1e-10);
  EXPECT_LONGS_EQUAL(optimizer.details().back().state.iterations + 1,
                     observations);
  EXPECT(optimizer.details().size() <= 2);
}
}  // namespace projection_retry
/* ************************************************************************* */
namespace history_retention {
class PositiveRetractor : public IERetractor {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<PositiveRetractor>(*this);
  }

 public:
  std::shared_ptr<std::atomic<size_t>> boundaryMoves =
      std::make_shared<std::atomic<size_t>>(0);
  IECMPoint retract(const IECMPoint* point,
                               const VectorValues& d,
                               const std::optional<IndexSet>& blocking = {},
                               IERetractionInfo* = nullptr) const override {
    Values v = point->values().retract(d);
    v.update(Key(1), blocking && !blocking->empty()
                         ? 0.
                         : std::max(0., v.at<double>(1)));
    return point->manifold()->point(point->bindings(), v);
  }
  IECMPoint moveToBoundary(
      const IECMPoint* point, const IndexSet& blocking,
      IERetractionInfo* info = nullptr) const override {
    ++*boundaryMoves;
    return retract(point, point->values().zeroVectors(), blocking, info);
  }
};
double logarithm(const double& x, OptionalJacobian<1, 1> H) {
  if (H) (*H)(0, 0) = 1. / (x + 1e-8);
  return std::log(x + 1e-8);
}
// A steep logarithmic residual rejects boundary-crossing trials while accepted
// interior steps approach the boundary. This exercises forced mode changes,
// including the older mode origin that a one-entry diagnostic window discards.
TEST(IELMOptimizer, HistoryDoesNotChangeBoundaryDecisions) {
  std::vector<Values> reference;
  size_t referenceMoves = 0;
  for (size_t window : {0, 1, 3}) {
    Values initial;
    initial.insert(Key(1), 1.);
    NonlinearFactorGraph graph;
    graph.emplace_shared<ExpressionFactor<double>>(
        noiseModel::Unit::Create(1), std::log(.01 + 1e-8),
        Double_(logarithm, Double_(Key(1))));
    NonlinearInequalityConstraints constraints;
    constraints.push_back(
        ScalarExpressionInequalityConstraint::GeqZero(Double_(Key(1)), 1.));
    auto retractor = std::make_shared<PositiveRetractor>();
    auto mp = std::make_shared<IEConstraintManifold::Params>();
    mp->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
    mp->retractorCreator =
        std::make_shared<UniversalIERetractorCreator>(retractor);
    IELMParams p;
    p.historyWindow = window;
    p.lmParams.maxIterations = 8;
    p.boundaryApproachRateThreshold = 10.;
    p.lmParams.relativeErrorTol = 0;
    p.lmParams.absoluteErrorTol = 0;
    std::vector<Values> states;
    p.iterationCallback = [&](const IELMState& state) {
      states.push_back(state.baseValues());
    };
    IELMOptimizer optimizer(p, mp);
    optimizer.optimize(graph, {}, constraints, initial);
    if (window == 0) {
      reference = states;
      referenceMoves = retractor->boundaryMoves->load();
    }
    EXPECT(referenceMoves > 0);
    EXPECT_LONGS_EQUAL(referenceMoves, retractor->boundaryMoves->load());
    EXPECT_LONGS_EQUAL(reference.size(), states.size());
    for (size_t i = 0; i < std::min(reference.size(), states.size()); ++i)
      EXPECT(assert_equal(reference[i], states[i], 1e-12));
    if (window) EXPECT(optimizer.details().size() <= window + 1);
  }
}
}  // namespace history_retention

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
