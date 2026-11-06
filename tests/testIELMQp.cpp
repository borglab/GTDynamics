/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file testIELMQp.cpp
 *  @brief Behavioral checks for the global B-coordinate tangent QP.
 *  @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <limits>

namespace tangent_qp {
using namespace gtsam;
using namespace gtdynamics;
IELMState makeState(const NonlinearFactorGraph& graph,
                    const NonlinearInequalityConstraints& inequalities,
                    const Values& initial,
                    const NonlinearEqualityConstraints& equalities = {}) {
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<BarrierRetractorCreator>();
  const auto manifolds =
      IEOptimizer::identifyManifolds(equalities, inequalities, initial, params);
  const auto free = IEOptimizer::identifyUnconstrainedValues(
      equalities, inequalities, initial);
  const auto manifoldGraph = ManifoldOptimizer::manifoldGraph(
      graph, IEOptimizer::varToManifoldKeyMap(manifolds));
  return IELMState(manifolds, free, graph, manifoldGraph, 0.2, 10.0);
}

NonlinearInequalityConstraint::shared_ptr bound(Key key, const Matrix& rows,
                                                const Vector& rhs,
                                                const Vector& sigmas) {
  return LinearConstraint::LessEqual(JacobianFactor(key, rows, rhs), sigmas)
      .createInequalityFactor();
}

// Exhaust all faces of two nonnegative 2-vectors. Each candidate is a hard
// equality-constrained QR solve, independent of GTSAM's active-set algorithm.
VectorValues exhaustiveOptimum(const GaussianFactorGraph& cost) {
  NonlinearOptimizerParams params;
  params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_QR;
  VectorValues best;
  double bestError = std::numeric_limits<double>::infinity();
  for (unsigned mask = 0; mask < 16; ++mask) {
    auto graph = cost;
    for (unsigned row = 0; row < 4; ++row) {
      if (!(mask & (1u << row))) continue;
      const Matrix12 normal = -Matrix2::Identity().row(row % 2);
      graph.emplace_shared<JacobianFactor>(row / 2 + 1, normal, Vector1::Zero(),
                                           noiseModel::Constrained::All(1));
    }
    const auto candidate = SolveLinear(graph, params);
    if (candidate.at(1).minCoeff() < -1e-8 ||
        candidate.at(2).minCoeff() < -1e-8)
      continue;
    if (cost.error(candidate) < bestError) {
      bestError = cost.error(candidate);
      best = candidate;
    }
  }
  return best;
}

// Check the global optimum against exhaustive constrained QR candidates.
TEST(IELMQp, CoupledComponentsAndFreeVariables) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(1, Vector(Vector2(-1., 2.)),
                                            noiseModel::Unit::Create(2));
  graph.emplace_shared<BetweenFactor<Vector>>(1, 2, Vector(Vector2(2., -3.)),
                                              noiseModel::Unit::Create(2));
  graph.emplace_shared<BetweenFactor<Vector>>(2, 3, Vector(Vector2(-1., 2.)),
                                              noiseModel::Unit::Create(2));
  Values initial;
  for (Key key : {1, 2, 3}) initial.insert(key, Vector(Vector2::Zero()));
  NonlinearInequalityConstraints inequalities;
  for (Key key : {1, 2})
    inequalities.push_back(
        bound(key, -Matrix2::Identity(), Vector2::Zero(), Vector2(0.3, 7.)));
  const auto state = makeState(graph, inequalities, initial);
  EXPECT_LONGS_EQUAL(0, state.tangentQp.eConstraints().size());
  EXPECT_LONGS_EQUAL(2, state.tangentQp.iConstraints().size());
  EXPECT_LONGS_EQUAL(2, state.qpConstraintBlocks.size());
  auto cost = *state.linearManifoldGraph;
  for (Key key : {1, 2, 3})
    cost.emplace_shared<JacobianFactor>(
        key, Matrix2::Identity(), Vector2::Zero(),
        noiseModel::Isotropic::Sigma(2, 1. / std::sqrt(0.2)));
  const IELMTrial::LinearUpdate update(0.2, graph, state, {});
  EXPECT(update.solveSuccessful);
  if (!update.solveSuccessful) return;
  EXPECT(assert_equal(exhaustiveOptimum(cost), update.tangentVector, 1e-8));
  EXPECT_DOUBLES_EQUAL(state.linearManifoldGraph->error(initial.zeroVectors()) -
                           state.linearManifoldGraph->error(update.delta),
                       update.costChange, 1e-9);
}

// Check cost compaction preserves the complete whitened quadratic model.
TEST(IELMQp, SameScopeCostsAreCompactedWithoutChangingTheModel) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector2(1., 2.)),
                                            noiseModel::Unit::Create(2));
  graph.emplace_shared<PriorFactor<Vector>>(
      7, Vector(Vector2(-3., 4.)),
      noiseModel::Diagonal::Sigmas(Vector2(2., 0.5)));
  Values initial;
  initial.insert(7, Vector(Vector2::Zero()));
  const auto state = makeState(graph, {}, initial);
  EXPECT_LONGS_EQUAL(2, state.linearManifoldGraph->size());
  EXPECT_LONGS_EQUAL(1, state.tangentQp.costs().size());
  for (const Vector& step :
       {Vector(Vector2::Zero()), Vector(Vector2(0.4, -0.7))}) {
    Values qpValues;
    qpValues.insert(7, step);
    VectorValues delta;
    delta.insert(7, step);
    EXPECT_DOUBLES_EQUAL(state.linearManifoldGraph->error(delta),
                         state.tangentQp.costs().error(qpValues), 1e-9);
  }
}

// Retain original indices and avoid promoting a partially active block.
TEST(IELMQp, PartialBlockAndOriginalConstraintIndices) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector2(1., -2.)),
                                            noiseModel::Unit::Create(2));
  Values initial;
  initial.insert(7, Vector(Vector2::Zero()));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(bound(7, Matrix2::Identity(), Vector2::Constant(10.),
                               Vector2::Ones()));  // inactive block 0
  inequalities.push_back(bound(7, -Matrix2::Identity(), Vector2::Zero(),
                               Vector2(2., 3.)));  // active block 1
  const auto state = makeState(graph, inequalities, initial);
  EXPECT_LONGS_EQUAL(1, state.tangentQp.iConstraints().size());
  EXPECT_LONGS_EQUAL(1, state.qpConstraintBlocks.at(0).constraint);
  const auto& block = state.manifolds.at(7).linearization()->rows(1, true);
  EXPECT(assert_equal((Matrix2() << -0.5, 0., 0., -1. / 3.).finished(), block));
  const IELMTrial::LinearUpdate update(0.2, graph, state, {});
  EXPECT(update.solveSuccessful);
  if (!update.solveSuccessful) return;
  EXPECT(assert_equal(Vector2(1. / 1.2, 0.), update.tangentVector.at(7), 1e-9));
  EXPECT(update.blockingIndicesMap.empty());
}

// Omit slack boundaries from the QP and restore their nonlinear feasibility.
TEST(IELMQp, SlackBoundaryIsLeftToRetraction) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector1(2.)),
                                            noiseModel::Unit::Create(1));
  Values initial;
  initial.insert(7, Vector(Vector1::Zero()));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      bound(7, Matrix1::Identity(), Vector1(1.), Vector1(1.)));
  const auto state = makeState(graph, inequalities, initial);
  EXPECT(state.tangentQp.iConstraints().empty());
  const IELMTrial::LinearUpdate update(0.2, graph, state, {});
  EXPECT(update.solveSuccessful);
  if (!update.solveSuccessful) return;
  EXPECT_DOUBLES_EQUAL(2. / 1.2, update.tangentVector.at(7)(0), 1e-9);
  const IELMTrial::NonlinearUpdate retracted(state, update, graph);
  const auto& manifold = retracted.newManifolds.at(7);
  EXPECT(manifold.values().at<Vector>(7)(0) <= 1. + 1e-3);
  EXPECT(manifold.evaluateInequalityViolation() <= 1e-3);
}

// Preserve diagonal damping while predicting reduction without damping.
TEST(IELMQp, DiagonalDampingAndUndampedPrediction) {
  NonlinearFactorGraph graph;
  const Vector target = Vector2(2., -3.);
  const Vector sigmas = Vector2(0.5, 2.);
  graph.emplace_shared<PriorFactor<Vector>>(
      7, target, noiseModel::Diagonal::Sigmas(sigmas));
  Values initial;
  initial.insert(7, Vector(Vector2::Zero()));
  const auto state = makeState(graph, {}, initial);
  IELMParams params;
  params.lmParams.setDiagonalDamping(true);
  const IELMTrial::LinearUpdate update(0.2, graph, state, params);
  EXPECT(update.solveSuccessful);
  if (!update.solveSuccessful) return;
  EXPECT(assert_equal(Vector(target / 1.2), update.tangentVector.at(7), 1e-9));
  const auto zero = initial.zeroVectors();
  EXPECT_DOUBLES_EQUAL(state.baseLinear->error(zero) -
                           state.baseLinear->error(update.tangentVector),
                       update.costChange, 1e-9);
  EXPECT_LONGS_EQUAL(state.linearManifoldGraph->size(),
                     state.tangentQp.costs().size());
}

// Keep equality directions eliminated and restore fixed ambient components.
TEST(IELMQp, EqualityReductionAndZeroDimensionalComponent) {
  NonlinearFactorGraph graph;
  Values initial;
  NonlinearEqualityConstraints equalities;
  for (Key key : {7, 8}) {
    initial.insert(key, Vector(Vector2::Zero()));
    graph.emplace_shared<PriorFactor<Vector>>(key, Vector(Vector2(2., 3.)),
                                              noiseModel::Unit::Create(2));
  }
  equalities.push_back(
      LinearConstraint::Equal(
          JacobianFactor(7, Matrix2::Identity(), Vector2::Zero()))
          .createEqualityFactor());
  equalities.push_back(
      LinearConstraint::Equal(
          JacobianFactor(8, (Matrix12() << 1., 0.).finished(), Vector1::Zero()))
          .createEqualityFactor());
  const auto state = makeState(graph, {}, initial, equalities);
  EXPECT(state.tangentQp.eConstraints().empty());
  const IELMTrial::LinearUpdate update(0.2, graph, state, {});
  EXPECT(update.solveSuccessful);
  if (!update.solveSuccessful) return;
  EXPECT(assert_equal(Vector2::Zero(), update.tangentVector.at(7), 1e-9));
  EXPECT(assert_equal(Vector2(0., 3. / 1.2), update.tangentVector.at(8), 1e-9));
}

// An entirely fixed problem, including a tight inequality, needs no QP call.
TEST(IELMQp, FullyFixedProblem) {
  Values initial;
  initial.insert(7, Vector(Vector1::Zero()));
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector1(2.)),
                                            noiseModel::Unit::Create(1));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(
      LinearConstraint::Equal(
          JacobianFactor(7, Matrix1::Identity(), Vector1::Zero()))
          .createEqualityFactor());
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      bound(7, Matrix1::Identity(), Vector1::Zero(), Vector1::Ones()));
  const auto state = makeState(graph, inequalities, initial, equalities);
  EXPECT(state.qpConstraintBlocks.empty());
  const IELMTrial::LinearUpdate update(0.2, graph, state, {});
  EXPECT(update.solveSuccessful);
  EXPECT_LONGS_EQUAL(0, update.qpIterations);
  EXPECT_DOUBLES_EQUAL(0., update.costChange, 0.);
  EXPECT(assert_equal(initial.zeroVectors(), update.tangentVector));
}

// Gate the replacement on dependent-row robustness in the production path.
TEST(IELMQp, DependentAndOpposingRows) {
  for (const bool opposing : {false, true}) {
    Values initial;
    initial.insert(7, Vector(Vector1::Zero()));
    NonlinearFactorGraph graph;
    graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector1(2.)),
                                              noiseModel::Unit::Create(1));
    NonlinearInequalityConstraints inequalities;
    const Matrix rows = (Matrix(2, 1) << -1., opposing ? 1. : -2.).finished();
    inequalities.push_back(bound(7, rows, Vector2::Zero(), Vector2::Ones()));
    const auto state = makeState(graph, inequalities, initial);
    const IELMTrial::LinearUpdate update(0.2, graph, state, {});
    EXPECT(update.solveSuccessful);
    if (update.solveSuccessful)
      EXPECT_DOUBLES_EQUAL(opposing ? 0. : 2. / 1.2,
                           update.tangentVector.at(7)(0), 1e-9);
  }
}

// Reject a singular undamped solve and allow recovery with LM damping.
TEST(IELMQp, SingularTrialIsRejected) {
  Values initial;
  initial.insert(7, Vector(Vector1::Zero()));
  initial.insert(8, Vector(Vector1::Zero()));
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Vector>>(7, 8, Vector(Vector1(1.)),
                                              noiseModel::Unit::Create(1));
  const auto state = makeState(graph, {}, initial);
  const IELMTrial::LinearUpdate singular(0., graph, state, {});
  EXPECT(!singular.solveSuccessful);
  const IELMTrial::LinearUpdate damped(0.2, graph, state, {});
  EXPECT(damped.solveSuccessful);
}

// Report exhausted QP retries instead of returning false convergence.
TEST(IELMQp, ExhaustedRetriesAreNotConvergence) {
  Values initial;
  initial.insert(7, Vector(Vector1::Zero()));
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector1(2.)),
                                            noiseModel::Unit::Create(1));
  const auto state = makeState(graph, {}, initial);
  IELMParams params;
  params.qpMaxIterations = 0;
  params.lmParams.lambdaUpperBound = 0.2;
  auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
  manifoldParams->retractorCreator =
      std::make_shared<BarrierRetractorCreator>();
  IELMOptimizer optimizer(params, manifoldParams);
  bool threw = false;
  try {
    optimizer.iterate(graph, state);
  } catch (const std::runtime_error& error) {
    threw = std::string(error.what()).find("exhausted damping trials") !=
            std::string::npos;
  }
  EXPECT(threw);
}

// Feasible descent can stop on the projected face before a full solve releases
// it.
TEST(IELMQp, ProjectedInitializationAndFeasibleStepModes) {
  NonlinearFactorGraph graph;
  const Matrix2 informationRoot = (Matrix2() << 1., -2., 0., 1.).finished();
  graph.emplace_shared<PriorFactor<Vector>>(
      7, Vector(Vector2(5., 3.)),
      noiseModel::Gaussian::SqrtInformation(informationRoot));
  Values initial;
  initial.insert(7, Vector(Vector2::Zero()));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      bound(7, -Matrix2::Identity(), Vector2::Zero(), Vector2(0.3, 7.)));
  const auto state = makeState(graph, inequalities, initial);
  const IELMTrial::LinearUpdate full(0.2, graph, state, {});
  EXPECT(full.solveSuccessful);
  for (bool projected : {false, true}) {
    IELMParams params;
    params.qpUseProjectedGradientInitialization = projected;
    const IELMTrial::LinearUpdate seeded(0.2, graph, state, params);
    EXPECT(seeded.solveSuccessful);
    EXPECT(assert_equal(full.delta, seeded.delta, 1e-9));
    params.qpUseFeasibleStep = true;
    const IELMTrial::LinearUpdate feasible(0.2, graph, state, params);
    EXPECT(feasible.solveSuccessful);
    if (!feasible.solveSuccessful) continue;
    EXPECT_LONGS_EQUAL(1, feasible.qpIterations);
    EXPECT(assert_equal(Vector2(0., 5. / 5.2), feasible.tangentVector.at(7),
                        1e-9));
    EXPECT(feasible.costChange > 0);
    EXPECT(full.newError < feasible.newError);
    EXPECT(IELMTrial::LinearUpdate::checkSolutionValid(
        state, feasible.delta, feasible.blockingIndicesMap));
    // Only one row is active, so the vector block stays an inequality.
    EXPECT(feasible.blockingIndicesMap.empty());
    EXPECT_DOUBLES_EQUAL(
        state.linearManifoldGraph->error(initial.zeroVectors()) -
            state.linearManifoldGraph->error(feasible.delta),
        feasible.costChange, 1e-9);
  }
}

// No progress supplies a zero trial, while exhaustion and singularity reject
// it.
TEST(IELMQp, FeasibleStepNoProgressAndFailures) {
  Values initial;
  initial.insert(7, Vector(Vector1::Zero()));
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(7, Vector(Vector1(-2.)),
                                            noiseModel::Unit::Create(1));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      bound(7, -Matrix1::Identity(), Vector1::Zero(), Vector1::Ones()));
  const auto state = makeState(graph, inequalities, initial);
  IELMParams params;
  params.qpUseFeasibleStep = true;
  const IELMTrial::LinearUpdate stopped(0.2, graph, state, params);
  EXPECT(stopped.solveSuccessful);
  EXPECT(assert_equal(initial.zeroVectors(), stopped.tangentVector, 1e-9));
  EXPECT_DOUBLES_EQUAL(0., stopped.costChange, 1e-9);
  params.qpMaxIterations = 0;
  const IELMTrial::LinearUpdate exhausted(0.2, graph, state, params);
  EXPECT(!exhausted.solveSuccessful);
  EXPECT(!exhausted.failureReason.empty());
  params.qpMaxIterations = 1000;
  initial.insert(8, Vector(Vector1::Zero()));
  graph = NonlinearFactorGraph();
  graph.emplace_shared<BetweenFactor<Vector>>(7, 8, Vector(Vector1(1.)),
                                              noiseModel::Unit::Create(1));
  const auto singularState = makeState(graph, {}, initial);
  const IELMTrial::LinearUpdate singular(0., graph, singularState, params);
  EXPECT(!singular.solveSuccessful);
  EXPECT(!singular.failureReason.empty());
  const IELMTrial::LinearUpdate damped(0.2, graph, singularState, params);
  EXPECT(damped.solveSuccessful);
}

}  // namespace tangent_qp

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
