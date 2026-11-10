/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testIELMEqualityOnly.cpp
 * @brief Compare Gaussian and QP steps through the same CMC optimizer.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/internal/CompiledIEConstraintManifold.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "constrainedExample.h"

using namespace gtsam;
using namespace gtdynamics;

namespace {
IELMState makeState(const NonlinearFactorGraph& cost,
                    const NonlinearEqualityConstraints& equalities,
                    const Values& initial,
                    const NonlinearInequalityConstraints& inequalities = {}) {
  const auto components = IEOptimizer::identifyConnectedComponents(
      equalities, inequalities, initial,
      std::make_shared<IEConstraintManifold::Params>());
  const auto free = IEOptimizer::identifyUnconstrainedValues(
      equalities, inequalities, initial);
  return IELMState(components.points, free, cost,
                   IEOptimizer::pointGraph(cost, components.points), .2, 10);
}

// The affine offset, fixed component and free variable survive either solve.
TEST(IELMEqualityOnly, CoupledStepsDampingAndOrdering) {
  Values initial;
  for (Key key : {1, 2, 3, 4})
    initial.insert(key, Vector(Vector1(key == 2 ? 3 : key == 3 ? 4 : 0)));
  NonlinearEqualityConstraints equalities;
  equalities.push_back(
      LinearConstraint::Equal(JacobianFactor(1, Matrix1::Constant(-2), 2,
                                             Matrix1::Identity(), Vector1(3)))
          .createEqualityFactor());
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(3, Matrix1::Identity(), Vector1(4)))
                           .createEqualityFactor());
  NonlinearFactorGraph cost;
  const auto unit = noiseModel::Unit::Create(1);
  cost.emplace_shared<PriorFactor<Vector>>(2, Vector(Vector1(9)), unit);
  cost.emplace_shared<PriorFactor<Vector>>(3, Vector(Vector1(9)), unit);
  cost.emplace_shared<BetweenFactor<Vector>>(1, 4, Vector(Vector1(1)), unit);
  const auto state = makeState(cost, equalities, initial);
  EXPECT(state.equalityOnly);
  EXPECT(state.tangentQp.costs().empty());

  for (bool diagonal : {false, true}) {
    for (bool ordered : {false, true}) {
      IELMParams params;
      params.lmParams.setDiagonalDamping(diagonal);
      if (ordered) {
        Ordering ordering;
        for (const auto& [key, point] : state.points) ordering.push_back(key);
        ordering.push_back(4);
        std::reverse(ordering.begin(), ordering.end());
        params.lmParams.ordering = ordering;
      }
      const IELMTrial::LinearUpdate direct(.2, cost, state, params);
      params.linearSolver = IELMParams::LinearSolver::QP;
      const IELMTrial::LinearUpdate qp(.2, cost, state, params);
      EXPECT(direct.solveSuccessful && qp.solveSuccessful);
      EXPECT_LONGS_EQUAL(1, direct.gaussianSolves);
      EXPECT_LONGS_EQUAL(0, direct.qpSolves);
      EXPECT_LONGS_EQUAL(1, qp.qpSolves);
      EXPECT(assert_equal(direct.delta, qp.delta, 1e-9));
      EXPECT(assert_equal(direct.tangentVector, qp.tangentVector, 1e-9));
      EXPECT_DOUBLES_EQUAL(direct.costChange, qp.costChange, 1e-9);
      EXPECT_DOUBLES_EQUAL(0, direct.tangentVector.at(3)(0), 1e-12);
      EXPECT_DOUBLES_EQUAL(2 * direct.tangentVector.at(1)(0),
                           direct.tangentVector.at(2)(0), 1e-10);
      const IELMTrial::NonlinearUpdate next(state, direct, cost);
      Values recovered = ambientValues(next.newPoints);
      recovered.insert(next.newUnconstrainedValues);
      EXPECT(equalities.violationNorm(recovered) < 1e-7);
      EXPECT_DOUBLES_EQUAL(
          state.linearPointGraph->error(state.values.zeroVectors()) -
              state.linearPointGraph->error(direct.delta),
          direct.costChange, 1e-10);
#ifdef GTDYNAMICS_WITH_PROXQP
      params.qpBackend = IELMParams::QpBackend::ProxQP;
      const IELMTrial::LinearUpdate prox(.2, cost, state, params);
      EXPECT(prox.solveSuccessful);
      EXPECT(assert_equal(direct.tangentVector, prox.tangentVector, 1e-6));
#endif
    }
  }
}

// Inactive bounds still prohibit Gaussian selection; zero tangent rows do not
// make the nonlinear problem equality-only.
TEST(IELMEqualityOnly, RejectSlackInequalitiesAndInvalidOrdering) {
  Values initial;
  initial.insert(1, Vector(Vector1::Zero()));
  NonlinearFactorGraph cost;
  cost.emplace_shared<PriorFactor<Vector>>(1, Vector(Vector1(2)),
                                           noiseModel::Unit::Create(1));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(LinearConstraint::LessEqual(
                             JacobianFactor(1, Matrix1::Identity(), Vector1(1)))
                             .createInequalityFactor());
  const auto constrained = makeState(cost, {}, initial, inequalities);
  EXPECT(constrained.qpConstraintBlocks.empty());
  IELMParams params;
  params.linearSolver = IELMParams::LinearSolver::Gaussian;
  CHECK_EXCEPTION(IELMTrial::LinearUpdate(.2, cost, constrained, params),
                  std::invalid_argument);
  params.linearSolver = IELMParams::LinearSolver::Automatic;
  const IELMTrial::LinearUpdate automatic(.2, cost, constrained, params);
  EXPECT_LONGS_EQUAL(1, automatic.qpSolves);
  EXPECT_LONGS_EQUAL(0, automatic.gaussianSolves);
  const auto state = makeState(cost, {}, initial);
  params.lmParams.ordering = Ordering{};
  CHECK_EXCEPTION(IELMTrial::LinearUpdate(.2, cost, state, params),
                  std::invalid_argument);
  params.lmParams.ordering = Ordering{1, 1};
  CHECK_EXCEPTION(IELMTrial::LinearUpdate(.2, cost, state, params),
                  std::invalid_argument);
}

// A singular direct step rejects the trial, then succeeds with damping.
TEST(IELMEqualityOnly, SingularRetryAndFullyFixed) {
  Values initial;
  initial.insert(1, Vector(Vector1::Zero()));
  initial.insert(2, Vector(Vector1::Zero()));
  NonlinearFactorGraph cost;
  cost.emplace_shared<BetweenFactor<Vector>>(1, 2, Vector(Vector1(1)),
                                             noiseModel::Unit::Create(1));
  auto state = makeState(cost, {}, initial);
  const IELMTrial::LinearUpdate singular(0., cost, state, {});
  EXPECT(!singular.solveSuccessful);
  EXPECT(!singular.failureReason.empty());
  const IELMTrial::LinearUpdate damped(.2, cost, state, {});
  EXPECT(damped.solveSuccessful);
  NonlinearEqualityConstraints equalities;
  for (Key key : {1, 2})
    equalities.push_back(
        LinearConstraint::Equal(
            JacobianFactor(key, Matrix1::Identity(), Vector1::Zero()))
            .createEqualityFactor());
  state = makeState(cost, equalities, initial);
  const IELMTrial::LinearUpdate fixed(.2, cost, state, {});
  EXPECT(fixed.solveSuccessful);
  EXPECT_LONGS_EQUAL(0, fixed.gaussianSolves);
  EXPECT_LONGS_EQUAL(0, fixed.qpSolves);
  EXPECT(assert_equal(initial.zeroVectors(), fixed.tangentVector));
}

// A compiled scalar chart has no constraint factors, but uses the same solve.
struct EqualityLineRecovery {
  double fromAmbient(const Values& values) const {
    return values.at<Vector>(0)(0);
  }
  Values ambient(double x) const {
    Values values;
    values.insert(0, Vector(Vector1(x)));
    values.insert(1, Vector(Vector1(2 * x)));
    return values;
  }
  std::map<Key, Matrix> jacobians(double) const {
    return {{0, Matrix1::Identity()}, {1, Matrix1::Constant(2)}};
  }
  std::vector<gtdynamics::internal::NativeBoundary> boundaries(double) const {
    return {};
  }
  double boundary(double value, const IndexSet&) const { return value; }
};

TEST(IELMEqualityOnly, CompiledAndRuntimeCharts) {
  const EqualityLineRecovery recovery;
  const auto native =
      std::make_shared<gtdynamics::internal::CompiledIEConstraintManifold<
          double, EqualityLineRecovery>>(0., recovery);
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(0, Matrix1::Constant(-2), 1,
                                          Matrix1::Identity(), Vector1::Zero()))
                           .createEqualityFactor());
  IEConstraintManifold::Params settings;
  settings.affineEqualities = true;
  settings.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{0}; });
  const auto runtime = std::make_shared<IEConstraintManifold>(
      equalities, NonlinearInequalityConstraints{}, recovery.ambient(0),
      settings);
  Values initial;
  initial.insert(10, Vector(Vector1(.5)));
  initial.insert(11, Vector(Vector1(1)));
  NonlinearFactorGraph cost;
  cost.emplace_shared<PriorFactor<Vector>>(11, Vector(Vector1(4)),
                                           noiseModel::Unit::Create(1));
  std::optional<Values> expected;
  for (const IEConstraintManifold::shared_ptr& type :
       {IEConstraintManifold::shared_ptr(native),
        IEConstraintManifold::shared_ptr(runtime)}) {
    IECMPoints points{{100, type->point({{0, 10}, {1, 11}}, initial)}};
    for (auto solver :
         {IELMParams::LinearSolver::Gaussian, IELMParams::LinearSolver::QP}) {
      IELMParams params;
      params.linearSolver = solver;
      const auto result = IELMOptimizer(params).optimize(cost, points, {});
      EXPECT_DOUBLES_EQUAL(2., result.at<Vector>(10)(0), 1e-4);
      if (expected) EXPECT(assert_equal(*expected, result, 1e-8));
      expected = result;
    }
  }
}

// A finite-looking solve cannot admit nonfinite output into retraction.
TEST(IELMEqualityOnly, RejectNonfiniteStep) {
  Values initial;
  initial.insert(1, Vector(Vector1::Zero()));
  NonlinearFactorGraph cost;
  cost.emplace_shared<PriorFactor<Vector>>(1, Vector(Vector1(2)),
                                           noiseModel::Unit::Create(1));
  auto state = makeState(cost, {}, initial);
  state.reducedObjective.emplace_shared<JacobianFactor>(
      1, Matrix1::Identity(),
      Vector1::Constant(std::numeric_limits<double>::quiet_NaN()));
  const IELMTrial::LinearUpdate invalid(.2, cost, state, {});
  EXPECT(!invalid.solveSuccessful);
  EXPECT(!invalid.failureReason.empty());
}

// Nonlinear chart rebuilding and LM acceptance agree, beyond one linear step.
TEST(IELMEqualityOnly, NonlinearOptimizationAndTimings) {
  using namespace e_constrained_example;
  Values initial;
  const double x = -.2;
  initial.insert(x1_key, x);
  initial.insert(x2_key, (-1 + std::sqrt(1 - 4 * (x + x * x * x))) / 2);
  IELMParams params;
  params.qpMaxIterations = 0;  // The Gaussian path must never consult this.
  params.lmParams.absoluteErrorTol = 1e-12;
  params.lmParams.relativeErrorTol = 1e-12;
  IELMOptimizer direct(params);
  const auto a = direct.optimize(cost, constraints, {}, initial);
  EXPECT(direct.timings().gaussianSolves > 0);
  EXPECT_LONGS_EQUAL(0, direct.timings().qpSolves);
  EXPECT(direct.timings().linearizationSeconds > 0);
  EXPECT(direct.timings().linearSolveSeconds > 0);
  EXPECT(direct.timings().retractionSeconds > 0);
  params.qpMaxIterations = 1000;
  params.linearSolver = IELMParams::LinearSolver::QP;
  IELMOptimizer qp(params);
  const auto b = qp.optimize(cost, constraints, {}, initial);
  EXPECT(qp.timings().qpSolves > 0);
  EXPECT_LONGS_EQUAL(0, qp.timings().gaussianSolves);
  EXPECT(assert_equal(a, b, 1e-7));
  EXPECT(constraints.violationNorm(a) < 1e-7);
  EXPECT_DOUBLES_EQUAL(0, a.atDouble(x1_key), 1e-4);
  EXPECT_DOUBLES_EQUAL(cost.error(a), cost.error(b), 1e-10);
}
}  // namespace

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
