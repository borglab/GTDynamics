/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testIEManifoldOptimizer.cpp
 * @brief Regression tests for IEManifoldOptimizer.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */


#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/scenarios/CartPoleWithFrictionRetractor.h>
#include <gtdynamics/scenarios/DomeRetractor.h>
#include <gtdynamics/scenarios/HalfSphereRetractor.h>
#include "gtdynamics/cmcopt/IERetractor.h"
#include "gtdynamics/cmopt/TangentSpaceBasis.h"
#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/constrained/LinearConstraint.h>

using namespace gtdynamics;
using namespace gtsam;

TEST(identifyManifolds, HalfSphere) {
  IEHalfSphere half_sphere;
  size_t num_steps = 2;

  NonlinearEqualityConstraints e_constraints;
  NonlinearInequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    e_constraints.add(half_sphere.eConstraints(k));
    i_constraints.add(half_sphere.iConstraints(k));
  }

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(PointKey(k), Point3(0, 1, 0));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<HalfSphereRetractor>(half_sphere));
  iecm_params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  auto manifolds = IEOptimizer::identifyManifolds(e_constraints, i_constraints,
                                                  values, iecm_params);
  for (const auto &it : manifolds) {
    const auto &manifold = it.second;
    EXPECT_LONGS_EQUAL(1, manifold.iConstraints()->size());
    EXPECT_LONGS_EQUAL(1, manifold.eConstraints()->size());
    EXPECT_LONGS_EQUAL(1, manifold.values().size());
  }
  EXPECT_LONGS_EQUAL(num_steps + 1, manifolds.size());
}

TEST(identifyManifolds, CartPoleWithFriction) {
  IECartPoleWithFriction cp;
  size_t num_steps = 2;

  NonlinearEqualityConstraints e_constraints;
  NonlinearInequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    e_constraints.add(cp.eConstraints(k));
    i_constraints.add(cp.iConstraints(k));
  }

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(cp.defaultValues(k));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<CartPoleWithFrictionRetractor>(cp));
  iecm_params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  auto manifolds = IEOptimizer::identifyManifolds(e_constraints, i_constraints,
                                                  values, iecm_params);
  for (const auto &it : manifolds) {
    const auto &manifold = it.second;
    EXPECT_LONGS_EQUAL(2, manifold.iConstraints()->size());
    EXPECT_LONGS_EQUAL(3, manifold.eConstraints()->size());
    EXPECT_LONGS_EQUAL(6, manifold.values().size());
  }
  EXPECT_LONGS_EQUAL(num_steps + 1, manifolds.size());
}

TEST(identifyManifolds, Dome) {
  IEHalfSphere half_sphere;
  size_t num_steps = 2;

  NonlinearEqualityConstraints e_constraints;
  NonlinearInequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    i_constraints.add(half_sphere.iDomeConstraints(k));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<DomeRetractor>(half_sphere));
  iecm_params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(PointKey(k), Point3(0, 1, 0));
  }

  auto manifolds = IEOptimizer::identifyManifolds(e_constraints, i_constraints,
                                                  values, iecm_params);
  for (const auto &it : manifolds) {
    const auto &manifold = it.second;
    EXPECT_LONGS_EQUAL(2, manifold.iConstraints()->size());
    EXPECT_LONGS_EQUAL(0, manifold.eConstraints()->size());
    EXPECT_LONGS_EQUAL(1, manifold.values().size());
  }
  EXPECT_LONGS_EQUAL(num_steps + 1, manifolds.size());
}

// Build a real CMCOpt state at a unilateral contact boundary. Force that
// boundary into the initial working set to exercise release during refinement.
static IELMState unilateralContactState(double target = 4.0) {
  const Key key = 5;
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<double>>(
      key, target, noiseModel::Isotropic::Sigma(1, 2));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(ScalarExpressionInequalityConstraint::GeqZero(
      Expression<double>(key), 1.0));
  Values initial;
  initial.insert(key, 0.0);
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<BarrierRetractorCreator>();
  const auto manifolds = IEOptimizer::identifyManifolds({}, inequalities,
                                                        initial, params);
  const auto manifoldGraph = ManifoldOptimizer::manifoldGraph(
      graph, IEOptimizer::varToManifoldKeyMap(manifolds));
  IELMState state(manifolds, {}, graph, manifoldGraph, 0.5, 10.0);
  state.gradientBlockingIndicesMap[key].insert(0);
  return state;
}

TEST(IELMLinearUpdate, ReleaseUnilateralContactWithGtsam) {
  const auto state = unilateralContactState();
  IELMParams params;
  params.iqpMaxIterations = 100;
  const IELMTrial::LinearUpdate update(0.5, {}, state, params);
  EXPECT(update.solveSuccessful);
  // Preserve both the prior precision 1/4 and LM damping 1/2.
  EXPECT_DOUBLES_EQUAL(4.0 / 3.0, update.tangentVector.at(5)(0), 1e-8);
  EXPECT(update.blockingIndicesMap.empty());
  EXPECT(update.numSolves > 1);
}

TEST(IELMLinearUpdate, GtsamIterationExhaustionIsFailure) {
  const auto state = unilateralContactState();
  IELMParams params;
  // A single iteration releases the boundary; it cannot certify the new step.
  params.iqpMaxIterations = 1;
  const IELMTrial::LinearUpdate update(0.5, {}, state, params);
  EXPECT(!update.solveSuccessful);
}

TEST(IELMLinearUpdate, GtsamWorkingSetPreservesConstraintIndices) {
  const auto state = unilateralContactState(-4.0);
  IELMParams params;
  params.iqpMaxIterations = 100;
  const IELMTrial::LinearUpdate update(0.5, {}, state, params);
  EXPECT(update.solveSuccessful);
  EXPECT_DOUBLES_EQUAL(0.0, update.tangentVector.at(5)(0), 1e-8);
  EXPECT_LONGS_EQUAL(1, update.blockingIndicesMap.size());
  EXPECT(update.blockingIndicesMap.at(5).exists(0));
}

// Coupling makes the projected gradient initially block x although the QP
// optimum can require releasing it. Exercise both inequality senses and KKT
// multiplier signs through the production CMC-to-GTSAM path.
TEST(IELMLinearUpdate, CoupledBoundsWithBothSenses) {
  const Key key = 7;
  const double damping = 0.01;
  for (const bool lowerBound : {false, true}) {
    for (const bool release : {false, true}) {
      const double sign = lowerBound ? 1.0 : -1.0;
      Matrix2 information;
      information << 1.0, sign * 0.9, sign * 0.9, 1.0;
      const Vector target = Vector2(sign * (release ? 1.0 : -1.0), -2.0);
      NonlinearFactorGraph graph;
      graph.emplace_shared<PriorFactor<Vector>>(
          key, target, noiseModel::Gaussian::Information(information));
      const JacobianFactor face(
          key, (Matrix(1, 2) << 1.0, 0.0).finished(), Vector1(0.0));
      NonlinearInequalityConstraints inequalities;
      inequalities.push_back(
          (lowerBound ? LinearConstraint::GreaterEqual(face, Vector1(3.0))
                      : LinearConstraint::LessEqual(face, Vector1(3.0)))
              .createInequalityFactor());
      Values initial;
      initial.insert(key, Vector(Vector2::Zero()));
      auto manifoldParams = std::make_shared<IEConstraintManifold::Params>();
      manifoldParams->equalityBasisCreator =
          std::make_shared<OrthonormalBasisCreator>();
      manifoldParams->retractorCreator =
          std::make_shared<BarrierRetractorCreator>();
      const auto manifolds = IEOptimizer::identifyManifolds(
          {}, inequalities, initial, manifoldParams);
      const auto manifoldGraph = ManifoldOptimizer::manifoldGraph(
          graph, IEOptimizer::varToManifoldKeyMap(manifolds));
      const IELMState state(manifolds, {}, graph, manifoldGraph, damping, 10.0);
      EXPECT(state.gradientBlockingIndicesMap.at(key).exists(0));
      IELMParams params;
      params.iqpMaxIterations = 100;
      const IELMTrial::LinearUpdate update(damping, graph, state, params);
      EXPECT(update.solveSuccessful);
      const Matrix2 hessian = information + damping * Matrix2::Identity();
      const Vector2 rhs = information * target;
      Vector expected = hessian.ldlt().solve(rhs);
      if (!release) expected = Vector2(0.0, rhs(1) / hessian(1, 1));
      EXPECT(assert_equal(expected, update.tangentVector.at(key), 1e-8));
      if (release) {
        EXPECT(update.blockingIndicesMap.empty());
      } else {
        EXPECT(update.blockingIndicesMap.at(key).exists(0));
      }
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
