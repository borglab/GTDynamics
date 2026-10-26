/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/** @file testCableRobotUtils.cpp
 * @brief Regression coverage for the PR #363 physical problem and manifold
 * path.
 */
#include <CppUnitLite/TestHarness.h>
#include <examples/example_constraint_manifold/CableRobotBenchmark.h>
#include <gtdynamics/cablerobot/utils/CableRobotUtils.h>
#include <gtdynamics/cmopt/Retractor.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <cmath>

using namespace gtsam;
using namespace gtdynamics;

namespace {
size_t dimension(const NonlinearFactorGraph &graph) {
  size_t result = 0;
  for (const auto &factor : graph) result += factor->dim();
  return result;
}

NonlinearFactorGraph combine(
    const std::map<std::string, NonlinearFactorGraph> &groups) {
  NonlinearFactorGraph graph;
  for (const auto &[name, group] : groups) graph.add(group);
  return graph;
}
}  // namespace

TEST(CableRobotUtils, FormulationAndWeights) {
  CableRobotParams params;
  params.timeStep = 0.01;
  params.waypointWeight = 100;
  params.torqueWeight = 1;
  CableRobot robot(params);
  constexpr int numSteps = 3;
  auto initialValues = robot.initialValues(numSteps);
  std::map<int, Pose3> waypoints;
  for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex)
    waypoints.emplace(stepIndex,
                      Pose3(Rot3(), Point3(1.5 + 0.1 * stepIndex, 1, 0)));
  const auto costs = robot.costFactors(numSteps, waypoints);
  const auto constraints = robot.constraintFactors(numSteps);
  EXPECT_LONGS_EQUAL(62 * numSteps, initialValues.dim());
  EXPECT_LONGS_EQUAL(46 * numSteps + 12, dimension(constraints));
  EXPECT_LONGS_EQUAL(22 * numSteps - 12, dimension(combine(costs)));
  DOUBLES_EQUAL(2.5, costs.at("waypoint").error(initialValues), 1e-10);
  DOUBLES_EQUAL(0, costs.at("minTorque").error(initialValues), 1e-12);
  // A torque perturbation contributes exactly 1/2 R delta^2.
  initialValues.update(TorqueKey(0, 1), 0.8);
  DOUBLES_EQUAL(0.02, costs.at("minTorque").error(initialValues), 1e-12);
  // Each equality component stays within one time slice: collocation is soft.
  for (const auto &factor : constraints) {
    const size_t time = DynamicsSymbol(factor->keys().front()).time();
    for (Key key : factor->keys()) EXPECT(time == DynamicsSymbol(key).time());
  }
  // Moving the middle pose by .02 adds two .02/sigma residuals, one on each
  // adjacent interval. Fixed-time trapezoidal twist residuals remain zero.
  initialValues.update(PoseKey(params.endEffectorId, 1),
                       Pose3(Rot3(), Point3(1.52, 1, 0)));
  DOUBLES_EQUAL(400, costs.at("Collocation").error(initialValues), 1e-8);
}

TEST(CableRobotUtils, ZeroGravityEquilibriumAndInitialGuess) {
  CableRobotParams params;
  CableRobot robot(params);
  auto values = robot.initialValues(2);
  const auto constraints = robot.constraintFactors(2);
  EXPECT(constraints.error(values) > 1);
  // Construct equilibrium independently from the cable direction and lever arm.
  // Equal tensions cancel both force and moment at the symmetric center.
  const double tension = params.torquePrior / params.winchParams.radius_;
  for (int stepIndex = 0; stepIndex < 2; ++stepIndex) {
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex) {
      const Point3 mount = params.endEffectorMountPoints[cableIndex];
      const Vector3 displacement = params.initialPose.transformFrom(mount) -
                                   params.frameMountPoints[cableIndex];
      const Vector3 force = -tension * displacement.normalized();
      const Vector6 wrench =
          (Vector6() << mount.cross(force), force).finished();
      values.update(JointAngleKey(cableIndex, stepIndex), displacement.norm());
      values.update(WrenchKey(params.endEffectorId, cableIndex, stepIndex),
                    wrench);
      values.update(DynamicsSymbol::JointSymbol("t", cableIndex, stepIndex),
                    tension);
    }
  }
  DOUBLES_EQUAL(0, constraints.error(values), 1e-20);
  const auto costs =
      robot.costFactors(2, {{0, params.initialPose}, {1, params.initialPose}});
  DOUBLES_EQUAL(0, combine(costs).error(values), 1e-20);
}

TEST(CableRobotUtils, ManifoldRetractionAndOptimization) {
  CableRobotParams params;
  params.timeStep = 0.01;
  params.waypointWeight = 100;
  params.torqueWeight = 1;
  CableRobot robot(params);
  constexpr int numSteps = 8;
  const auto initialValues = robot.initialValues(numSteps);
  const auto constraints = NonlinearEqualityConstraints::FromCostGraph(
      robot.constraintFactors(numSteps));
  std::map<int, Pose3> waypoints;
  for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex)
    waypoints.emplace(
        stepIndex,
        Pose3(Rot3(), Point3(1.5 + 0.2 * stepIndex / (numSteps - 1),
                             1 + 0.1 * stepIndex / (numSteps - 1), 0)));
  const auto costs = combine(robot.costFactors(numSteps, waypoints));
  const auto options = cableRobotBenchmarkOptions({});
  EXPECT(!options.cmIRetractFinal);
  EXPECT_LONGS_EQUAL(1, options.cmIRetractorMaxIterations);
  for (bool feasible : {true, false}) {
    auto manifoldParams = cableRobotManifoldParams();
    manifoldParams.constraintManifoldParams->retractorCreator->params()
        ->lmParams.maxIterations = feasible ? options.cmFRetractorMaxIterations
                                            : options.cmIRetractorMaxIterations;
    LevenbergMarquardtParams lmParams;
    configureCableRobotLm(ConstrainedOptBenchmark::Method::CM_F, &lmParams);
    NonlinearManifoldOptimizer optimizer(manifoldParams, lmParams);
    const auto problem = optimizer.initializeManifoldOptimizationProblem(
        costs, constraints, initialValues);
    EXPECT_LONGS_EQUAL(numSteps, problem.components.size());
    EXPECT_LONGS_EQUAL(16 * numSteps - 12, problem.values.dim());
    const auto result = optimizer.optimizeManifoldProblem(problem);
    EXPECT(std::isfinite(costs.error(result)));
    EXPECT(costs.error(result) < costs.error(initialValues));
    EXPECT(robot.constraintFactors(numSteps).error(result) < 1e-10);
    EXPECT(assert_equal(params.initialPose,
                        Pose(result, params.endEffectorId, 0), 1e-7));
    EXPECT(Twist(result, params.endEffectorId, 0).norm() < 1e-7);
  }
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
