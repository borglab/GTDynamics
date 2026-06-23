/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testEndEffFactor.cpp
 * @brief test end-effector pose goal factor.
 * @author Karthik Shaji
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/config.h>
#include <gtdynamics/kinematics/endEffGoalFactor.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <string>

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::LabeledSymbol;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector6;
using gtsam::noiseModel::Constrained;
using gtsam::noiseModel::Unit;

// Load the bar_lab gantry robot once for all tests.
static const Robot kRobot =
    CreateRobotFromFile(kUrdfPath + std::string("bar_lab.urdf"));

// Test the unwhitened error of PoseGoalFactor at and away from the goal pose.
// Naming follows the aTb convention: wTcom is the link CoM pose in the world
// frame (the pose variable), wTcom_goal is the desired CoM pose in world.
TEST(PoseGoalFactor, error) {
  auto cost_model = Unit::Create(6);
  LabeledSymbol pose_key('P', 0, 0);

  // Use a nominal end-effector link pose as the goal.
  Pose3 wTcom_goal = kRobot.link("robot1_link_6")->bMcom();
  PoseGoalFactor factor(pose_key, cost_model, wTcom_goal);

  // Zero error when the link is exactly at the goal pose.
  Values values_at_goal;
  values_at_goal.insert(pose_key, wTcom_goal);
  EXPECT(assert_equal(Vector6::Zero(), factor.unwhitenedError(values_at_goal)));

  // Away from the goal, the error is the tangent-space difference (logmap),
  // computed here independently via Pose3::logmap.
  Pose3 wTcom(Rot3::RzRyRx(0.3, -0.2, 0.5), Point3(-1.0, 2.0, 0.5));
  Values values;
  values.insert(pose_key, wTcom);
  EXPECT(assert_equal(wTcom.logmap(wTcom_goal),
                      factor.unwhitenedError(values), 1e-9));

  // The error should be non-trivial for this displaced pose.
  EXPECT(1e-3 < factor.unwhitenedError(values).norm());

  // Make sure the linearization (Jacobian) is correct.
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test that the goal pose is recovered by the standalone PoseGoalConstraint
// expression (the building block used by both the factor and the constraints).
TEST(PoseGoalFactor, expression) {
  LabeledSymbol pose_key('P', 0, 0);
  Pose3 wTcom_goal(Rot3::RzRyRx(0.1, 0.2, -0.3), Point3(1.5, -2.0, 3.0));

  auto expr = PoseGoalConstraint(pose_key, wTcom_goal);

  // Evaluated at the goal, the residual vanishes. Compare as dynamic vectors to
  // avoid the ambiguous Matrix/Vector assert_equal overloads on fixed Vector6.
  Values at_goal;
  at_goal.insert(pose_key, wTcom_goal);
  EXPECT(assert_equal(gtsam::Vector(Vector6::Zero()),
                      gtsam::Vector(expr.value(at_goal))));

  // Away from the goal, it matches Pose3::logmap.
  Pose3 wTcom = kRobot.link("robot1_link_1")->bMcom();
  Values values;
  values.insert(pose_key, wTcom);
  EXPECT(assert_equal(gtsam::Vector(wTcom.logmap(wTcom_goal)),
                      gtsam::Vector(expr.value(values)), 1e-9));
}

// Test that optimizing a single link pose drives it to the goal pose.
TEST(PoseGoalFactor, optimization) {
  auto cost_model = Constrained::All(6);
  LabeledSymbol pose_key('P', 0, 0);

  Pose3 wTcom_goal(Rot3::RzRyRx(0.1, 0.2, -0.3), Point3(1.5, -2.0, 3.0));
  PoseGoalFactor factor(pose_key, cost_model, wTcom_goal);

  // Start from a nominal robot link pose, far from the goal.
  Pose3 wTcom_init = kRobot.link("robot1_link_1")->bMcom();

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  Values init_values;
  init_values.insert(pose_key, wTcom_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("SILENT");
  params.setVerbosityLM("SILENT");
  params.setAbsoluteErrorTol(1e-12);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();
  Values results = optimizer.values();

  Pose3 wTcom_opt = results.at<Pose3>(pose_key);
  EXPECT(assert_equal(wTcom_goal, wTcom_opt, 1e-4));
  EXPECT(assert_equal(Vector6::Zero(), factor.unwhitenedError(results), 1e-4));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
