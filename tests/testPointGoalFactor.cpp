/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPointGoalFactor.cpp
 * @brief test point goal factor.
 * @author Alejandro Escontrela, Frank dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "gtdynamics/factors/PointGoalFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::LabeledSymbol;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::noiseModel::Constrained;
using gtsam::noiseModel::Unit;

// Test the evaluateError method with various link poses.
TEST(PointGoalFactor, error) {
  using simple_urdf::robot;

  auto cost_model = Unit::Create(3);
  LabeledSymbol pose_key('P', 0, 0);

  // Initialize factor with goal point.
  Point3 goal_point(0, 0, 2), point_com(0, 0, 1);
  PointGoalFactor factor(pose_key, cost_model, point_com, goal_point);

  // Test the goal pose error against the robot's various nominal poses.
  EXPECT(assert_equal(Vector3(0, 0, 0),
                      factor.evaluateError(robot.link("l1")->bMcom())));

  EXPECT(assert_equal(Vector3(0, 0, 2),
                      factor.evaluateError(robot.link("l2")->bMcom())));

  // Make sure linearization is correct
  Values values;
  Pose3 pose(Rot3::RzRyRx(M_PI / 4, 0.4932, 9.81), Point3(-12, 5, 16));
  values.insert(pose_key, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

// Test the optimization of a link pose to ensure goal point is reached.
TEST(PointGoalFactor, optimization) {
  using simple_urdf::robot;

  auto cost_model = Constrained::All(3);

  LabeledSymbol pose_key('P', 0, 0);

  // Initialize factor with goal point.
  Point3 goal_point(2, 15, 6), point_com(0, 0, 1);
  PointGoalFactor factor(pose_key, cost_model, point_com, goal_point);

  // Initial link pose.
  Pose3 pose_init = robot.link("l1")->bMcom();

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  Values init_values;
  init_values.insert(pose_key, pose_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("SILENT");
  params.setVerbosityLM("SILENT");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the initial link twist to ensure no linear velocity
  // at the contact point.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();
  Values results = optimizer.values();
  Pose3 pose_optimized = results.at<Pose3>(pose_key);
  EXPECT(assert_equal(factor.evaluateError(pose_optimized), Vector3::Zero(),
                      1e-4));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
