/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPoseGoalFactor.cpp
 * @brief test pose goal factor.
 * @Author: Frank Dellaert, Mandy Xie, and Alejandro Escontrela
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

#include "gtdynamics/factors/PoseGoalFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"

using gtsam::assert_equal;

/**
 * Test the evaluateError method with various link poses.
 **/
TEST(PoseGoalFactor, error) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_6x6);
  gtsam::LabeledSymbol pose_key = gtsam::LabeledSymbol('P', 0, 0);

  // Goal pose at origin with orientation of identity.
  gtsam::Pose3 goal_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());
  gtdynamics::PoseGoalFactor factor(pose_key, cost_model, goal_pose);

  // Test the goal pose error against the robot's various nominal poses.
  EXPECT(assert_equal(
      (gtsam::Vector(6) << 0, 0, 0, 0, 0, -1).finished(),
      factor.evaluateError(my_robot.getLinkByName("l1")->wTcom())));

  EXPECT(assert_equal(
      (gtsam::Vector(6) << 0, 0, 0, 0, 0, -3).finished(),
      factor.evaluateError(my_robot.getLinkByName("l2")->wTcom())));

  // Make sure linearization is correct
  gtsam::Values values;
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 4, 0.4932, 9.81),
                                   gtsam::Point3(-12, 5, 16));
  values.insert(pose_key, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a link pose to ensure goal pose is reached.
 **/
TEST(PoseGoalFactor, optimization) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Constrained::All(6);

  gtsam::LabeledSymbol pose_key = gtsam::LabeledSymbol('P', 0, 0);

  // Initialize factor with goal pose.
  gtsam::Pose3 goal_pose =
      gtsam::Pose3(gtsam::Rot3::Rx(M_PI / 2), gtsam::Point3(-12, 15, 6));
  gtdynamics::PoseGoalFactor factor(pose_key, cost_model, goal_pose);

  // Initial link twist.
  gtsam::Pose3 pose_init = my_robot.getLinkByName("l1")->wTcom();

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values init_values;
  init_values.insert(pose_key, pose_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the initial link twist to ensure no linear velocity
  // at the contact point.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();
  gtsam::Values results = optimizer.values();
  gtsam::Pose3 pose_optimized = results.at(pose_key).cast<gtsam::Pose3>();
  EXPECT(assert_equal(factor.evaluateError(pose_optimized),
                      gtsam::Vector6::Zero()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
