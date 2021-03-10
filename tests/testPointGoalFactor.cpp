/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPointGoalFactor.cpp
 * @brief test point goal factor.
 * @author Alejandro Escontrela
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

/**
 * Test the evaluateError method with various link poses.
 **/
TEST(PointGoalFactor, error) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Gaussian::Covariance(gtsam::I_3x3);
  gtsam::LabeledSymbol pose_key = gtsam::LabeledSymbol('P', 0, 0);

  // Initialize factor with goal point.
  gtsam::Point3 goal_point = gtsam::Point3(0, 0, 2);
  gtsam::Pose3 comTp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1));
  PointGoalFactor factor(pose_key, cost_model, comTp, goal_point);

  // Test the goal pose error against the robot's various nominal poses.
  EXPECT(assert_equal((gtsam::Vector(3) << 0, 0, 0).finished(),
                      factor.evaluateError(my_robot.link("l1")->wTcom())));

  EXPECT(assert_equal((gtsam::Vector(3) << 0, 0, 2).finished(),
                      factor.evaluateError(my_robot.link("l2")->wTcom())));

  // Make sure linearization is correct
  gtsam::Values values;
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 4, 0.4932, 9.81),
                                   gtsam::Point3(-12, 5, 16));
  values.insert(pose_key, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/**
 * Test the optimization of a link pose to ensure goal point is reached.
 **/
TEST(PointGoalFactor, optimization) {
  using simple_urdf::my_robot;

  gtsam::noiseModel::Gaussian::shared_ptr cost_model =
      gtsam::noiseModel::Constrained::All(3);

  gtsam::LabeledSymbol pose_key = gtsam::LabeledSymbol('P', 0, 0);

  // Initialize factor with goal point.
  gtsam::Point3 goal_point = gtsam::Point3(2, 15, 6);
  gtsam::Pose3 comTp = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1));
  PointGoalFactor factor(pose_key, cost_model, comTp, goal_point);

  // Initial link pose.
  gtsam::Pose3 pose_init = my_robot.link("l1")->wTcom();
  // std::cout << "Error Init: " << factor.evaluateError(pose_init).transpose()
  // << std::endl;

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);
  gtsam::Values init_values;
  init_values.insert(pose_key, pose_init);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("SILENT");
  params.setVerbosityLM("SILENT");
  params.setAbsoluteErrorTol(1e-12);

  // Optimize the initial link twist to ensure no linear velocity
  // at the contact point.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  optimizer.optimize();
  gtsam::Values results = optimizer.values();
  gtsam::Pose3 pose_optimized = results.at<gtsam::Pose3>(pose_key);
  // std::cout << "Error Final: "
  //           << factor.evaluateError(pose_optimized).transpose() << std::endl;
  EXPECT(assert_equal(factor.evaluateError(pose_optimized),
                      gtsam::Vector3::Zero(), 1e-4));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
