/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  stephanie01_ForwardDynamicsThreeLink.cpp
 * @brief Test forward dynamics optimization for a three-link robot with two
 * revolute joints.
 * @Author: Stephanie McCormick
 */

#include <DynamicsGraph.h>
#include <RobotModels.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

TEST(DynamicsGraph, optimization) {
  // Load the three-link robot using the relevant namespace from RobotModels.
  using three_link::my_robot;

  // Build the factor graph for the robot.
  gtsam::Vector3 gravity =
      (gtsam::Vector(3) << 0, 0, -9.8).finished();
  gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

  robot::DynamicsGraphBuilder graph_builder = robot::DynamicsGraphBuilder();
  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);

  // Add forward dynamics priors to factor graph.
  gtsam::Vector joint_angles =
      (gtsam::Vector(1) << 0).finished();
  gtsam::Vector joint_vels = (gtsam::Vector(1) << 0).finished();

  gtsam::Vector joint_torques = (gtsam::Vector(1) << 0).finished();
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles, joint_vels,
                                          joint_torques);
  graph.add(prior_factors);

  // Generate initial values to be passed in to the optimization function.
  gtsam::Values init_values = graph_builder.zeroValues(my_robot, 0);

  // Compute forward dynamics.
  gtsam::Values result = graph_builder.optimize(
      graph, init_values,
      robot::DynamicsGraphBuilder::OptimizerType::GaussNewton);

  // Print the result and its associated error.
  graph_builder.print_values(result);
  std::cout << "Optimization error: " << graph.error(result) << std::endl;
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
