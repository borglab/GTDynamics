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

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/RobotModels.h"

#include <CppUnitLite/TestHarness.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

TEST(DynamicsGraph, optimization) {
  // Load the three-link robot using the relevant namespace from RobotModels.
  using three_link::my_robot;

  // Build the factor graph for the robot.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

  auto graph_builder = gtdynamics::DynamicsGraph();
  auto graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);

  // Add forward dynamics priors to factor graph.
  gtdynamics::Robot::JointValues joint_angles, joint_vels, joint_torques;
  joint_angles["joint_1"] = 0;
  joint_vels["joint_1"] = 0;
  joint_torques["joint_1"] = 0;
  joint_angles["joint_2"] = 0;
  joint_vels["joint_2"] = 0;
  joint_torques["joint_2"] = 0;

  auto priorFactors = graph_builder.forwardDynamicsPriors(my_robot, 0, 
            joint_angles, joint_vels, joint_torques);
  graph.add(priorFactors);

  // Generate initial values to be passed in to the optimization function.
  auto init_values = graph_builder.zeroValues(my_robot, 0);

  // Compute forward dynamics.
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  gtsam::Values result = optimizer.optimize();

  // Print the result and its associated error.
  graph_builder.printValues(result);
  std::cout << "Optimization error: " << graph.error(result) << std::endl;
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
