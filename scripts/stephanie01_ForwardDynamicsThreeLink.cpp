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

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using namespace gtdynamics; 

int main(int argc, char** argv) {
  // Load the three-link robot using the relevant namespace from RobotModels.
  using simple_rr::my_robot;

  // Build the factor graph for the robot.
  my_robot.getLinkByName("link_0")->fix();
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

  auto graph_builder = DynamicsGraph();
  auto graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);

  // Add forward dynamics priors to factor graph.
  JointValues joint_angles, joint_vels, joint_torques;
  joint_angles.insert<double>(my_robot.getJointByName("joint_1")->getKey(), 0);
  joint_vels.insert<double>(my_robot.getJointByName("joint_1")->getKey(), 0);
  joint_torques.insert<double>(my_robot.getJointByName("joint_1")->getKey(), 0);
  joint_angles.insert<double>(my_robot.getJointByName("joint_2")->getKey(), 0);
  joint_vels.insert<double>(my_robot.getJointByName("joint_2")->getKey(), 0);
  joint_torques.insert<double>(my_robot.getJointByName("joint_2")->getKey(), 0);

  auto priorFactors = graph_builder.forwardDynamicsPriors(
      my_robot, 0, joint_angles, joint_vels, joint_torques);
  graph.add(priorFactors);

  // Generate initial values to be passed in to the optimization function.
  auto init_values = ZeroValues(my_robot, 0);

  // Compute forward dynamics.
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  gtsam::Values result = optimizer.optimize();

  // Print the result and its associated error.
  graph_builder.printValues(result);
  std::cout << "Optimization error: " << graph.error(result) << std::endl;

  return 0;
}
