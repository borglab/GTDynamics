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
 * @author: Stephanie McCormick
 */

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using namespace gtdynamics;

int main(int argc, char **argv) {
  // Load the three-link robot using the relevant namespace from RobotModels.
  using simple_rr::robot;

  // Build the factor graph for the robot.
  robot.fixLink("link_0");
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  gtsam::Vector3 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

  DynamicsGraph graph_builder(gravity, planar_axis);
  auto graph = graph_builder.dynamicsFactorGraph(robot, 0);

  // Add forward dynamics priors to factor graph.
  gtsam::Values values;

  auto priorFactors = graph_builder.forwardDynamicsPriors(robot, 0, values);
  graph.add(priorFactors);

  // Generate initial values to be passed in to the optimization function.
  auto init_values = ZeroValues(robot, 0);

  // Compute forward dynamics.
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  gtsam::Values result = optimizer.optimize();

  // Print the result and its associated error.
  graph_builder.printValues(result);
  std::cout << "Optimization error: " << graph.error(result) << std::endl;

  return 0;
}
