/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  forward_dynamics_simple.cpp
 * @brief Simple forward dynamics optimization for a two link robot with one
 * revolute joint.
 * @author Alejandro Escontrela, Stephanie McCormick, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <iostream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using namespace gtdynamics;

int main(int argc, char** argv) {
  // Load the simple robot and fix the first link's pose.
  using simple_urdf::planar_axis;
  using simple_urdf::robot;
  robot.fixLink("l1");

  gtsam::Vector3 gravity(0, 0, -9.8);

  // Build a factor graph with all the kinodynamics constraints.
  DynamicsGraph dg_builder = DynamicsGraph(gravity, planar_axis);
  gtsam::NonlinearFactorGraph dfg =
      dg_builder.dynamicsFactorGraph(robot, 0);

  // Specify the priors and add them to the factor graph.
  gtsam::Values known_values;
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    InsertJointAngle(&known_values, j, 0, 0.0);
    InsertJointVel(&known_values, j, 0, 0.0);
    InsertTorque(&known_values, j, 0, 0.0);
  }
  gtsam::NonlinearFactorGraph fd_priors =
      dg_builder.forwardDynamicsPriors(robot, 0, known_values);
  dfg.add(fd_priors);

  // Obtain solution initialization.
  gtsam::Values init_values = ZeroValues(robot, 0);

  // Compute the forward dynamics.
  gtsam::LevenbergMarquardtOptimizer optimizer(dfg, init_values);
  gtsam::Values results = optimizer.optimize();

  // Print the resulting values and compute error.
  dg_builder.printValues(results);
  std::cout << "Optimization error: " << dfg.error(results) << std::endl;

  return 0;
}
