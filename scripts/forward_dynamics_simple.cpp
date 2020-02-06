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
 * @Author: Alejandro Escontrela, Stephanie McCormick, and Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <DynamicsGraph.h>
#include <RobotModels.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

TEST(DynamicsGraph, forward_dynamics_r) {
  // Load the simple robot and fix the first link's pose.
  using simple_urdf::my_robot, simple_urdf::planar_axis;
  my_robot.getLinkByName("l1")->fix();

  // Build a factor graph with all the kinodynamics constraints.
  gtdynamics::DynamicsGraph dg_builder = gtdynamics::DynamicsGraph();
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  gtsam::NonlinearFactorGraph dfg =
      dg_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);

  // Specify the priors and add them to the factor graph.
  gtsam::Vector theta = (gtsam::Vector(1) << 0).finished();
  gtsam::Vector theta_dot = (gtsam::Vector(1) << 0).finished();
  gtsam::Vector tau = (gtsam::Vector(1) << 0).finished();
  gtsam::NonlinearFactorGraph fd_priors =
      dg_builder.forwardDynamicsPriors(my_robot, 0, theta, theta_dot, tau);
  dfg.add(fd_priors);

  // Obtain solution initialization.
  gtsam::Values init_values = dg_builder.zeroValues(my_robot, 0);

  // Compute the forward dynamics.
  gtsam::Values results = dg_builder.optimize(
      dfg, init_values, gtdynamics::DynamicsGraph::OptimizerType::LM);

  // Print the resulting values and compute error.
  dg_builder.printValues(results);
  std::cout << "Optimization error: " << dfg.error(results) << std::endl;
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
