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
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  gtsam::Vector3 planarAxis = (gtsam::Vector(3) << 1, 0, 0).finished();

  auto graphBuilder = robot::DynamicsGraphBuilder();
  auto graph =
      graphBuilder.dynamicsFactorGraph(my_robot, 0, gravity, planarAxis);

  // Add forward dynamics priors to factor graph.
  gtsam::Vector jointAngles =
      (gtsam::Vector(1) << 0).finished();
  gtsam::Vector jointVels = (gtsam::Vector(1) << 0).finished();

  gtsam::Vector jointTorques = (gtsam::Vector(1) << 0).finished();
  auto priorFactors = graphBuilder.forwardDynamicsPriors(my_robot, 0, 
            jointAngles, jointVels, jointTorques);
  graph.add(priorFactors);

  // Generate initial values to be passed in to the optimization function.
  auto initValues = graphBuilder.zeroValues(my_robot, 0);

  // Compute forward dynamics.
  gtsam::Values result = graphBuilder.optimize(
      graph, initValues,
      robot::DynamicsGraphBuilder::OptimizerType::GaussNewton);

  // Print the result and its associated error.
  graphBuilder.print_values(result);
  std::cout << "Optimization error: " << graph.error(result) << std::endl;
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
