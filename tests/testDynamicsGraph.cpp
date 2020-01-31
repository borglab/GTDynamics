/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDynamicsGraph.cpp
 * @brief Test forward and inverse dynamics factor graph.
 * @Author: Yetong Zhang, Alejandro Escontrela
 */

#include <CppUnitLite/TestHarness.h>
#include <DynamicsGraph.h>
#include <RobotModels.h>
#include <UniversalRobot.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <utils.h>

#include <iostream>

using robot::DynamicsGraphBuilder;

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

using gtsam::assert_equal;

// Test forward dynamics with gravity of a two-link robot, with base link fixed
TEST(dynamicsFactorGraph_FD, simple_urdf_eq_mass) {
  using simple_urdf_eq_mass::my_robot, simple_urdf_eq_mass::gravity,
      simple_urdf_eq_mass::planar_axis, simple_urdf_eq_mass::joint_angles,
      simple_urdf_eq_mass::joint_vels;
  gtsam::Vector torques = gtsam::Vector::Ones(my_robot.numJoints());

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();
  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles,
                                                joint_vels, torques));
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links()) {
    int i = link->getID();
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        robot::PoseKey(i, 0), link->Twcom(), gtsam::noiseModel::Constrained::All(6)));
    graph.add(gtsam::PriorFactor<gtsam::Vector6>(
        robot::TwistKey(i, 0), gtsam::Vector6::Zero(),
        gtsam::noiseModel::Constrained::All(6)));
  }

  gtsam::Values result = graph_builder.optimize(
      graph, DynamicsGraphBuilder::zeroValues(my_robot, 0),
      DynamicsGraphBuilder::OptimizerType::GaussNewton);

  gtsam::Vector actual_qAccel =
      DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  gtsam::Vector expected_qAccel = (gtsam::Vector(1) << 4).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 1e-3));
}

// Test forward dynamics with gravity of a four-bar linkage
TEST(dynamicsFactorGraph_FD, four_bar_linkage) {
  // Load the robot from urdf file
  using four_bar_linkage::my_robot, four_bar_linkage::joint_angles,
      four_bar_linkage::joint_vels, four_bar_linkage::gravity,
      four_bar_linkage::planar_axis;
  gtsam::Vector torques = (gtsam::Vector(4) << 1, 0, 1, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraphBuilder();

  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles, joint_vels,
                                          torques);
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links()) {
    int i = link->getID();
    prior_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        robot::PoseKey(i, 0), link->Twcom(), gtsam::noiseModel::Constrained::All(6)));
    prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
        robot::TwistKey(i, 0), gtsam::Vector6::Zero(),
        gtsam::noiseModel::Constrained::All(6)));
  }

  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  gtsam::Values init_values = DynamicsGraphBuilder::zeroValues(my_robot, 0);

  // test the four bar linkage FD in the free-floating scenario
  gtsam::Values result = graph_builder.optimize(
      graph, init_values, DynamicsGraphBuilder::OptimizerType::LM);
  gtsam::Vector actual_qAccel =
      DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  gtsam::Vector expected_qAccel = (gtsam::Vector(4) << 1, -1, 1, -1).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 1e-4));

  // A planar four bar linkage in 3D space should throw an ILS error with the
  // constraints specified.
  graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity);
  graph.add(prior_factors);
  gtsam::GaussNewtonOptimizer optimizer1(graph, init_values);
  THROWS_EXCEPTION(optimizer1.optimize());

  // test the condition when we fix link "l1"
  my_robot.getLinkByName("l1")->fix();
  graph = graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  result = graph_builder.optimize(
      graph, init_values, DynamicsGraphBuilder::OptimizerType::GaussNewton);
  actual_qAccel = DynamicsGraphBuilder::jointAccels(my_robot, result, 0);
  expected_qAccel = (gtsam::Vector(4) << 0.25, -0.25, 0.25, -0.25).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
