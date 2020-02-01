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
using gtsam::Values, gtsam::NonlinearFactorGraph, gtsam::PriorFactor,
    gtsam::Vector;
using robot::JointAccelKey;
using robot::JointAngleKey;
using robot::JointVelKey;
using robot::PhaseKey;
using robot::PoseKey;
using robot::TorqueKey;
using robot::TwistAccelKey;
using robot::TwistKey;
using robot::WrenchKey;
using std::vector;

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
        robot::PoseKey(i, 0), link->Twcom(),
        gtsam::noiseModel::Constrained::All(6)));
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
        robot::PoseKey(i, 0), link->Twcom(),
        gtsam::noiseModel::Constrained::All(6)));
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

TEST(collocationFactors, simple_urdf) {
  using simple_urdf::my_robot;
  double dt = 1;
  int t = 0;
  int j = my_robot.joints()[0]->getID();

  NonlinearFactorGraph prior_factors;
  prior_factors.add(PriorFactor<double>(
      JointAngleKey(j, t), 1, gtsam::noiseModel::Constrained::All(1)));
  prior_factors.add(PriorFactor<double>(
      JointVelKey(j, t), 1, gtsam::noiseModel::Constrained::All(1)));
  prior_factors.add(PriorFactor<double>(
      JointAccelKey(j, t), 1, gtsam::noiseModel::Constrained::All(1)));
  prior_factors.add(PriorFactor<double>(
      JointAccelKey(j, t + 1), 2, gtsam::noiseModel::Constrained::All(1)));

  auto graph_builder = DynamicsGraphBuilder();

  Values init_values;
  init_values.insert(JointAngleKey(j, t), 0.0);
  init_values.insert(JointVelKey(j, t), 0.0);
  init_values.insert(JointAccelKey(j, t), 0.0);
  init_values.insert(JointAngleKey(j, t + 1), 0.0);
  init_values.insert(JointVelKey(j, t + 1), 0.0);
  init_values.insert(JointAccelKey(j, t + 1), 0.0);

  // test trapezoidal
  NonlinearFactorGraph trapezoidal_graph;
  trapezoidal_graph.add(graph_builder.collocationFactors(
      my_robot, t, dt, DynamicsGraphBuilder::CollocationScheme::Trapezoidal));
  trapezoidal_graph.add(prior_factors);
  Values trapezoidal_result =
      graph_builder.optimize(trapezoidal_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(
      assert_equal(2.75, trapezoidal_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.5, trapezoidal_result.atDouble(JointVelKey(j, t + 1))));

  // test Euler
  NonlinearFactorGraph euler_graph;
  euler_graph.add(graph_builder.collocationFactors(
      my_robot, t, dt, DynamicsGraphBuilder::CollocationScheme::Euler));
  euler_graph.add(prior_factors);
  Values euler_result =
      graph_builder.optimize(euler_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.0, euler_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.0, euler_result.atDouble(JointVelKey(j, t + 1))));

  // test the scenario with dt as a variable
  int phase = 0;
  init_values.insert(PhaseKey(phase), 0.0);
  prior_factors.add(PriorFactor<double>(
      PhaseKey(phase), dt, gtsam::noiseModel::Constrained::All(1)));

  // multi-phase euler
  NonlinearFactorGraph mp_euler_graph;
  mp_euler_graph.add(graph_builder.multiPhaseCollocationFactors(
      my_robot, t, phase, DynamicsGraphBuilder::CollocationScheme::Euler));
  mp_euler_graph.add(prior_factors);
  Values mp_euler_result =
      graph_builder.optimize(mp_euler_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointVelKey(j, t + 1))));

  // multi-phase trapezoidal
  NonlinearFactorGraph mp_trapezoidal_graph;
  mp_trapezoidal_graph.add(graph_builder.collocationFactors(
      my_robot, t, dt, DynamicsGraphBuilder::CollocationScheme::Trapezoidal));
  mp_trapezoidal_graph.add(prior_factors);
  Values mp_trapezoidal_result =
      graph_builder.optimize(mp_trapezoidal_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(2.75,
                      mp_trapezoidal_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(
      assert_equal(2.5, mp_trapezoidal_result.atDouble(JointVelKey(j, t + 1))));
}

// test forward dynamics of a trajectory
TEST(dynamicsTrajectoryFG, simple_urdf_eq_mass) {
  using simple_urdf_eq_mass::my_robot, simple_urdf_eq_mass::gravity,
    simple_urdf_eq_mass::planar_axis, simple_urdf_eq_mass::joint_vels,
    simple_urdf_eq_mass::joint_angles;
  my_robot.getLinkByName("l1")->fix();
  int j = my_robot.joints()[0]->getID();
  auto graph_builder = DynamicsGraphBuilder();

  int num_steps = 2;
  double dt = 1;
  vector<Vector> torques_seq;
  for (int i = 0; i <= num_steps; i++) {
    torques_seq.emplace_back((Vector(1) << i * 1.0 + 1.0).finished());
  }

  Values init_values =
      DynamicsGraphBuilder::zeroValuesTrajectory(my_robot, num_steps);

  // test Euler
  NonlinearFactorGraph euler_graph = graph_builder.trajectoryFG(
      my_robot, num_steps, dt, DynamicsGraphBuilder::CollocationScheme::Euler,
      gravity, planar_axis);
  euler_graph.add(graph_builder.trajectoryFDPriors(
      my_robot, num_steps, joint_angles, joint_vels, torques_seq));
  Values euler_result =
      graph_builder.optimize(euler_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(0.0, euler_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.0, euler_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, euler_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(1.0, euler_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(3.0, euler_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, euler_result.atDouble(JointAccelKey(j, 2))));

  // test trapezoidal
  NonlinearFactorGraph trapezoidal_graph = graph_builder.trajectoryFG(
      my_robot, num_steps, dt,
      DynamicsGraphBuilder::CollocationScheme::Trapezoidal, gravity,
      planar_axis);
  trapezoidal_graph.add(graph_builder.trajectoryFDPriors(
      my_robot, num_steps, joint_angles, joint_vels, torques_seq));
  Values trapezoidal_result =
      graph_builder.optimize(trapezoidal_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  EXPECT(assert_equal(0.75, trapezoidal_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.5, trapezoidal_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, trapezoidal_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(3.5, trapezoidal_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(4.0, trapezoidal_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, trapezoidal_result.atDouble(JointAccelKey(j, 2))));

  // test the scenario with dt as a variable
  vector<int> phase_steps{1, 1};
  vector<UniversalRobot> robots(2, my_robot);
  NonlinearFactorGraph transition_graph =
      graph_builder.dynamicsFactorGraph(my_robot, 1, gravity, planar_axis);
  vector<NonlinearFactorGraph> transition_graphs{transition_graph};
  double dt0 = 1;
  double dt1 = 2;
  NonlinearFactorGraph mp_prior_graph = graph_builder.trajectoryFDPriors(
      my_robot, num_steps, joint_angles, joint_vels, torques_seq);
  mp_prior_graph.add(PriorFactor<double>(
      PhaseKey(0), dt0, gtsam::noiseModel::Constrained::All(1)));
  mp_prior_graph.add(PriorFactor<double>(
      PhaseKey(1), dt1, gtsam::noiseModel::Constrained::All(1)));
  init_values =
      DynamicsGraphBuilder::zeroValuesTrajectory(my_robot, num_steps, 2);

  // multi-phase Euler
  NonlinearFactorGraph mp_euler_graph = graph_builder.multiPhaseTrajectoryFG(
      robots, phase_steps, transition_graphs,
      DynamicsGraphBuilder::CollocationScheme::Euler, gravity, planar_axis);
  mp_euler_graph.add(mp_prior_graph);
  Values mp_euler_result =
      graph_builder.optimize(mp_euler_graph, init_values,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);

  // t        0   1   2
  // dt         1   2
  // torque   1   2   3
  // q        0   0   2
  // v        0   1   5
  // a        1   2   3
  EXPECT(assert_equal(0.0, mp_euler_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.0, mp_euler_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(5.0, mp_euler_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, mp_euler_result.atDouble(JointAccelKey(j, 2))));

  // multi-phase Trapezoidal
  NonlinearFactorGraph mp_trapezoidal_graph =
      graph_builder.multiPhaseTrajectoryFG(
          robots, phase_steps, transition_graphs,
          DynamicsGraphBuilder::CollocationScheme::Trapezoidal, gravity,
          planar_axis);
  mp_trapezoidal_graph.add(mp_prior_graph);
  Values mp_trapezoidal_result =
      graph_builder.optimize(mp_trapezoidal_graph, mp_euler_result,
                             DynamicsGraphBuilder::OptimizerType::GaussNewton);
  // t        0     1     2
  // dt          1     2
  // torque   1     2     3
  // q        0     0.75  8.75
  // v        0     1.5   6.5
  // a        1     2     3
  EXPECT(
      assert_equal(0.75, mp_trapezoidal_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.5, mp_trapezoidal_result.atDouble(JointVelKey(j, 1))));
  EXPECT(
      assert_equal(2.0, mp_trapezoidal_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(
      assert_equal(8.75, mp_trapezoidal_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(6.5, mp_trapezoidal_result.atDouble(JointVelKey(j, 2))));
  EXPECT(
      assert_equal(3.0, mp_trapezoidal_result.atDouble(JointAccelKey(j, 2))));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
