/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testDynamicsGraph.cpp
 * @brief Test forward and inverse dynamics factor graph.
 * @authors Yetong Zhang, Alejandro Escontrela
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/factors/MinTorqueFactor.h"
#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/universal_robot/sdf.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using namespace gtdynamics; 

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

using gtsam::assert_equal;
using gtsam::Values, gtsam::NonlinearFactorGraph, gtsam::PriorFactor,
    gtsam::Vector;
using std::vector;

// Test linear dynamics graph of a two-link robot, base fixed, with gravity
TEST(linearDynamicsFactorGraph, simple_urdf_eq_mass) {
  using simple_urdf_eq_mass::my_robot, simple_urdf_eq_mass::gravity,
      simple_urdf_eq_mass::planar_axis;

  auto graph_builder = DynamicsGraph();
  int t = 0;
  Robot::JointValues joint_angles, joint_vels, joint_torques, joint_accels;
  joint_angles["j1"] = 0;
  joint_vels["j1"] = 0;
  joint_torques["j1"] = 1;
  joint_accels["j1"] = 4;
  std::string prior_link_name = "l1";
  auto l1 = my_robot.getLinkByName(prior_link_name);
  gtsam::Vector6 V_l1 = gtsam::Vector6::Zero();
  auto fk_results = my_robot.forwardKinematics(
      joint_angles, joint_vels, prior_link_name, l1->wTcom(), V_l1);

  // test forward dynamics
  Values result_fd = graph_builder.linearSolveFD(my_robot, t, joint_angles,
                                              joint_vels, joint_torques,
                                              fk_results, gravity, planar_axis);

  int j = my_robot.joints()[0]->getID();
  EXPECT(assert_equal(4.0, result_fd.atDouble(JointAccelKey(j, t)), 1e-3));

  // test inverse dynamics
  Values result_id =  graph_builder.linearSolveID(my_robot, t, joint_angles,
                                              joint_vels, joint_accels,
                                              fk_results, gravity, planar_axis);
  EXPECT(assert_equal(1.0, result_id.atDouble(TorqueKey(j, t)), 1e-3));
}

// Test forward dynamics with gravity of a two-link robot, with base link fixed
TEST(dynamicsFactorGraph_FD, simple_urdf_eq_mass) {
  using simple_urdf_eq_mass::my_robot, simple_urdf_eq_mass::gravity,
      simple_urdf_eq_mass::planar_axis, simple_urdf_eq_mass::joint_angles,
      simple_urdf_eq_mass::joint_vels;
  gtsam::Vector torques = gtsam::Vector::Ones(my_robot.numJoints());

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraph();
  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles,
                                                joint_vels, torques));
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links()) {
    int i = link->getID();
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(i, 0), link->wTcom(),
        graph_builder.opt().bp_cost_model));
    graph.add(gtsam::PriorFactor<gtsam::Vector6>(
        TwistKey(i, 0), gtsam::Vector6::Zero(),
        graph_builder.opt().bv_cost_model));
  }

  gtsam::GaussNewtonOptimizer optimizer(graph,
                                        ZeroValues(my_robot, 0));
  Values result = optimizer.optimize();

  gtsam::Vector actual_qAccel = DynamicsGraph::jointAccels(my_robot, result, 0);
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
  auto graph_builder = DynamicsGraph();

  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles, joint_vels,
                                          torques);
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : my_robot.links()) {
    int i = link->getID();
    prior_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(i, 0), link->wTcom(),
        graph_builder.opt().bp_cost_model));
    prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
        TwistKey(i, 0), gtsam::Vector6::Zero(),
        graph_builder.opt().bv_cost_model));
  }

  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(prior_factors);

  gtsam::Values init_values = ZeroValues(my_robot, 0);

  // test the four bar linkage FD in the free-floating scenario
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values result = optimizer.optimize();
  gtsam::Vector actual_qAccel = DynamicsGraph::jointAccels(my_robot, result, 0);
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

  gtsam::GaussNewtonOptimizer optimizer2(graph, init_values);
  result = optimizer2.optimize();

  actual_qAccel = DynamicsGraph::jointAccels(my_robot, result, 0);
  expected_qAccel = (gtsam::Vector(4) << 0.25, -0.25, 0.25, -0.25).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

// test jumping robot
TEST(dynamicsFactorGraph_FD, jumping_robot) {
  using jumping_robot::gravity, jumping_robot::planar_axis,
      jumping_robot::joint_angles, jumping_robot::joint_vels,
      jumping_robot::my_robot;
  double torque3 = 0;
  double torque2 = 0.5;
  Vector torques =
      (Vector(6) << 0, torque2, torque3, torque3, torque2, 0).finished();

  // build the dynamics factor graph
  auto graph_builder = DynamicsGraph();
  NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(my_robot, 0, gravity, planar_axis);
  graph.add(graph_builder.forwardDynamicsPriors(my_robot, 0, joint_angles,
                                                joint_vels, torques));

  // test jumping robot FD
  gtsam::GaussNewtonOptimizer optimizer(graph,
                                        ZeroValues(my_robot, 0));
  Values result = optimizer.optimize();

  // check acceleration
  auto expected_qAccel = Vector(6);
  double m1 = 0.31;
  double m2 = 0.28;
  double m3 = 0.54;
  double link_radius = 0.02;
  double l = 0.55;
  double theta = 0.0 / 180.0 * M_PI;
  double acc =
      (torque3 - torque2 * 2 -
       (0.5 * m1 + 1.5 * m2 + 1.0 * m3) * 9.8 * l * std::sin(theta)) /
      (std::pow(l, 2) *
           (1.0 / 4 * m1 + (1.0 / 4 + 2 * std::pow(std::sin(theta), 2)) * m2 +
            2 * std::pow(std::sin(theta), 2) * m3) +
       (std::pow(l, 2) + 3 * std::pow(link_radius, 2)) *
           (1.0 / 12 * m1 + 1.0 / 12 * m2));
  expected_qAccel << acc, -2 * acc, acc, acc, -2 * acc, acc;
  Vector actual_qAccel = DynamicsGraph::jointAccels(my_robot, result, 0);
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(collocationFactors, simple_urdf) {
  auto graph_builder = DynamicsGraph();
  using simple_urdf::my_robot;
  double dt = 1;
  int t = 0;
  int j = my_robot.joints()[0]->getID();

  NonlinearFactorGraph prior_factors;
  prior_factors.add(PriorFactor<double>(
      JointAngleKey(j, t), 1, graph_builder.opt().prior_q_cost_model));
  prior_factors.add(PriorFactor<double>(
      JointVelKey(j, t), 1, graph_builder.opt().prior_qv_cost_model));
  prior_factors.add(PriorFactor<double>(
      JointAccelKey(j, t), 1, graph_builder.opt().prior_qa_cost_model));
  prior_factors.add(PriorFactor<double>(
      JointAccelKey(j, t + 1), 2, graph_builder.opt().prior_qa_cost_model));

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
      my_robot, t, dt, DynamicsGraph::CollocationScheme::Trapezoidal));
  trapezoidal_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_t(trapezoidal_graph, init_values);
  Values trapezoidal_result = optimizer_t.optimize();

  EXPECT(
      assert_equal(2.75, trapezoidal_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.5, trapezoidal_result.atDouble(JointVelKey(j, t + 1))));

  // test Euler
  NonlinearFactorGraph euler_graph;
  euler_graph.add(graph_builder.collocationFactors(
      my_robot, t, dt, DynamicsGraph::CollocationScheme::Euler));
  euler_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_e(euler_graph, init_values);
  Values euler_result = optimizer_e.optimize();

  EXPECT(assert_equal(2.0, euler_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.0, euler_result.atDouble(JointVelKey(j, t + 1))));

  // test the scenario with dt as a variable
  int phase = 0;
  init_values.insert(PhaseKey(phase), 0.0);
  prior_factors.add(PriorFactor<double>(PhaseKey(phase), dt,
                                        graph_builder.opt().time_cost_model));

  // multi-phase euler
  NonlinearFactorGraph mp_euler_graph;
  mp_euler_graph.add(graph_builder.multiPhaseCollocationFactors(
      my_robot, t, phase, DynamicsGraph::CollocationScheme::Euler));
  mp_euler_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_mpe(mp_euler_graph, init_values);
  Values mp_euler_result = optimizer_mpe.optimize();

  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointAngleKey(j, t + 1))));
  EXPECT(assert_equal(2.0, mp_euler_result.atDouble(JointVelKey(j, t + 1))));

  // multi-phase trapezoidal
  NonlinearFactorGraph mp_trapezoidal_graph;
  mp_trapezoidal_graph.add(graph_builder.collocationFactors(
      my_robot, t, dt, DynamicsGraph::CollocationScheme::Trapezoidal));
  mp_trapezoidal_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_mpt(mp_trapezoidal_graph, init_values);
  Values mp_trapezoidal_result = optimizer_mpt.optimize();

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
  auto graph_builder = DynamicsGraph();

  int num_steps = 2;
  double dt = 1;
  vector<Vector> torques_seq;
  for (int i = 0; i <= num_steps; i++) {
    torques_seq.emplace_back((Vector(1) << i * 1.0 + 1.0).finished());
  }

  Values init_values = ZeroValuesTrajectory(my_robot, num_steps);

  // test Euler
  NonlinearFactorGraph euler_graph = graph_builder.trajectoryFG(
      my_robot, num_steps, dt, DynamicsGraph::CollocationScheme::Euler, gravity,
      planar_axis);
  euler_graph.add(graph_builder.trajectoryFDPriors(
      my_robot, num_steps, joint_angles, joint_vels, torques_seq));

  gtsam::GaussNewtonOptimizer optimizer_e(euler_graph, init_values);
  Values euler_result = optimizer_e.optimize();

  EXPECT(assert_equal(0.0, euler_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.0, euler_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, euler_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(1.0, euler_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(3.0, euler_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, euler_result.atDouble(JointAccelKey(j, 2))));

  // test trapezoidal
  NonlinearFactorGraph trapezoidal_graph = graph_builder.trajectoryFG(
      my_robot, num_steps, dt, DynamicsGraph::CollocationScheme::Trapezoidal,
      gravity, planar_axis);
  trapezoidal_graph.add(graph_builder.trajectoryFDPriors(
      my_robot, num_steps, joint_angles, joint_vels, torques_seq));

  gtsam::GaussNewtonOptimizer optimizer_t(trapezoidal_graph, init_values);
  Values trapezoidal_result = optimizer_t.optimize();

  EXPECT(assert_equal(0.75, trapezoidal_result.atDouble(JointAngleKey(j, 1))));
  EXPECT(assert_equal(1.5, trapezoidal_result.atDouble(JointVelKey(j, 1))));
  EXPECT(assert_equal(2.0, trapezoidal_result.atDouble(JointAccelKey(j, 1))));
  EXPECT(assert_equal(3.5, trapezoidal_result.atDouble(JointAngleKey(j, 2))));
  EXPECT(assert_equal(4.0, trapezoidal_result.atDouble(JointVelKey(j, 2))));
  EXPECT(assert_equal(3.0, trapezoidal_result.atDouble(JointAccelKey(j, 2))));

  // test the scenario with dt as a variable
  vector<int> phase_steps{1, 1};
  vector<Robot> robots(2, my_robot);
  NonlinearFactorGraph transition_graph =
      graph_builder.dynamicsFactorGraph(my_robot, 1, gravity, planar_axis);
  vector<NonlinearFactorGraph> transition_graphs{transition_graph};
  double dt0 = 1;
  double dt1 = 2;
  NonlinearFactorGraph mp_prior_graph = graph_builder.trajectoryFDPriors(
      my_robot, num_steps, joint_angles, joint_vels, torques_seq);
  mp_prior_graph.add(PriorFactor<double>(PhaseKey(0), dt0,
                                         graph_builder.opt().time_cost_model));
  mp_prior_graph.add(PriorFactor<double>(PhaseKey(1), dt1,
                                         graph_builder.opt().time_cost_model));
  init_values = ZeroValuesTrajectory(my_robot, num_steps, 2);

  // multi-phase Euler
  NonlinearFactorGraph mp_euler_graph = graph_builder.multiPhaseTrajectoryFG(
      robots, phase_steps, transition_graphs,
      DynamicsGraph::CollocationScheme::Euler, gravity, planar_axis);
  mp_euler_graph.add(mp_prior_graph);
  gtsam::GaussNewtonOptimizer optimizer_mpe(mp_euler_graph, init_values);
  Values mp_euler_result = optimizer_mpe.optimize();

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
          DynamicsGraph::CollocationScheme::Trapezoidal, gravity, planar_axis);
  mp_trapezoidal_graph.add(mp_prior_graph);
  gtsam::GaussNewtonOptimizer optimizer_mpt(mp_trapezoidal_graph,
                                            mp_euler_result);
  Values mp_trapezoidal_result = optimizer_mpt.optimize();

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

// Test contacts in dynamics graph.
TEST(dynamicsFactorGraph_Contacts, dynamics_graph_simple_rr) {
  // Load the robot from urdf file
  using simple_rr::my_robot;

  // Add some contact points.
  ContactPoints contact_points;
  contact_points.emplace("link_0",
      ContactPoint{gtsam::Point3(0, 0, -0.1), 0});

  // Build the dynamics FG.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  auto graph_builder = DynamicsGraph();
  gtsam::NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(
      my_robot, 0, gravity, boost::none, contact_points, 1.0);

  // Compute inverse dynamics prior factors.
  gtsam::Vector joint_accels = gtsam::Vector::Zero(my_robot.numJoints());
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(my_robot, 0, simple_rr::joint_angles,
                                          simple_rr::joint_vels, joint_accels);

  // Specify pose and twist priors for one leg.
  prior_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      PoseKey(my_robot.getLinkByName("link_0")->getID(), 0),
      my_robot.getLinkByName("link_0")->wTcom(),
      gtsam::noiseModel::Constrained::All(6)));
  prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      TwistKey(my_robot.getLinkByName("link_0")->getID(), 0),
      gtsam::Vector6::Zero(), gtsam::noiseModel::Constrained::All(6)));
  graph.add(prior_factors);

  // Add min torque factor.
  for (auto joint : my_robot.joints())
    graph.add(
        MinTorqueFactor(TorqueKey(joint->getID(), 0),
                                    gtsam::noiseModel::Isotropic::Sigma(1, 1)));

  // Set initial values.
  gtsam::Values init_values =
      ZeroValues(my_robot, 0, 0.0, contact_points);

//   graph_builder.printGraph(graph);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values results = optimizer.optimize();
//   std::cout << "Error: " << graph.error(results) << std::endl;

  LinkSharedPtr l0 = my_robot.getLinkByName("link_0");

  auto contact_wrench_key = ContactWrenchKey(
      l0->getID(), contact_points["link_0"].contact_id, 0);
  gtsam::Vector contact_wrench_optimized =
      results.at<gtsam::Vector>(contact_wrench_key);

  graph_builder.saveGraph("../../visualization/factor_graph.json", graph,
                          results, my_robot, 0, false);

  EXPECT(assert_equal((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0.294).finished(),
                      contact_wrench_optimized));

  for (auto joint : my_robot.joints())
    EXPECT(assert_equal(
        0, results.atDouble(TorqueKey(joint->getID(), 0))));
}

// Test contacts in dynamics graph.
TEST(dynamicsFactorGraph_Contacts, dynamics_graph_biped) {
  // Load the robot from urdf file
  Robot biped =
      CreateRobotFromFile(std::string(URDF_PATH) + "/biped.urdf");

  // Add some contact points.
  ContactPoints contact_points;
  contact_points.emplace("lower0",
      ContactPoint{gtsam::Point3(0.14, 0, 0), 0, -0.54});
  contact_points.emplace("lower2",
      ContactPoint{gtsam::Point3(0.14, 0, 0), 0, -0.54});

  // Build the dynamics FG.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  auto graph_builder = DynamicsGraph();
  gtsam::NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(
      biped, 0, gravity, boost::none, contact_points, 1.0);

  // Compute inverse dynamics prior factors.
  // Inverse dynamics priors. We care about the torques.
  gtsam::Vector joint_angles = gtsam::Vector::Zero(biped.numJoints());
  gtsam::Vector joint_vels = gtsam::Vector::Zero(biped.numJoints());
  gtsam::Vector joint_accels = gtsam::Vector::Zero(biped.numJoints());
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(biped, 0, joint_angles, joint_vels,
                                          joint_accels);

  // Specify pose and twist priors for base.
  auto body = biped.getLinkByName("body");
  prior_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      PoseKey(body->getID(), 0), body->wTcom(),
      graph_builder.opt().bp_cost_model));
  prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      TwistKey(body->getID(), 0), gtsam::Vector6::Zero(),
      graph_builder.opt().bv_cost_model));
  prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      TwistAccelKey(body->getID(), 0), gtsam::Vector6::Zero(),
      graph_builder.opt().ba_cost_model));
  graph.add(prior_factors);

  // Add min torque factor.
  for (auto joint : biped.joints())
    graph.add(
        MinTorqueFactor(TorqueKey(joint->getID(), 0),
                                    gtsam::noiseModel::Isotropic::Sigma(1, 1)));

  // Set initial values.
  gtsam::Values init_values =
      ZeroValues(biped, 0, 0.0, contact_points);

//   graph_builder.printGraph(graph);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  gtsam::Values results = optimizer.optimize();

//   std::cout << "Error: " << graph.error(results) << std::endl;

  double normal_force = 0;
  for (auto&& contact_point : contact_points) {
    LinkSharedPtr l = biped.getLinkByName("lower0");
    auto contact_wrench_key =
        ContactWrenchKey(l->getID(), contact_point.second.contact_id, 0);
    gtsam::Vector contact_wrench_optimized =
        results.at<gtsam::Vector>(contact_wrench_key);
    gtsam::Pose3 pose_optimized =
        results.at<gtsam::Pose3>(PoseKey(l->getID(), 0));
    gtsam::Pose3 comTc =
        gtsam::Pose3(pose_optimized.rotation(), contact_point.second.contact_point);
    normal_force =
        normal_force + (comTc.AdjointMap() * contact_wrench_optimized)[5];
  }

  // Assert that the normal forces at the contacts sum up to the robot's weight.
  //TODO(Varun) Check this test, total weight should be 187.8615
  EXPECT(assert_equal(187.67, normal_force, 1e-2));
}

// check joint limit factors
TEST(jointlimitFactors, simple_urdf) {
  using simple_urdf::my_robot;
  auto graph_builder = DynamicsGraph();
  NonlinearFactorGraph joint_limit_factors =
      graph_builder.jointLimitFactors(my_robot, 0);

  // 4 joint limit factors per joint (angle, velocity, acceleration, torque).
  EXPECT(assert_equal(static_cast<int>(my_robot.joints().size()) * 4,
                      joint_limit_factors.keys().size()));
}

// Test contacts in dynamics graph.
TEST(dynamicsFactorGraph_Contacts, dynamics_graph_simple_rrr) {
  // Load the robot from urdf file
  Robot my_robot = CreateRobotFromFile(
      std::string(SDF_PATH) + "/test/simple_rrr.sdf", "simple_rrr_sdf");

  // Add some contact points.
  ContactPoints contact_points;
  contact_points.emplace("link_0",
      ContactPoint{gtsam::Point3(0, 0, -0.1), 0, 0});

  // Build the dynamics FG.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  auto graph_builder = DynamicsGraph();
  gtsam::NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(
      my_robot, 0, gravity, boost::none, contact_points, 1.0);

  // Compute inverse dynamics prior factors.
  gtsam::Vector joint_accels = gtsam::Vector::Zero(my_robot.numJoints());
  gtsam::Vector joint_angles = gtsam::Vector::Zero(my_robot.numJoints());
  gtsam::Vector joint_vels = gtsam::Vector::Zero(my_robot.numJoints());
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(my_robot, 0, joint_angles,
                                          joint_vels, joint_accels);

  // Specify pose and twist priors for one leg.
  prior_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      PoseKey(my_robot.getLinkByName("link_0")->getID(), 0),
      my_robot.getLinkByName("link_0")->wTcom(),
      gtsam::noiseModel::Constrained::All(6)));
  prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      TwistKey(my_robot.getLinkByName("link_0")->getID(), 0),
      gtsam::Vector6::Zero(), gtsam::noiseModel::Constrained::All(6)));
  graph.add(prior_factors);

  // Add min torque factor.
  for (auto joint : my_robot.joints())
    graph.add(
      MinTorqueFactor(
        TorqueKey(joint->getID(), 0),
          gtsam::noiseModel::Isotropic::Sigma(1, 0.1)));

  graph_builder.printGraph(graph);

  // Set initial values.
  gtsam::Values init_values =
      ZeroValues(my_robot, 0, 0.0, contact_points);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values results = optimizer.optimize();
  std::cout << "Error: " << graph.error(results) << std::endl;

  LinkSharedPtr l0 = my_robot.getLinkByName("link_0");

  auto contact_wrench_key = ContactWrenchKey(
      l0->getID(), contact_points["link_0"].contact_id, 0);
  gtsam::Vector contact_wrench_optimized =
      results.at<gtsam::Vector>(contact_wrench_key);

  graph_builder.saveGraph("../../visualization/factor_graph.json", graph,
                          results, my_robot, 0, false);

  EXPECT(assert_equal((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0.392).finished(),
                      contact_wrench_optimized));

  for (auto joint : my_robot.joints())
    EXPECT(assert_equal(
        0, results.atDouble(TorqueKey(joint->getID(), 0))));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
