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
#include "gtdynamics/utils/initialize_solution_utils.h"
#include "gtdynamics/utils/utils.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;

int DEBUG_SIMPLE_OPTIMIZATION_EXAMPLE = 0;
int DEBUG_FOUR_BAR_LINKAGE_ILS_EXAMPLE = 0;

using gtsam::assert_equal;
using gtsam::NonlinearFactorGraph;
using gtsam::PriorFactor;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector6;
using std::vector;

// ============================ VALUES-STYLE ================================

// Test linear dynamics graph of a two-link robot, base fixed, with gravity
TEST(linearDynamicsFactorGraph, simple_urdf_eq_mass_values) {
  using simple_urdf_eq_mass::robot;

  std::string prior_link_name = "l1";
  auto l1 = robot.link(prior_link_name);

  // TODO(frank): joint numbering starts at 0?? Should fix.
  Values values;
  int t = 777;
  auto j = robot.joint("j1")->id();
  InsertPose(&values, l1->id(), t, l1->wTcom());
  InsertTwist(&values, l1->id(), t, gtsam::Z_6x1);

  // Do forward kinematics.
  Values fk_results = robot.forwardKinematics(values, t, prior_link_name);

  DynamicsGraph graph_builder(simple_urdf_eq_mass::gravity,
                              simple_urdf_eq_mass::planar_axis);

  // Test forward dynamics.
  Values known_torques = fk_results;
  InsertTorque(&known_torques, j, t, 1.0);
  Values result_fd = graph_builder.linearSolveFD(robot, t, known_torques);
  EXPECT(assert_equal(4.0, JointAccel(result_fd, j, t), 1e-3));

  // test inverse dynamics
  Values desired_accels = fk_results;
  InsertJointAccel(&desired_accels, j, t, 4.0);
  Values result_id = graph_builder.linearSolveID(robot, t, desired_accels);
  EXPECT(assert_equal(1.0, Torque(result_id, j, t), 1e-3));
}

Values zero_values(const Robot& robot, size_t t, bool insert_accels = false) {
  Values values;
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    InsertJointAngle(&values, j, t, 0.0);
    InsertJointVel(&values, j, t, 0.0);
    if (insert_accels) {
      InsertJointAccel(&values, j, t, 0.0);
    }
  }
  return values;
}

// Test forward dynamics with gravity of a two-link robot, with base link fixed
TEST(dynamicsFactorGraph_FD, simple_urdf_eq_mass) {
  using simple_urdf_eq_mass::robot;

  // build the dynamics factor graph
  size_t t = 777;
  DynamicsGraph graph_builder(simple_urdf_eq_mass::gravity,
                              simple_urdf_eq_mass::planar_axis);
  auto graph = graph_builder.dynamicsFactorGraph(robot, t);

  // Create values with rest kinematics and unit torques
  Values known_values = zero_values(robot, t);
  for (auto&& joint : robot.joints()) {
    InsertTorque(&known_values, joint->id(), t, 1.0);
  }

  graph.add(graph_builder.forwardDynamicsPriors(robot, t, known_values));
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : robot.links()) {
    int i = link->id();
    graph.addPrior(internal::PoseKey(i, t), link->wTcom(),
                   graph_builder.opt().bp_cost_model);
    graph.addPrior<Vector6>(internal::TwistKey(i, t), gtsam::Z_6x1,
                            graph_builder.opt().bv_cost_model);
  }

  gtsam::GaussNewtonOptimizer optimizer(graph, ZeroValues(robot, t));
  Values result = optimizer.optimize();

  gtsam::Vector actual_qAccel = DynamicsGraph::jointAccels(robot, result, t);
  gtsam::Vector expected_qAccel = (gtsam::Vector(1) << 4).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 1e-3));
}

// ========================== OLD_STYLE BELOW ===============================

// Test forward dynamics with gravity of a four-bar linkage
TEST(dynamicsFactorGraph_FD, four_bar_linkage_pure) {
  // Load the robot from urdf file
  using four_bar_linkage_pure::robot;

  Values known_values = zero_values(robot, 0);
  gtsam::Vector torques = (gtsam::Vector(4) << 1, 0, 1, 0).finished();
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    InsertTorque(&known_values, j, 0, torques[j]);
  }

  // build the dynamics factor graph
  DynamicsGraph graph_builder(four_bar_linkage_pure::gravity,
                              four_bar_linkage_pure::planar_axis);

  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.forwardDynamicsPriors(robot, 0, known_values);
  // still need to add pose and twist priors since no link is fixed in this case
  for (auto link : robot.links()) {
    int i = link->id();
    prior_factors.addPrior(internal::PoseKey(i, 0), link->wTcom(),
                           graph_builder.opt().bp_cost_model);
    prior_factors.addPrior<Vector6>(internal::TwistKey(i, 0), gtsam::Z_6x1,
                                    graph_builder.opt().bv_cost_model);
  }

  auto graph = graph_builder.dynamicsFactorGraph(robot, 0);
  graph.add(prior_factors);

  Values init_values = ZeroValues(robot, 0);

  // test the four bar linkage FD in the free-floating scenario
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values result = optimizer.optimize();
  gtsam::Vector actual_qAccel = DynamicsGraph::jointAccels(robot, result, 0);
  gtsam::Vector expected_qAccel = (gtsam::Vector(4) << 1, -1, 1, -1).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel, 1e-4));

  // test the condition when we fix link "l1"
  robot.fixLink("l1");
  graph = graph_builder.dynamicsFactorGraph(robot, 0);
  graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer2(graph, init_values);
  result = optimizer2.optimize();

  actual_qAccel = DynamicsGraph::jointAccels(robot, result, 0);
  expected_qAccel = (gtsam::Vector(4) << 0.25, -0.25, 0.25, -0.25).finished();
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

// test jumping robot
TEST(dynamicsFactorGraph_FD, jumping_robot) {
  using jumping_robot::robot;

  Values known_values = zero_values(robot, 0);
  double torque3 = 0;
  double torque2 = 0.5;
  Vector torques =
      (Vector(6) << 0, torque2, torque3, torque3, torque2, 0).finished();
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    InsertTorque(&known_values, j, 0, torques[j]);
  }

  // build the dynamics factor graph
  DynamicsGraph graph_builder(jumping_robot::gravity,
                              jumping_robot::planar_axis);
  auto graph = graph_builder.dynamicsFactorGraph(robot, 0);
  graph.add(graph_builder.forwardDynamicsPriors(robot, 0, known_values));

  // test jumping robot FD
  gtsam::GaussNewtonOptimizer optimizer(graph, ZeroValues(robot, 0));
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
  Vector actual_qAccel = DynamicsGraph::jointAccels(robot, result, 0);
  EXPECT(assert_equal(expected_qAccel, actual_qAccel));
}

TEST(collocationFactors, simple_urdf) {
  DynamicsGraph graph_builder;
  using simple_urdf::robot;
  double dt = 1;
  int t = 0;
  int j = robot.joints()[0]->id();

  NonlinearFactorGraph prior_factors;
  prior_factors.add(
      PriorFactor<double>(internal::JointAngleKey(j, t), 1,
                          graph_builder.opt().prior_q_cost_model));
  prior_factors.add(PriorFactor<double>(
      internal::JointVelKey(j, t), 1, graph_builder.opt().prior_qv_cost_model));
  prior_factors.add(
      PriorFactor<double>(internal::JointAccelKey(j, t), 1,
                          graph_builder.opt().prior_qa_cost_model));
  prior_factors.add(
      PriorFactor<double>(internal::JointAccelKey(j, t + 1), 2,
                          graph_builder.opt().prior_qa_cost_model));

  Values init_values;
  InsertJointAngle(&init_values, j, t, 0.0);
  InsertJointVel(&init_values, j, t, 0.0);
  InsertJointAccel(&init_values, j, t, 0.0);
  InsertJointAngle(&init_values, j, t + 1, 0.0);
  InsertJointVel(&init_values, j, t + 1, 0.0);
  InsertJointAccel(&init_values, j, t + 1, 0.0);

  // test trapezoidal
  NonlinearFactorGraph trapezoidal_graph;
  trapezoidal_graph.add(graph_builder.collocationFactors(
      robot, t, dt, CollocationScheme::Trapezoidal));
  trapezoidal_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_t(trapezoidal_graph, init_values);
  Values trapezoidal_result = optimizer_t.optimize();

  EXPECT(assert_equal(2.75, JointAngle(trapezoidal_result, j, t + 1)));
  EXPECT(assert_equal(2.5, JointVel(trapezoidal_result, j, t + 1)));

  // test Euler
  NonlinearFactorGraph euler_graph;
  euler_graph.add(
      graph_builder.collocationFactors(robot, t, dt, CollocationScheme::Euler));
  euler_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_e(euler_graph, init_values);
  Values euler_result = optimizer_e.optimize();

  EXPECT(assert_equal(2.0, JointAngle(euler_result, j, t + 1)));
  EXPECT(assert_equal(2.0, JointVel(euler_result, j, t + 1)));

  // test the scenario with dt as a variable
  int phase = 0;
  init_values.insert(PhaseKey(phase), 0.0);
  prior_factors.add(PriorFactor<double>(PhaseKey(phase), dt,
                                        graph_builder.opt().time_cost_model));

  // multi-phase euler
  NonlinearFactorGraph mp_euler_graph;
  mp_euler_graph.add(graph_builder.multiPhaseCollocationFactors(
      robot, t, phase, CollocationScheme::Euler));
  mp_euler_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_mpe(mp_euler_graph, init_values);
  Values mp_euler_result = optimizer_mpe.optimize();

  EXPECT(assert_equal(2.0, JointAngle(mp_euler_result, j, t + 1)));
  EXPECT(assert_equal(2.0, JointVel(mp_euler_result, j, t + 1)));

  // multi-phase trapezoidal
  NonlinearFactorGraph mp_trapezoidal_graph;
  mp_trapezoidal_graph.add(graph_builder.collocationFactors(
      robot, t, dt, CollocationScheme::Trapezoidal));
  mp_trapezoidal_graph.add(prior_factors);

  gtsam::GaussNewtonOptimizer optimizer_mpt(mp_trapezoidal_graph, init_values);
  Values mp_trapezoidal_result = optimizer_mpt.optimize();

  EXPECT(assert_equal(2.75, JointAngle(mp_trapezoidal_result, j, t + 1)));
  EXPECT(assert_equal(2.5, JointVel(mp_trapezoidal_result, j, t + 1)));
}

// test forward dynamics of a trajectory
TEST(dynamicsTrajectoryFG, simple_urdf_eq_mass) {
  using simple_urdf_eq_mass::robot;

  robot.fixLink("l1");
  int j = robot.joints()[0]->id();
  DynamicsGraph graph_builder(simple_urdf_eq_mass::gravity,
                              simple_urdf_eq_mass::planar_axis);

  int num_steps = 2;
  double dt = 1;

  Values known_values = zero_values(robot, 0);
  for (int i = 0; i <= num_steps; i++) {
    for (auto&& joint : robot.joints()) {
      int j = joint->id();
      InsertTorque(&known_values, j, i, i * 1.0 + 1.0);
    }
  }

  Values init_values = ZeroValuesTrajectory(robot, num_steps);

  // test Euler
  auto euler_graph = graph_builder.trajectoryFG(robot, num_steps, dt,
                                                CollocationScheme::Euler);
  euler_graph.add(
      graph_builder.trajectoryFDPriors(robot, num_steps, known_values));

  gtsam::GaussNewtonOptimizer optimizer_e(euler_graph, init_values);
  Values euler_result = optimizer_e.optimize();

  EXPECT(assert_equal(0.0, JointAngle(euler_result, j, 1)));
  EXPECT(assert_equal(1.0, JointVel(euler_result, j, 1)));
  EXPECT(assert_equal(2.0, JointAccel(euler_result, j, 1)));
  EXPECT(assert_equal(1.0, JointAngle(euler_result, j, 2)));
  EXPECT(assert_equal(3.0, JointVel(euler_result, j, 2)));
  EXPECT(assert_equal(3.0, JointAccel(euler_result, j, 2)));

  // test trapezoidal
  auto trapezoidal_graph = graph_builder.trajectoryFG(
      robot, num_steps, dt, CollocationScheme::Trapezoidal);
  trapezoidal_graph.add(
      graph_builder.trajectoryFDPriors(robot, num_steps, known_values));

  gtsam::GaussNewtonOptimizer optimizer_t(trapezoidal_graph, init_values);
  Values trapezoidal_result = optimizer_t.optimize();

  EXPECT(assert_equal(0.75, JointAngle(trapezoidal_result, j, 1)));
  EXPECT(assert_equal(1.5, JointVel(trapezoidal_result, j, 1)));
  EXPECT(assert_equal(2.0, JointAccel(trapezoidal_result, j, 1)));
  EXPECT(assert_equal(3.5, JointAngle(trapezoidal_result, j, 2)));
  EXPECT(assert_equal(4.0, JointVel(trapezoidal_result, j, 2)));
  EXPECT(assert_equal(3.0, JointAccel(trapezoidal_result, j, 2)));

  // test the scenario with dt as a variable
  vector<int> phase_steps{1, 1};
  auto transition_graph = graph_builder.dynamicsFactorGraph(robot, 1);
  vector<NonlinearFactorGraph> transition_graphs{transition_graph};
  double dt0 = 1;
  double dt1 = 2;
  NonlinearFactorGraph mp_prior_graph =
      graph_builder.trajectoryFDPriors(robot, num_steps, known_values);
  mp_prior_graph.add(PriorFactor<double>(PhaseKey(0), dt0,
                                         graph_builder.opt().time_cost_model));
  mp_prior_graph.add(PriorFactor<double>(PhaseKey(1), dt1,
                                         graph_builder.opt().time_cost_model));
  init_values = ZeroValuesTrajectory(robot, num_steps, 2);

  // multi-phase Euler
  NonlinearFactorGraph mp_euler_graph = graph_builder.multiPhaseTrajectoryFG(
      robot, phase_steps, transition_graphs, CollocationScheme::Euler);
  mp_euler_graph.add(mp_prior_graph);
  gtsam::GaussNewtonOptimizer optimizer_mpe(mp_euler_graph, init_values);
  Values mp_euler_result = optimizer_mpe.optimize();

  // t        0   1   2
  // dt         1   2
  // torque   1   2   3
  // q        0   0   2
  // v        0   1   5
  // a        1   2   3
  EXPECT(assert_equal(0.0, JointAngle(mp_euler_result, j, 1)));
  EXPECT(assert_equal(1.0, JointVel(mp_euler_result, j, 1)));
  EXPECT(assert_equal(2.0, JointAccel(mp_euler_result, j, 1)));
  EXPECT(assert_equal(2.0, JointAngle(mp_euler_result, j, 2)));
  EXPECT(assert_equal(5.0, JointVel(mp_euler_result, j, 2)));
  EXPECT(assert_equal(3.0, JointAccel(mp_euler_result, j, 2)));

  // multi-phase Trapezoidal
  auto mp_trapezoidal_graph = graph_builder.multiPhaseTrajectoryFG(
      robot, phase_steps, transition_graphs, CollocationScheme::Trapezoidal);
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
  EXPECT(assert_equal(0.75, JointAngle(mp_trapezoidal_result, j, 1)));
  EXPECT(assert_equal(1.5, JointVel(mp_trapezoidal_result, j, 1)));
  EXPECT(assert_equal(2.0, JointAccel(mp_trapezoidal_result, j, 1)));
  EXPECT(assert_equal(8.75, JointAngle(mp_trapezoidal_result, j, 2)));
  EXPECT(assert_equal(6.5, JointVel(mp_trapezoidal_result, j, 2)));
  EXPECT(assert_equal(3.0, JointAccel(mp_trapezoidal_result, j, 2)));
}

// Test contacts in dynamics graph.
TEST(dynamicsFactorGraph_Contacts, dynamics_graph_simple_rr) {
  // Load the robot from urdf file
  using simple_rr::robot;

  // Add some contact points.
  ContactPoints contact_points;
  contact_points.emplace("link_0", ContactPoint{gtsam::Point3(0, 0, -0.1), 0});

  // Build the dynamics FG.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  DynamicsGraph graph_builder(gravity);
  auto graph = graph_builder.dynamicsFactorGraph(robot, 0, contact_points, 1.0);

  Values known_values = zero_values(robot, 0, true);
  // Compute inverse dynamics prior factors.
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(robot, 0, known_values);

  // Specify pose and twist priors for one leg.
  prior_factors.addPrior(internal::PoseKey(robot.link("link_0")->id(), 0),
                         robot.link("link_0")->wTcom(),
                         gtsam::noiseModel::Constrained::All(6));
  prior_factors.addPrior<Vector6>(
      internal::TwistKey(robot.link("link_0")->id(), 0), gtsam::Z_6x1,
      gtsam::noiseModel::Constrained::All(6));
  graph.add(prior_factors);

  // Add min torque factor.
  for (auto joint : robot.joints())
    graph.add(MinTorqueFactor(internal::TorqueKey(joint->id(), 0),
                              gtsam::noiseModel::Isotropic::Sigma(1, 1)));

  // Set initial values.
  Values init_values = ZeroValues(robot, 0, 0.0, contact_points);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values results = optimizer.optimize();
  //   std::cout << "Error: " << graph.error(results) << std::endl;

  LinkSharedPtr l0 = robot.link("link_0");

  auto contact_wrench_key =
      ContactWrenchKey(l0->id(), contact_points["link_0"].id, 0);
  gtsam::Vector contact_wrench_optimized =
      results.at<gtsam::Vector>(contact_wrench_key);

  graph_builder.saveGraph("../../visualization/factor_graph.json", graph,
                          results, robot, 0, false);

  EXPECT(assert_equal((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0.294).finished(),
                      contact_wrench_optimized));

  for (auto joint : robot.joints())
    EXPECT(assert_equal(0, Torque(results, joint->id())));
}

// Test contacts in dynamics graph.
TEST(dynamicsFactorGraph_Contacts, dynamics_graph_biped) {
  // Load the robot from urdf file
  Robot biped = CreateRobotFromFile(kUrdfPath + std::string("/biped.urdf"));

  // Add some contact points.
  ContactPoints contact_points;
  contact_points.emplace("lower0", ContactPoint{gtsam::Point3(0.14, 0, 0), 0});
  contact_points.emplace("lower2", ContactPoint{gtsam::Point3(0.14, 0, 0), 0});

  // Build the dynamics FG.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  DynamicsGraph graph_builder(gravity);
  auto graph = graph_builder.dynamicsFactorGraph(biped, 0, contact_points, 1.0);

  // Compute inverse dynamics prior factors.
  // Inverse dynamics priors. We care about the torques.
  Values known_values = zero_values(biped, 0, true);
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(biped, 0, known_values);

  // Specify pose and twist priors for base.
  auto body = biped.link("body");
  prior_factors.addPrior(internal::PoseKey(body->id(), 0), body->wTcom(),
                         graph_builder.opt().bp_cost_model);
  prior_factors.addPrior<Vector6>(internal::TwistKey(body->id(), 0),
                                  gtsam::Z_6x1,
                                  graph_builder.opt().bv_cost_model);
  prior_factors.addPrior<Vector6>(internal::TwistAccelKey(body->id(), 0),
                                  gtsam::Z_6x1,
                                  graph_builder.opt().ba_cost_model);
  graph.add(prior_factors);

  // Add min torque factor.
  for (auto joint : biped.joints())
    graph.add(MinTorqueFactor(internal::TorqueKey(joint->id(), 0),
                              gtsam::noiseModel::Isotropic::Sigma(1, 1)));

  // Set initial values.
  Values init_values = ZeroValues(biped, 0, 0.0, contact_points);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values results = optimizer.optimize();

  //   std::cout << "Error: " << graph.error(results) << std::endl;

  double normal_force = 0;
  for (auto&& contact_point : contact_points) {
    LinkSharedPtr l = biped.link("lower0");
    auto contact_wrench_key =
        ContactWrenchKey(l->id(), contact_point.second.id, 0);
    gtsam::Vector contact_wrench_optimized =
        results.at<gtsam::Vector>(contact_wrench_key);
    gtsam::Pose3 pose_optimized = Pose(results, l->id());
    gtsam::Pose3 comTc =
        gtsam::Pose3(pose_optimized.rotation(), contact_point.second.point);
    normal_force =
        normal_force + (comTc.AdjointMap() * contact_wrench_optimized)[5];
  }

  // Assert that the normal forces at the contacts sum up to the robot's weight.
  // TODO(Varun) Check this test, total weight should be 187.8615
  EXPECT(assert_equal(187.67, normal_force, 1e-2));
}

// check joint limit factors
TEST(jointlimitFactors, simple_urdf) {
  using simple_urdf::robot;
  DynamicsGraph graph_builder;
  NonlinearFactorGraph joint_limit_factors =
      graph_builder.jointLimitFactors(robot, 0);

  // 4 joint limit factors per joint (angle, velocity, acceleration, torque).
  EXPECT(assert_equal(static_cast<int>(robot.joints().size()) * 4,
                      joint_limit_factors.keys().size()));
}

// Test contacts in dynamics graph.
TEST(dynamicsFactorGraph_Contacts, dynamics_graph_simple_rrr) {
  // Load the robot from urdf file
  Robot robot = CreateRobotFromFile(
      kSdfPath + std::string("/test/simple_rrr.sdf"), "simple_rrr_sdf");

  // Add some contact points.
  ContactPoints contact_points;
  contact_points.emplace("link_0", ContactPoint{gtsam::Point3(0, 0, -0.1), 0});

  // Build the dynamics FG.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  DynamicsGraph graph_builder(gravity);
  auto graph = graph_builder.dynamicsFactorGraph(robot, 0, contact_points, 1.0);

  // Compute inverse dynamics prior factors.
  gtsam::Values known_values = zero_values(robot, 0, true);
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(robot, 0, known_values);

  // Specify pose and twist priors for one leg.
  prior_factors.addPrior(internal::PoseKey(robot.link("link_0")->id(), 0),
                         robot.link("link_0")->wTcom(),
                         gtsam::noiseModel::Constrained::All(6));
  prior_factors.addPrior<Vector6>(
      internal::TwistKey(robot.link("link_0")->id(), 0), gtsam::Z_6x1,
      gtsam::noiseModel::Constrained::All(6));
  graph.add(prior_factors);

  // Add min torque factor.
  for (auto joint : robot.joints())
    graph.add(MinTorqueFactor(internal::TorqueKey(joint->id(), 0),
                              gtsam::noiseModel::Isotropic::Sigma(1, 0.1)));

  // Set initial values.
  Values init_values = ZeroValues(robot, 0, 0.0, contact_points);

  // Optimize!
  gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
  Values results = optimizer.optimize();
  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-9);

  LinkSharedPtr l0 = robot.link("link_0");

  auto contact_wrench_key =
      ContactWrenchKey(l0->id(), contact_points["link_0"].id, 0);
  gtsam::Vector contact_wrench_optimized =
      results.at<gtsam::Vector>(contact_wrench_key);

  graph_builder.saveGraph("../../visualization/factor_graph.json", graph,
                          results, robot, 0, false);

  EXPECT(assert_equal((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0.392).finished(),
                      contact_wrench_optimized));

  for (auto joint : robot.joints())
    EXPECT(assert_equal(0, Torque(results, joint->id())));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
