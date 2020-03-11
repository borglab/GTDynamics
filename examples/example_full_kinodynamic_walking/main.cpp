/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a walking legged robot with contacts.
 * @Author: Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PoseGoalFactor.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#include "initialize_solutions.hpp"

#define GROUND_HEIGHT -0.191839

int main(int argc, char** argv) {
  // Load the quadruped. Based on the vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  auto vision60 = gtdynamics::Robot("../vision60.urdf");

  double sigma_dynamics = 1e-5;    // Variance of dynamics constraints.
  double sigma_objectives = 1e-3;  // Variance of additional objectives.

  // Initialize graph builder with desired dynamics constraint variances.
  auto opt = gtdynamics::OptimizerSetting();
  opt.bp_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.bv_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.ba_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.p_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.v_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.fa_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics);
  opt.t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.cp_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.cfriction_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.cv_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  opt.ca_cost_model = gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  opt.planar_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(3, sigma_dynamics);
  opt.prior_q_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.prior_qv_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.prior_qa_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.prior_t_cost_model =
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.q_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.v_col_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  opt.time_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics);
  auto graph_builder = gtdynamics::DynamicsGraph(opt);

  // Env parameters.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  double mu = 1.0;

  // All contacts.
  auto c1 = gtdynamics::ContactPoint{
      "lower0", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};
  auto c2 = gtdynamics::ContactPoint{
      "lower1", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};
  auto c3 = gtdynamics::ContactPoint{
      "lower2", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};
  auto c4 = gtdynamics::ContactPoint{
      "lower3", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};

  // Contact points for each phase. In this example the robot moves one foot at
  // a time.
  gtdynamics::ContactPoints p0 = {c1, c2, c3, c4};  // Initially stationary.
  gtdynamics::ContactPoints p1 = {c1, c2, c4};      // Front right leg swing.
  gtdynamics::ContactPoints p2 = {c1, c3, c4};      // Hind left leg swing.
  gtdynamics::ContactPoints p3 = {c2, c3, c4};      // Front left leg swing.
  gtdynamics::ContactPoints p4 = {c1, c2, c3};      // Hind right leg swing.
  gtdynamics::ContactPoints p5 = {c1, c2, c3, c4};  // Finally stationary.

  std::vector<gtdynamics::ContactPoints> phase_contact_points;
  phase_contact_points = {p0, p1, p2, p3, p4, p5};

  // Robot model for each phase.
  std::vector<gtdynamics::Robot> robots(6, vision60);

  // Number of descretized timesteps for each phase.
  int steps_per_phase = 300;
  std::vector<int> phase_steps(6, steps_per_phase);
  int dt_f = 6 * steps_per_phase; // The final discretized timestep.

  // Collocation scheme.
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Trapezoidal;

  // Transition graphs. Transition graph in this case is equivalent to the
  // stationary case.
  std::vector<gtsam::NonlinearFactorGraph> transition_graphs;
  transition_graphs = {
      graph_builder.dynamicsFactorGraph(robots[0], steps_per_phase, gravity, boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[1], 2 * steps_per_phase, gravity, boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[2], 3 * steps_per_phase, gravity, boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[3], 4 * steps_per_phase, gravity, boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[4], 5 * steps_per_phase, gravity, boost::none, p0, mu)
  };

  // Construct the multi-phase trajectory factor graph.
  auto graph = graph_builder.multiPhaseTrajectoryFG(
    robots, phase_steps, transition_graphs, collocation, gravity, boost::none,
    phase_contact_points, mu);

  // Specify boundary conditions for base and joints.
  gtsam::Pose3 base_pose_init = vision60.getLinkByName("body")->wTcom();
  gtsam::Vector6 base_twist_init = gtsam::Vector6::Zero(),
                 base_twist_final = gtsam::Vector6::Zero(),
                 base_accel_init = gtsam::Vector6::Zero(),
                 base_accel_final = gtsam::Vector6::Zero();
  gtsam::Vector joint_angles_init = gtsam::Vector::Zero(12),
                joint_vels_init = gtsam::Vector::Zero(12),
                joint_accels_init = gtsam::Vector::Zero(12),
                joint_vels_final = gtsam::Vector::Zero(12),
                joint_accels_final = gtsam::Vector::Zero(12);

  // The task: to reach a desired pose.
  gtsam::Pose3 des_pose_final = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, M_PI / 2), gtsam::Point3(0, 0, 0.1));
  auto des_pose_nm = gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives);

  auto base_link = vision60.getLinkByName("body");
  gtsam::NonlinearFactorGraph objective_factors;

  // Add the task objective.
  objective_factors.add(gtdynamics::PoseGoalFactor(
    gtdynamics::PoseKey(base_link->getID(),
                        dt_f), des_pose_nm, des_pose_final));

  // Add base boundary conditions to FG.
  objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtdynamics::PoseKey(base_link->getID(), 0), base_pose_init,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistKey(base_link->getID(), 0), base_twist_init,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistAccelKey(base_link->getID(), 0), base_accel_init,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistKey(base_link->getID(), dt_f), base_twist_final,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistAccelKey(base_link->getID(), dt_f), base_accel_final,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));

  // Add joint boundary conditions to FG.
  for (auto&& joint : vision60.joints()) {
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointAngleKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointVelKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointAccelKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointVelKey(joint->getID(), dt_f), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_objectives)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointAccelKey(joint->getID(), dt_f), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_objectives)));
  }

  // Add min torque objectives.
  for (int t = 0; t <= dt_f; t++) {
    for (auto&& joint : vision60.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          gtdynamics::TorqueKey(joint->getID(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }
  graph.add(objective_factors);

  // Initialize values.
  gtsam::Values init_vals = multiPhaseZeroValuesTrajectory(
    graph_builder, robots, phase_steps, phase_contact_points);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  gtsam::Values results = optimizer.optimize();

  return 0;
}