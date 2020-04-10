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
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#define GROUND_HEIGHT -0.191839

using gtdynamics::PoseKey, gtsam::Vector6, gtsam::Vector3, gtsam::Vector,
    gtdynamics::JointAngleKey, gtdynamics::JointVelKey,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtdynamics::ContactPoints,
    gtdynamics::ContactPoint, gtdynamics::ZeroValues, gtdynamics::PhaseKey,
    gtdynamics::TwistKey, gtdynamics::TwistAccelKey, gtdynamics::Robot,
    std::vector, std::string;

int main(int argc, char** argv) {
  // Load the quadruped. Based on the vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  auto vision60 = Robot("../vision60.urdf");

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
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  double mu = 1.0;

  // All contacts.
  auto c1 = ContactPoint{"lower0", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};
  auto c2 = ContactPoint{"lower1", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};
  auto c3 = ContactPoint{"lower2", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};
  auto c4 = ContactPoint{"lower3", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT};

  // Contact points for each phase. In this example the robot moves one foot at
  // a time.
  ContactPoints p0 = {c1, c2, c3, c4};  // Initially stationary.
  ContactPoints p1 = {c1, c2, c4};      // Front right leg swing.
  ContactPoints p2 = {c1, c3, c4};      // Hind left leg swing.
  ContactPoints p3 = {c2, c3, c4};      // Front left leg swing.
  ContactPoints p4 = {c1, c2, c3};      // Hind right leg swing.
  ContactPoints p5 = {c1, c2, c3, c4};  // Finally stationary.

  vector<ContactPoints> phase_contact_points;
  phase_contact_points = {p0, p1, p2, p3, p4, p5};

  // Robot model for each phase.
  vector<Robot> robots(phase_contact_points.size(), vision60);

  // Number of descretized timesteps for each phase.
  int steps_per_phase = 100;
  vector<int> phase_steps(phase_contact_points.size(), steps_per_phase);
  int dt_f = phase_contact_points.size() *
             steps_per_phase;  // The final discretized timestep.
  double dt_des = 1. / 240.;

  // Collocation scheme.
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

  // Transition graphs. Transition graph in this case is equivalent to the
  // stationary case.
  vector<gtsam::NonlinearFactorGraph> transition_graphs;
  transition_graphs = {
      graph_builder.dynamicsFactorGraph(robots[0], 1 * steps_per_phase, gravity,
                                        boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[1], 2 * steps_per_phase, gravity,
                                        boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[2], 3 * steps_per_phase, gravity,
                                        boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[3], 4 * steps_per_phase, gravity,
                                        boost::none, p0, mu),
      graph_builder.dynamicsFactorGraph(robots[4], 5 * steps_per_phase, gravity,
                                        boost::none, p0, mu)};

  // Initial values for transition graphs.
  vector<gtsam::Values> transition_graph_init;
  double gaussian_noise = 1e-8;
  transition_graph_init.push_back(
      ZeroValues(robots[0], 1 * steps_per_phase, gaussian_noise, p0));
  transition_graph_init.push_back(
      ZeroValues(robots[1], 2 * steps_per_phase, gaussian_noise, p0));
  transition_graph_init.push_back(
      ZeroValues(robots[2], 3 * steps_per_phase, gaussian_noise, p0));
  transition_graph_init.push_back(
      ZeroValues(robots[3], 4 * steps_per_phase, gaussian_noise, p0));
  transition_graph_init.push_back(
      ZeroValues(robots[4], 5 * steps_per_phase, gaussian_noise, p0));

  // Construct the multi-phase trajectory factor graph.
  auto graph = graph_builder.multiPhaseTrajectoryFG(
      robots, phase_steps, transition_graphs, collocation, gravity, boost::none,
      phase_contact_points, mu);

  auto base_link = vision60.getLinkByName("body");
  gtsam::NonlinearFactorGraph objective_factors;

  // The task: to reach a desired pose.
  vector<gtsam::Pose3> wTb_t;
  vector<int> ts;
  wTb_t.push_back(gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0),
                               gtsam::Point3(0.25, 0.0, 0.1)));
  ts.push_back(dt_f);

  for (int t = 0; t <= dt_f; t++)
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(base_link->getID(), t), wTb_t[0],
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));

  // Add link boundary conditions to FG.
  Vector6 link_twist_init = Vector6::Zero(), link_twist_final = Vector6::Zero(),
          link_accel_init = Vector6::Zero(), link_accel_final = Vector6::Zero();
  for (auto&& link : vision60.links()) {
    // Initial link pose, twists, accelerations.
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(link->getID(), 0), link->wTcom(),
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->getID(), 0), link_twist_init,
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistAccelKey(link->getID(), 0), link_accel_init,
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));

    // Final link twists, accelerations.
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->getID(), dt_f), link_twist_final,
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistAccelKey(link->getID(), dt_f), link_accel_final,
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));
  }

  // Add joint boundary conditions to FG.
  Vector joint_angles_init = Vector::Zero(12),
         joint_vels_init = Vector::Zero(12),
         joint_accels_init = Vector::Zero(12),
         joint_vels_final = Vector::Zero(12),
         joint_accels_final = Vector::Zero(12);
  for (auto&& joint : vision60.joints()) {
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAngleKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAccelKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->getID(), dt_f), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_objectives)));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAccelKey(joint->getID(), dt_f), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_objectives)));
  }

  // Add prior factor constraining all Phase keys to have duration of 1 / 240.
  for (int phase = 0; phase < phase_steps.size(); phase++)
    objective_factors.add(gtsam::PriorFactor<double>(
        PhaseKey(phase), dt_des,
        gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

  // Add min torque objectives.
  for (int t = 0; t <= dt_f; t++) {
    for (auto&& joint : vision60.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          TorqueKey(joint->getID(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }
  graph.add(objective_factors);

  // Initialize solution.
  gtsam::Values init_vals;
  string initialization_technique = "zeros";
  if (initialization_technique == "zeros")
    init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
        robots, phase_steps, transition_graph_init, dt_des, gaussian_noise,
        phase_contact_points);
  else if (initialization_technique == "inverse_kinematics")
    init_vals = gtdynamics::MultiPhaseInverseKinematicsTrajectory(
        robots, base_link->name(), phase_steps, base_link->wTcom(), wTb_t, ts,
        transition_graph_init, dt_des, gaussian_noise, phase_contact_points);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setlambdaInitial(1e-3);
  params.setlambdaLowerBound(1e0);
  params.setlambdaUpperBound(1e7);
  params.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  gtsam::Values results = optimizer.optimize();

  // Log the joint angles, velocities, accels, torques, and current goal pose.
  vector<string> jnames;
  for (auto&& joint : vision60.joints()) jnames.push_back(joint->name());
  string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;
  traj_file.open("../traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",t"
            << "\n";
  int t = 0;
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
      vector<string> vals;
      for (auto&& joint : vision60.joints())
        vals.push_back(
            std::to_string(results.atDouble(JointAngleKey(joint->getID(), t))));
      for (auto&& joint : vision60.joints())
        vals.push_back(
            std::to_string(results.atDouble(JointVelKey(joint->getID(), t))));
      for (auto&& joint : vision60.joints())
        vals.push_back(
            std::to_string(results.atDouble(JointAccelKey(joint->getID(), t))));
      for (auto&& joint : vision60.joints())
        vals.push_back(
            std::to_string(results.atDouble(TorqueKey(joint->getID(), t))));

      vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));

      t++;
      string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
  traj_file.close();

  return 0;
}
