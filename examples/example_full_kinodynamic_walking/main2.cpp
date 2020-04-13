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
#include <gtdynamics/factors/PointGoalFactor.h>
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
    gtdynamics::JointAngleKey, gtdynamics::JointVelKey, gtsam::Point3,
    gtsam::Rot3, gtsam::Pose3,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtdynamics::ContactPoints,
    gtdynamics::ContactPoint, gtdynamics::ZeroValues, gtdynamics::PhaseKey,
    gtdynamics::TwistKey, gtdynamics::TwistAccelKey, gtdynamics::Robot,
    std::vector, std::string;

int main(int argc, char** argv) {
  // Load the quadruped. Based on the vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  auto vision60 = Robot("../vision60.urdf");

  double sigma_dynamics = 1e-5;    // Variance of dynamics constraints.
  double sigma_objectives = 1e-4;  // Variance of additional objectives.

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
  auto c0 = ContactPoint{"lower0", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT}; // Front left.
  auto c1 = ContactPoint{"lower1", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT}; // Hind left.
  auto c2 = ContactPoint{"lower2", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT}; // Front right.
  auto c3 = ContactPoint{"lower3", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT}; // Hind right.

  // Contact points for each phase. In this example the robot moves one foot at
  // a time.
  ContactPoints p0 = {c0, c1, c2, c3};  // Initially stationary.
  ContactPoints p1 = {    c1, c2    };  // Front left leg and hind right swing.
  ContactPoints p2 = {c0,         c3};  // Front right leg and hind left swing.

  // One step at a time.
  ContactPoints p3 = {c0            };  // Front right leg swing.
  ContactPoints p4 = {    c1        };  // Front left leg swing.
  ContactPoints p5 = {        c2    };  // Front hind right swing.
  ContactPoints p6 = {            c3};  // Front hind left swing.

  vector<ContactPoints> phase_contact_points;
  phase_contact_points = {p0, p1, p2};

  // Robot model for each phase.
  vector<Robot> robots(phase_contact_points.size(), vision60);

  // Number of descretized timesteps for each phase.
  int steps_per_phase = 100;
  vector<int> phase_steps(phase_contact_points.size(), steps_per_phase);
  int dt_f = phase_contact_points.size() *
             steps_per_phase;  // The final discretized timestep.
  double dt_des = 1. / 240.;

  // Collocation scheme.
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Trapezoidal;

  // Transition graphs. Transition graph in this case is equivalent to the
  // stationary case.
  vector<gtsam::NonlinearFactorGraph> transition_graphs;
  transition_graphs = {
      graph_builder.dynamicsFactorGraph(robots[0], 1 * steps_per_phase, gravity,
                                        boost::none, p1, mu),
      graph_builder.dynamicsFactorGraph(robots[0], 2 * steps_per_phase, gravity,
                                        boost::none, p2, mu)};

  // Initial values for transition graphs.
  vector<gtsam::Values> transition_graph_init;
  double gaussian_noise = 1e-8;
  transition_graph_init.push_back(
      ZeroValues(robots[0], 1 * steps_per_phase, gaussian_noise, p1));
  transition_graph_init.push_back(
      ZeroValues(robots[0], 2 * steps_per_phase, gaussian_noise, p2));

  // Construct the multi-phase trajectory factor graph.
  auto graph = graph_builder.multiPhaseTrajectoryFG(
      robots, phase_steps, transition_graphs, collocation, gravity, boost::none,
      phase_contact_points, mu);

  // The task: To move the leg lower3's contact point to the specified point.
  auto base_link = vision60.getLinkByName("body");
  auto l0 = vision60.getLinkByName("lower0");
  auto l1 = vision60.getLinkByName("lower1");
  auto l2 = vision60.getLinkByName("lower2");
  auto l3 = vision60.getLinkByName("lower3");
  gtsam::NonlinearFactorGraph objective_factors;

  auto c0_new = l0->wTcom() * Pose3(Rot3(), c0.contact_point + Point3(0.1, 0, 0.025)),
       c3_new = l3->wTcom() * Pose3(Rot3(), c3.contact_point + Point3(0.1, 0, 0.025));
  int t_i = 1 * steps_per_phase;
  int t_f = 2 * steps_per_phase;

  for (int t = t_i; t <= t_f; t++) {
    // Contact point objecives.
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l0->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c0.contact_point), c0_new.translation()));
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l1->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c1.contact_point), (l1->wTcom() * Pose3(Rot3(), c1.contact_point)).translation()));
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l2->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c2.contact_point), (l2->wTcom() * Pose3(Rot3(), c2.contact_point)).translation()));
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l3->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c3.contact_point), c3_new.translation()));

    // objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
    //   PoseKey(base_link->getID(), t),
    //   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.05, 0.0, 0.1)),
    //   gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));
  }

  auto c1_new = l1->wTcom() * Pose3(Rot3(), c1.contact_point + Point3(0.1, 0, 0.025)),
       c2_new = l2->wTcom() * Pose3(Rot3(), c2.contact_point + Point3(0.1, 0, 0.025));
  t_i = 2 * steps_per_phase + 1;
  t_f = 3 * steps_per_phase;
  for (int t = t_i; t <= t_f; t++) {
    // Contact point objecives.
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l0->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c0.contact_point), Point3(c0_new.x(), c0_new.y(), GROUND_HEIGHT)));
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l1->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c1.contact_point), c1_new.translation()));
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l2->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c2.contact_point), c2_new.translation()));
    objective_factors.add(gtdynamics::PointGoalFactor(
      PoseKey(l3->getID(), t), gtsam::noiseModel::Isotropic::Sigma(3, sigma_objectives),
      Pose3(Rot3(), c3.contact_point), Point3(c3_new.x(), c3_new.y(), GROUND_HEIGHT)));

    // objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
    //   PoseKey(base_link->getID(), t),
    //   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.1)),
    //   gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));
  }

  for (int t = 0; t <= dt_f; t++) {
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      PoseKey(base_link->getID(), t),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.1)),
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives)));
  }

  // Add link boundary conditions to FG.
  Vector6 link_twist_init = Vector6::Zero(), link_twist_final = Vector6::Zero(),
          link_accel_final = Vector6::Zero();
  for (auto&& link : vision60.links()) {
    // Initial link pose, twists, accelerations.
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(link->getID(), 0), link->wTcom(),
        gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->getID(), 0), link_twist_init,
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
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1 * 10)));
  }
  graph.add(objective_factors);

  // Initialize solution.
  gtsam::Values init_vals;
  string initialization_technique = "zeros";
//   if (initialization_technique == "zeros")
    init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
        robots, phase_steps, transition_graph_init, dt_des, gaussian_noise,
        phase_contact_points);
//   else if (initialization_technique == "inverse_kinematics")
//     init_vals = gtdynamics::MultiPhaseInverseKinematicsTrajectory(
//         robots, base_link->name(), phase_steps, base_link->wTcom(), wTb_t, ts,
//         transition_graph_init, dt_des, gaussian_noise, phase_contact_points);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
//   params.setlambdaInitial(1e-3);
//   params.setlambdaLowerBound(1e0);
//   params.setlambdaUpperBound(1e7);
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
