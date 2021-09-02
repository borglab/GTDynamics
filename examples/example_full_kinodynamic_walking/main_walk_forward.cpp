/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Quadruped trajectory optimization with pre-specified footholds.
 * @Author: Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <utility>

#define GROUND_HEIGHT -0.191839

using std::string;
using std::vector;

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;
using gtsam::noiseModel::Isotropic;

using namespace gtdynamics;

int main(int argc, char** argv) {
  // Load the quadruped. Based on the vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  auto robot =
      gtdynamics::CreateRobotFromFile(kUrdfPath + string("/vision60.urdf"));

  double sigma_dynamics = 1e-6;    // std of dynamics constraints.
  double sigma_objectives = 1e-4;  // std of additional objectives.

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_3 = Isotropic::Sigma(3, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_3 = Isotropic::Sigma(3, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

  // Initialize graph builder with desired dynamics constraint stds.
  // TODO(aescontrela): Make a constructor for OptimizerSetting that
  //     initializes all noise models with the same sigma.
  auto opt = gtdynamics::OptimizerSetting();
  opt.bp_cost_model = dynamics_model_6;
  opt.bv_cost_model = dynamics_model_6;
  opt.ba_cost_model = dynamics_model_6;
  opt.p_cost_model = dynamics_model_6;
  opt.v_cost_model = dynamics_model_6;
  opt.a_cost_model = dynamics_model_6;
  opt.f_cost_model = dynamics_model_6;
  opt.fa_cost_model = dynamics_model_6;
  opt.t_cost_model = dynamics_model_1;
  opt.cp_cost_model = dynamics_model_1;
  opt.cfriction_cost_model = dynamics_model_1;
  opt.cv_cost_model = dynamics_model_3;
  opt.ca_cost_model = dynamics_model_3;
  opt.planar_cost_model = dynamics_model_3;
  opt.prior_q_cost_model = dynamics_model_1;
  opt.prior_qv_cost_model = dynamics_model_1;
  opt.prior_qa_cost_model = dynamics_model_1;
  opt.prior_t_cost_model = dynamics_model_1;
  opt.q_col_cost_model = dynamics_model_1;
  opt.v_col_cost_model = dynamics_model_1;
  opt.time_cost_model = dynamics_model_1;
  auto graph_builder = gtdynamics::DynamicsGraph(opt);

  // Env parameters.
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  double mu = 1.0;

  // All contacts.
  Point3 contact_in_com(0.14, 0, 0);
  PointOnLink c0{robot.link("lower0"), contact_in_com},
      c1{robot.link("lower1"), contact_in_com},
      c2{robot.link("lower2"), contact_in_com},
      c3{robot.link("lower3"), contact_in_com};

  // Contact points for each phase. First move one leg at a time then switch
  // to a more dynamic gait with two legs in swing per phase.
  using CPs = PointOnLinks;
  // Initially stationary.
  CPs p0 = {c0, c1, c2, c3}, t01 = {c1, c2, c3};
  // Front right leg swing.
  CPs p1 = {c1, c2, c3}, t12 = {c2, c3};
  // Front left leg swing.
  CPs p2 = {c0, c2, c3}, t23 = {c0, c3};
  // Hind right swing.
  CPs p3 = {c0, c1, c3}, t34 = {c0, c1};
  // Hind left swing.
  CPs p4 = {c0, c1, c2}, t45 = {c1, c2};
  // Front left leg and hind right swing.
  CPs p5 = {c1, c2}, t56 = {};
  // Front right leg and hind left swing.
  CPs p6 = {c0, c3};

  // Define contact points for each phase, transition contact points,
  // and phase durations.
  vector<CPs> phase_cps = {p0, p5, p0, p6, p0, p5, p0, p6, p0, p5, p0, p6,
                           p0, p5, p0, p6, p0, p5, p0, p6, p0, p5, p0, p6};
  vector<CPs> trans_cps = {t45, t45, t23, t23, t45, t45, t23, t23,
                           t45, t45, t23, t23, t45, t45, t23, t23,
                           t45, t45, t23, t23, t45, t45, t23};
  vector<int> phase_steps = {50, 60, 50, 60, 50, 60, 50, 60, 50, 60, 50, 60,
                             50, 60, 50, 60, 50, 60, 50, 60, 50, 60, 50, 60};

  // Define the cumulative phase steps.
  vector<int> cum_phase_steps;
  for (int i = 0; i < phase_steps.size(); i++) {
    int cum_val =
        i == 0 ? phase_steps[0] : phase_steps[i] + cum_phase_steps[i - 1];
    cum_phase_steps.push_back(cum_val);
    std::cout << cum_val << std::endl;
  }
  int t_f = cum_phase_steps[cum_phase_steps.size() - 1];  // Final timestep.
  double dt_des = 1. / 240.;  // Desired timestep duration.

  // Collocation scheme.
  auto collocation = CollocationScheme::Euler;

  // Graphs for transition between phases + their initial values.
  vector<gtsam::NonlinearFactorGraph> transition_graphs;
  vector<Values> transition_graph_init;
  double gaussian_noise = 1e-5;  // Add gaussian noise to initial values.
  for (int p = 1; p < phase_cps.size(); p++) {
    transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
        robot, cum_phase_steps[p - 1], trans_cps[p - 1], mu));
    transition_graph_init.push_back(ZeroValues(
        robot, cum_phase_steps[p - 1], gaussian_noise, trans_cps[p - 1]));
  }

  // Construct the multi-phase trajectory factor graph.
  auto graph = graph_builder.multiPhaseTrajectoryFG(
      robot, phase_steps, transition_graphs, collocation, phase_cps, mu);

  // Build the objective factors.
  gtsam::NonlinearFactorGraph objective_factors;
  auto base_link = robot.link("body");
  vector<string> links = {"lower0", "lower1", "lower2", "lower3"};
  std::map<string, gtdynamics::LinkSharedPtr> link_map;
  for (auto&& link : links)
    link_map.insert(std::make_pair(link, robot.link(link)));

  // Previous contact point goal.
  std::map<string, Point3> prev_cp;
  for (auto&& link : links)
    prev_cp.insert(std::make_pair(
        link,
        (link_map[link]->wTcom() * Pose3(Rot3(), c0.point)).translation()));

  // Distance to move contact point during swing.
  auto contact_offset = Point3(0.15, 0, 0);

  // // Add contact point objectives to factor graph.
  for (int p = 0; p < phase_cps.size(); p++) {
    // Phase start and end timesteps.
    int t_p_i = cum_phase_steps[p] - phase_steps[p];
    if (p != 0) t_p_i += 1;
    int t_p_f = cum_phase_steps[p];

    // Obtain the contact links and swing links for this phase.
    vector<string> phase_contact_links;
    for (auto&& cp : phase_cps[p]) {
      phase_contact_links.push_back(cp.link->name());
    }

    vector<string> phase_swing_links;
    for (auto&& l : links) {
      if (std::find(phase_contact_links.begin(), phase_contact_links.end(),
                    l) == phase_contact_links.end())
        phase_swing_links.push_back(l);
    }

    if (p == 2) contact_offset = 2 * contact_offset;

    // Update the goal point for the swing links.
    for (auto&& psl : phase_swing_links)
      prev_cp[psl] = prev_cp[psl] + contact_offset;

    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);
      for (auto&& pcl : phase_contact_links)
        // TODO(aescontrela): Use correct contact point for each link.
        // TODO(frank): #179 make sure height is handled correctly.
        objective_factors.add(gtdynamics::PointGoalFactor(
            internal::PoseKey(link_map[pcl]->id(), t), objectives_model_3,
            c0.point,
            Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.03)));

      double h = GROUND_HEIGHT +
                 0.05 * std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

      for (auto&& psl : phase_swing_links)
        objective_factors.add(gtdynamics::PointGoalFactor(
            internal::PoseKey(link_map[psl]->id(), t), objectives_model_3,
            c0.point, Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));
    }
  }

  // Add base goal objectives to the factor graph.
  auto base_pose_model = gtsam::noiseModel::Diagonal::Sigmas(
      (Vector(6) << sigma_objectives * 3, sigma_objectives * 3,
       sigma_objectives * 3, 10000, sigma_objectives, sigma_objectives)
          .finished());
  for (int t = 0; t <= t_f; t++)
    objective_factors.addPrior(
        internal::PoseKey(base_link->id(), t),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0.0, 0.13)),
        base_pose_model);

  // Add link boundary conditions to FG.
  for (auto&& link : robot.links()) {
    // Initial link pose, twists.
    objective_factors.addPrior(internal::PoseKey(link->id(), 0), link->wTcom(),
                               dynamics_model_6);
    objective_factors.addPrior<Vector6>(internal::TwistKey(link->id(), 0),
                                        Vector6::Zero(), dynamics_model_6);

    // Final link twists, accelerations.
    objective_factors.addPrior<Vector6>(internal::TwistKey(link->id(), t_f),
                                        Vector6::Zero(), objectives_model_6);
    objective_factors.addPrior<Vector6>(
        internal::TwistAccelKey(link->id(), t_f), Vector6::Zero(),
        objectives_model_6);
  }

  // Add joint boundary conditions to FG.
  for (auto&& joint : robot.joints()) {
    objective_factors.addPrior(internal::JointAngleKey(joint->id(), 0), 0.0,
                               dynamics_model_1);
    objective_factors.addPrior(internal::JointVelKey(joint->id(), 0), 0.0,
                               dynamics_model_1);

    objective_factors.addPrior(internal::JointVelKey(joint->id(), t_f), 0.0,
                               objectives_model_1);
    objective_factors.addPrior(internal::JointAccelKey(joint->id(), t_f), 0.0,
                               objectives_model_1);
  }

  // Add prior factor constraining all Phase keys to have duration of 1 / 240.
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    objective_factors.addPrior(PhaseKey(phase), dt_des,
                               gtsam::noiseModel::Isotropic::Sigma(1, 1e-30));
  }

  // Add min torque objectives.
  for (int t = 0; t <= t_f; t++) {
    for (auto&& joint : robot.joints()) {
      objective_factors.add(gtdynamics::MinTorqueFactor(
          internal::TorqueKey(joint->id(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
    }
  }
  graph.add(objective_factors);

  // Initialize solution.
  gtsam::Values init_vals;
  init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
      robot, phase_steps, transition_graph_init, dt_des, gaussian_noise,
      phase_cps);

  // Optimize!
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setlambdaInitial(1e0);
  params.setlambdaLowerBound(1e-7);
  params.setlambdaUpperBound(1e10);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  gtsam::Values results = optimizer.optimize();

  // Log the joint angles, velocities, accels, torques, and current goal pose.
  vector<string> joint_names;
  for (auto&& joint : robot.joints()) {
    joint_names.push_back(joint->name());
  }
  string joint_names_str = boost::algorithm::join(joint_names, ",");
  std::ofstream traj_file;
  traj_file.open("traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << joint_names_str << "," << joint_names_str << ","
            << joint_names_str << "," << joint_names_str << ",t"
            << "\n";
  int t = 0;
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
      vector<string> vals;
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(
            results.atDouble(internal::JointAngleKey(joint->id(), t))));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(
            results.atDouble(internal::JointVelKey(joint->id(), t))));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(
            results.atDouble(internal::JointAccelKey(joint->id(), t))));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(
            results.atDouble(internal::TorqueKey(joint->id(), t))));

      vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));

      t++;
      string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
  traj_file.close();

  return 0;
}
