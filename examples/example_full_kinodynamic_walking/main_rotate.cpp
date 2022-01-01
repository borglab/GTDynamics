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
#include <cmath>
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
      gtdynamics::CreateRobotFromFile(kUrdfPath + string("vision60.urdf"));

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
  Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  // All contacts.
  Point3 contact_in_com(0.14, 0, 0);
  PointOnLink c0{robot.link("lower0"), contact_in_com},  // Front left.
      c1{robot.link("lower1"), contact_in_com},          // Hind left.
      c2{robot.link("lower2"), contact_in_com},          // Front right.
      c3{robot.link("lower3"), contact_in_com};          // Hind right.

  // Contact points for each phase. First move one leg at a time then switch
  // to a more dynamic gait with two legs in swing per phase. Below pi are the
  // contacts for phase i and tij=contacts for transition from phase i to j.
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
  vector<CPs> phase_cps = {p0, p1, p0, p2, p0, p3, p0, p4,
                           p0, p1, p0, p2, p0, p3, p0, p4};
  vector<CPs> trans_cps = {p1, p1, p2, p2, p3, p3, p4, p4,
                           p1, p1, p2, p2, p3, p3, p4};
  vector<int> phase_steps = {50, 60, 50, 60, 50, 60, 50, 60,
                             50, 60, 50, 60, 50, 60, 50, 60};

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

  // Robot model for each phase.
  vector<Robot> robots(phase_cps.size(), robot);

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

  // Compute the centroid of the contacts.
  auto centroid = Point3(0, 0, 0);
  for (auto&& link : links) centroid += link_map[link]->bMcom() * c0.point;
  centroid = centroid / 4;

  std::map<string, double> centroid_contact_dist;
  std::map<string, double> prev_theta;
  for (auto&& link : links) {
    auto cp = link_map[link]->bMcom() * c0.point;
    auto delta = cp - centroid;
    centroid_contact_dist.insert(std::make_pair(link, delta.norm()));
    prev_theta.insert(std::make_pair(link, std::atan2(delta.y(), delta.x())));
  }

  // Distance to move contact point during swing.
  double theta_inc = M_PI / 6;

  // Add contact point objectives to factor graph.
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

    // Update the goal point for the swing links.
    for (auto&& psl : phase_swing_links)
      prev_theta[psl] = prev_theta[psl] + theta_inc;

    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);
      for (auto&& pcl : phase_contact_links) {
        // TODO(aescontrela): Use correct contact point for each link.
        auto new_cp = Point3(centroid.x() + centroid_contact_dist[pcl] *
                                                std::cos(prev_theta[pcl]),
                             centroid.y() + centroid_contact_dist[pcl] *
                                                std::sin(prev_theta[pcl]),
                             GROUND_HEIGHT - 0.03);

        objective_factors.add(
            gtdynamics::PointGoalFactor(PoseKey(link_map[pcl]->id(), t),
                                        objectives_model_3, c0.point, new_cp));
      }

      double h = GROUND_HEIGHT +
                 0.05 * std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

      for (auto&& psl : phase_swing_links) {
        auto new_cp = Point3(centroid.x() + centroid_contact_dist[psl] *
                                                std::cos(prev_theta[psl]),
                             centroid.y() + centroid_contact_dist[psl] *
                                                std::sin(prev_theta[psl]),
                             h);

        objective_factors.add(
            gtdynamics::PointGoalFactor(PoseKey(link_map[psl]->id(), t),
                                        objectives_model_3, c0.point, new_cp));
      }
    }
  }

  // Add base goal objectives to the factor graph.
  auto base_pose_goal =
      Pose3(Rot3::Rz(theta_inc * phase_cps.size() / 8.), Point3(0, 0, 0.13));
  auto base_pose_model = gtsam::noiseModel::Diagonal::Sigmas(
      (Vector(6) << sigma_objectives * 3, sigma_objectives * 3,
       sigma_objectives * 100, 10000, sigma_objectives, sigma_objectives)
          .finished());
  for (int t = 0; t <= t_f; t++)
    objective_factors.addPrior(PoseKey(base_link->id(), t), base_pose_goal,
                               base_pose_model);

  // Add link boundary conditions to FG.
  for (auto&& link : robot.links()) {
    // Initial link pose, twists.
    objective_factors.addPrior(PoseKey(link->id(), 0), link->bMcom(),
                               dynamics_model_6);
    objective_factors.addPrior<Vector6>(TwistKey(link->id(), 0),
                                        Vector6::Zero(), dynamics_model_6);

    // Final link twists, accelerations.
    objective_factors.addPrior<Vector6>(TwistKey(link->id(), t_f),
                                        Vector6::Zero(), objectives_model_6);
    objective_factors.addPrior<Vector6>(TwistAccelKey(link->id(), t_f),
                                        Vector6::Zero(), objectives_model_6);
  }

  // Add joint boundary conditions to FG.
  for (auto&& joint : robot.joints()) {
    objective_factors.addPrior(JointAngleKey(joint->id(), 0), 0.0,
                               dynamics_model_1);
    objective_factors.addPrior(JointVelKey(joint->id(), 0), 0.0,
                               dynamics_model_1);

    objective_factors.addPrior(JointVelKey(joint->id(), t_f), 0.0,
                               objectives_model_1);
    objective_factors.addPrior(JointAccelKey(joint->id(), t_f), 0.0,
                               objectives_model_1);
  }

  // Add prior factor constraining all Phase keys to have duration of 1 / 240.
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    objective_factors.addPrior(PhaseKey(phase), dt_des,
                               gtsam::noiseModel::Isotropic::Sigma(1, 1e-30));
  }

  // Add min torque objectives.
  for (int t = 0; t <= t_f; t++) {
    for (auto&& joint : robot.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          TorqueKey(joint->id(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
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
  vector<string> jnames;
  for (auto&& joint : robot.joints()) jnames.push_back(joint->name());
  string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;
  traj_file.open("traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",t"
            << "\n";
  int t = 0;
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
      vector<string> vals;
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(JointAngle(results, joint->id(), t)));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(JointVel(results, joint->id(), t)));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(JointAccel(results, joint->id(), t)));
      for (auto&& joint : robot.joints())
        vals.push_back(std::to_string(Torque(results, joint->id(), t)));

      vals.push_back(std::to_string(results.atDouble(PhaseKey(phase))));

      t++;
      string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
  traj_file.close();

  return 0;
}
