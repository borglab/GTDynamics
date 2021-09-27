/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Spider trajectory optimization with pre-specified footholds.
 * @Author: Alejandro Escontrela
 * @Author: Stephanie McCormick
 * @Author: Disha Das
 * @Author: Tarushree Gandhi
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <utility>

#define GROUND_HEIGHT -1.75

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
  // Load Stephanie's spider robot.
  auto robot = gtdynamics::CreateRobotFromFile(kSdfPath + string("spider.sdf"),
                                               "spider");

  double sigma_dynamics = 1e-5;    // std of dynamics constraints.
  double sigma_objectives = 1e-6;  // std of additional objectives.
  double sigma_joints = 1.85e-4;   // 1.85e-4

  // Noise models.
  auto dynamics_model_6 = Isotropic::Sigma(6, sigma_dynamics),
       dynamics_model_3 = Isotropic::Sigma(3, sigma_dynamics),
       dynamics_model_1 = Isotropic::Sigma(1, sigma_dynamics),
       dynamics_model_1_2 = Isotropic::Sigma(1, sigma_joints),
       objectives_model_6 = Isotropic::Sigma(6, sigma_objectives),
       objectives_model_3 = Isotropic::Sigma(3, sigma_objectives),
       objectives_model_1 = Isotropic::Sigma(1, sigma_objectives);

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

  // Env parameters.
  Vector3 gravity(0, 0, -9.8);
  double mu = 1.0;

  auto graph_builder = gtdynamics::DynamicsGraph(opt, gravity);

  vector<string> links = {"tarsus_1_L1", "tarsus_2_L2", "tarsus_3_L3",
                          "tarsus_4_L4", "tarsus_5_R4", "tarsus_6_R3",
                          "tarsus_7_R2", "tarsus_8_R1"};
  // All contacts.
  const Point3 contact_in_com(0, 0.19, 0);
  PointOnLink cp1(robot.link("tarsus_1_L1"), contact_in_com);  // Front left.
  PointOnLink cp2(robot.link("tarsus_2_L2"), contact_in_com);  // Hind left.
  PointOnLink cp3(robot.link("tarsus_3_L3"), contact_in_com);  // Front right.
  PointOnLink cp4(robot.link("tarsus_4_L4"), contact_in_com);  // Hind right.
  PointOnLink cp5(robot.link("tarsus_5_R4"), contact_in_com);  // Front left.
  PointOnLink cp6(robot.link("tarsus_6_R3"), contact_in_com);  // Hind left.
  PointOnLink cp7(robot.link("tarsus_7_R2"), contact_in_com);  // Front right.
  PointOnLink cp8(robot.link("tarsus_8_R1"), contact_in_com);  // Hind right.

  // Contact points for each phase.
  // This gait moves one leg at a time.
  using CPs = PointOnLinks;
  CPs t00 = {cp1, cp2, cp3, cp4, cp5, cp6, cp7, cp8};
  // Initially stationary.
  CPs p0 = {cp1, cp2, cp3, cp4, cp5, cp6, cp7, cp8};
  CPs t01 = {cp2, cp3, cp4, cp5, cp6, cp7, cp8};
  CPs p1 = {cp2, cp3, cp4, cp5, cp6, cp7, cp8};
  CPs t12 = {cp3, cp4, cp5, cp6, cp7, cp8};
  CPs p2 = {cp1, cp3, cp4, cp5, cp6, cp7, cp8};
  CPs t23 = {cp1, cp4, cp5, cp6, cp7, cp8};
  CPs p3 = {cp1, cp2, cp4, cp5, cp6, cp7, cp8};
  CPs t34 = {cp1, cp2, cp5, cp6, cp7, cp8};
  CPs p4 = {cp1, cp2, cp3, cp5, cp6, cp7, cp8};
  CPs t45 = {cp1, cp2, cp3, cp6, cp7, cp8};
  CPs p5 = {cp1, cp2, cp3, cp4, cp6, cp7, cp8};
  CPs t56 = {cp1, cp2, cp3, cp4, cp7, cp8};
  CPs p6 = {cp1, cp2, cp3, cp4, cp5, cp7, cp8};
  CPs t67 = {cp1, cp2, cp3, cp4, cp5, cp8};
  CPs p7 = {cp1, cp2, cp3, cp4, cp5, cp6, cp8};
  CPs t78 = {cp1, cp2, cp3, cp4, cp5, cp6};
  CPs p8 = {cp1, cp2, cp3, cp4, cp5, cp6, cp7};
  CPs t80 = {cp2, cp3, cp4, cp5, cp6, cp7};

  // This gait moves four legs at a time (alternating tetrapod).
  CPs t0a = {cp2, cp4, cp6, cp8};
  CPs pa = {cp2, cp4, cp6, cp8};
  CPs tab = {};
  CPs pb = {cp1, cp3, cp5, cp7};
  CPs tb0 = {cp1, cp3, cp5, cp7};

  // Define contact points for each phase, transition contact points,
  // and phase durations.
  // Alternating Tetrapod:
  vector<CPs> phase_cps = {p0, pa, p0, pb, p0, pa, p0, pb, p0, pa,
                           p0, pb, p0, pa, p0, pb, p0, pa, p0, pb};
  vector<CPs> trans_cps = {t0a, t0a, tb0, tb0, t0a, t0a, tb0, tb0, t0a, t0a,
                           tb0, tb0, t0a, t0a, tb0, tb0, t0a, t0a, tb0};
  vector<int> phase_steps = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
                             20, 20, 20, 20, 20, 20, 20, 20, 20, 20};

  // Define noise to be added to initial values, desired timestep duration,
  // vector of link name strings, robot model for each phase, and
  // phase transition initial values.
  double gaussian_noise = 1e-5;

  double dt_des = 1. / 240;
  vector<Values> transition_graph_init;

  // Define the cumulative phase steps.
  vector<int> cum_phase_steps;
  for (int i = 0; i < phase_steps.size(); i++) {
    int cum_val =
        i == 0 ? phase_steps[0] : phase_steps[i] + cum_phase_steps[i - 1];
    cum_phase_steps.push_back(cum_val);
    std::cout << cum_val << std::endl;
  }
  int t_f = cum_phase_steps[cum_phase_steps.size() - 1];  // Final timestep.

  // Collocation scheme.
  auto collocation = gtdynamics::CollocationScheme::Euler;

  // Graphs for transition between phases + their initial values.
  vector<gtsam::NonlinearFactorGraph> transition_graphs;
  for (int p = 1; p < phase_cps.size(); p++) {
    std::cout << "Creating transition graph" << std::endl;
    transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
        robot, cum_phase_steps[p - 1], trans_cps[p - 1], mu));
    std::cout << "Creating initial values" << std::endl;
    transition_graph_init.push_back(ZeroValues(
        robot, cum_phase_steps[p - 1], gaussian_noise, trans_cps[p - 1]));
  }

  // Construct the multi-phase trajectory factor graph.
  std::cout << "Creating dynamics graph" << std::endl;
  auto graph = graph_builder.multiPhaseTrajectoryFG(
      robot, phase_steps, transition_graphs, collocation, phase_cps, mu);

  // Build the objective factors.
  gtsam::NonlinearFactorGraph objective_factors;
  auto base_link = robot.link("body");

  std::map<string, gtdynamics::LinkSharedPtr> link_map;
  for (auto&& link : links)
    link_map.insert(std::make_pair(link, robot.link(link)));

  // Previous contact point goal.
  std::map<string, Point3> prev_cp;
  for (auto&& link : links) {
    prev_cp.insert(std::make_pair(link, link_map[link]->bMcom() * cp1.point));
  }

  // Distance to move contact point per time step during swing.
  auto contact_offset = Point3(0, 0.007, 0);

  // Set this to 'right' or 'left' to make the spider rotate in place
  string turn = "right";

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
                    l) == phase_contact_links.end()) {
        phase_swing_links.push_back(l);
      }
    }

    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double)(t - t_p_i) / (double)(t_p_f - t_p_i);

      for (auto&& pcl : phase_contact_links) {
        // TODO(aescontrela): Use correct contact point for each link.
        // TODO(frank): #179 make sure height is handled correctly.
        objective_factors.add(gtdynamics::PointGoalFactor(
            internal::PoseKey(link_map[pcl]->id(), t),
            Isotropic::Sigma(3, 1e-7), cp1.point,
            Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.05)));
      }

      double h =
          GROUND_HEIGHT + std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

      for (auto&& psl : phase_swing_links) {
        objective_factors.add(gtdynamics::PointGoalFactor(
            internal::PoseKey(link_map[psl]->id(), t),
            Isotropic::Sigma(3, 1e-7), cp1.point,
            Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));
      }

      // Update the goal point for the swing links.
      for (auto&& psl : phase_swing_links) {
        if (turn.compare("right") == 0) {
          if (psl.find_first_of("1234") != string::npos)
            prev_cp[psl] = prev_cp[psl] + contact_offset;
          else
            prev_cp[psl] = prev_cp[psl] - contact_offset;
        } else {
          if (psl.find_first_of("5678") != string::npos)
            prev_cp[psl] = prev_cp[psl] + contact_offset;
          else
            prev_cp[psl] = prev_cp[psl] - contact_offset;
        }
      }
    }
  }

  // Add base goal objectives to the factor graph.
  for (int t = 0; t <= t_f; t++) {
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        internal::PoseKey(base_link->id(), t),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.0, 0.5)),
        Isotropic::Sigma(6, 6e-5)));  // 6.2e-5
  }

  // Add link boundary conditions to FG.
  for (auto&& link : robot.links()) {
    // Initial link pose, twists.
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        internal::PoseKey(link->id(), 0), link->bMcom(), dynamics_model_6));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        internal::TwistKey(link->id(), 0), Vector6::Zero(), dynamics_model_6));

    // Final link twists, accelerations.
    objective_factors.add(
        gtsam::PriorFactor<Vector6>(internal::TwistKey(link->id(), t_f),
                                    Vector6::Zero(), objectives_model_6));
    objective_factors.add(
        gtsam::PriorFactor<Vector6>(internal::TwistAccelKey(link->id(), t_f),
                                    Vector6::Zero(), objectives_model_6));
  }

  // Add joint boundary conditions to FG.
  for (auto&& joint : robot.joints()) {
    // Add priors to joint angles
    for (int t = 0; t <= t_f; t++) {
      if (joint->name().find("hip_") == 0) {
        objective_factors.add(gtsam::PriorFactor<double>(
            internal::JointAngleKey(joint->id(), t), 0, dynamics_model_1_2));
      } else if (joint->name().find("hip2") == 0) {
        objective_factors.add(gtsam::PriorFactor<double>(
            internal::JointAngleKey(joint->id(), t), 0.9, dynamics_model_1_2));
      } else if (joint->name().find("knee") == 0) {
        objective_factors.add(
            gtsam::PriorFactor<double>(internal::JointAngleKey(joint->id(), t),
                                       -1.22, dynamics_model_1_2));
      } else {
        objective_factors.add(gtsam::PriorFactor<double>(
            internal::JointAngleKey(joint->id(), t), 0.26, dynamics_model_1_2));
      }
    }

    objective_factors.add(gtsam::PriorFactor<double>(
        internal::JointVelKey(joint->id(), 0), 0.0, dynamics_model_1));

    objective_factors.add(gtsam::PriorFactor<double>(
        internal::JointVelKey(joint->id(), t_f), 0.0, objectives_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        internal::JointAccelKey(joint->id(), t_f), 0.0, objectives_model_1));
  }

  // Add prior factor constraining all Phase keys to have duration of 1 / 240.
  for (int phase = 0; phase < phase_steps.size(); phase++)
    objective_factors.add(gtsam::PriorFactor<double>(
        PhaseKey(phase), dt_des,
        gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

  // Add min torque objectives.
  for (int t = 0; t <= t_f; t++) {
    for (auto&& joint : robot.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          internal::TorqueKey(joint->id(), t),
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
  auto results = optimizer.optimize();

  vector<string> joint_names;
  for (auto&& joint : robot.joints()) joint_names.push_back(joint->name());
  string joint_names_str = boost::algorithm::join(joint_names, ",");
  std::ofstream traj_file;

  traj_file.open("rotation_traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << joint_names_str << "," << joint_names_str << ","
            << joint_names_str << "," << joint_names_str << ",t"
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

}  // namespace gtdynamics
