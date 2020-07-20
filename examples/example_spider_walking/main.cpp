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
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
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
#include <algorithm>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

#define GROUND_HEIGHT -0.04

using gtdynamics::PoseKey, gtsam::Vector6, gtsam::Vector3, gtsam::Vector,
    gtdynamics::JointAngleKey, gtdynamics::JointVelKey, gtsam::Point3,
    gtsam::Rot3, gtsam::Pose3, gtsam::Values,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtdynamics::ContactPoints,
    gtdynamics::ContactPoint, gtdynamics::ZeroValues, gtdynamics::PhaseKey,
    gtdynamics::TwistKey, gtdynamics::TwistAccelKey, gtdynamics::Robot,
    std::vector, std::string, gtsam::noiseModel::Isotropic;

int main(int argc, char** argv) {

  // Load Stephanie's spider robot.
  auto spider = gtdynamics::CreateRobotFromFile("../spider.sdf", "spider");
  spider.printRobot();

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
  typedef ContactPoint CP;
  auto c1 = CP{"tarsus_1", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front left.
  auto c2 = CP{"tarsus_2", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind left.
  auto c3 = CP{"tarsus_3", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front right.
  auto c4 = CP{"tarsus_4", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind right.
  auto c5 = CP{"tarsus_5", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front left.
  auto c6 = CP{"tarsus_6", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind left.
  auto c7 = CP{"tarsus_7", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Front right.
  auto c8 = CP{"tarsus_8", Point3(0, 1.741, 0), 0, GROUND_HEIGHT}; // Hind right.

  // Contact points for each phase. This gait moves one leg at a time.
  typedef ContactPoints CPs;
  CPs p0  = {c1, c2, c3, c4, c5, c6, c7, c8};  // Initially stationary.
  CPs t01 = {    c2, c3, c4, c5, c6, c7, c8};
  CPs p1  = {    c2, c3, c4, c5, c6, c7, c8};
  CPs t12 = {        c3, c4, c5, c6, c7, c8};
  CPs p2  = {c1,     c3, c4, c5, c6, c7, c8};
  CPs t23 = {c1,         c4, c5, c6, c7, c8};
  CPs p3  = {c1, c2,     c4, c5, c6, c7, c8};
  CPs t34 = {c1, c2,         c5, c6, c7, c8};
  CPs p4  = {c1, c2, c3,     c5, c6, c7, c8};
  CPs t45 = {c1, c2, c3,         c6, c7, c8};
  CPs p5  = {c1, c2, c3, c4,     c6, c7, c8};
  CPs t56 = {c1, c2, c3, c4,         c7, c8};
  CPs p6  = {c1, c2, c3, c4, c5,     c7, c8};
  CPs t67 = {c1, c2, c3, c4, c5,         c8};
  CPs p7  = {c1, c2, c3, c4, c5, c6,     c8};
  CPs t78 = {c1, c2, c3, c4, c5, c6,       };
  CPs p8  = {c1, c2, c3, c4, c5, c6, c7    };
  CPs t89 = {c1, c2, c3, c4, c5, c6, c7    };
  CPs p9  = {c1, c2, c3, c4, c5, c6, c7, c8};  // End stationary.

  // Define contact points for each phase, transition contact points,
  // and phase durations.
  vector<CPs> phase_cps =   {p0, p1, p2, p3, p4, p5, p6, p7, p8, p9};
  vector<CPs> trans_cps =   {t01, t12, t23, t34, t45, t56, t67, t78, t89};
  vector<int> phase_steps = {50, 60, 50, 60, 50, 60, 50, 60, 50, 60};

  // Define the cumulative phase steps.
  vector<int> cum_phase_steps;
  for (int i = 0; i < phase_steps.size(); i++) {
    int cum_val = i == 0 ? phase_steps[0] : phase_steps[i] + cum_phase_steps[i-1];
    cum_phase_steps.push_back(cum_val);
    std::cout << cum_val << std::endl;
  }
  int t_f = cum_phase_steps[cum_phase_steps.size() - 1];  // Final timestep.
  double dt_des = 1. / 240.;  // Desired timestep duration.
    
  // Robot model for each phase.
  vector<Robot> robots(phase_cps.size(), spider);

  // Collocation scheme.
  auto collocation = gtdynamics::DynamicsGraph::CollocationScheme::Euler;

  // Graphs for transition between phases + their initial values.
  vector<gtsam::NonlinearFactorGraph> transition_graphs;
  vector<Values> transition_graph_init;
  double gaussian_noise = 1e-5;  // Add gaussian noise to initial values.
  for (int p = 1; p < phase_cps.size(); p++) {
    std::cout << "Creating transition graph" << std::endl;
    transition_graphs.push_back(graph_builder.dynamicsFactorGraph(
      robots[p], cum_phase_steps[p - 1], gravity, boost::none, trans_cps[p - 1], mu));
    std::cout << "Creating initial values" << std::endl;
    transition_graph_init.push_back(
      ZeroValues(robots[p], cum_phase_steps[p - 1], gaussian_noise, trans_cps[p - 1]));
  }

  // Construct the multi-phase trajectory factor graph.
  std::cout << "Creating dynamics graph" << std::endl;
  auto graph = graph_builder.multiPhaseTrajectoryFG(
    robots, phase_steps, transition_graphs, collocation, gravity, boost::none,
    phase_cps, mu);

  // Build the objective factors.
  gtsam::NonlinearFactorGraph objective_factors;
  auto base_link = spider.getLinkByName("body");
  vector<string> links = {"tarsus_1", "tarsus_2", "tarsus_3", "tarsus_4", "tarsus_5", "tarsus_6", "tarsus_7", "tarsus_8"};
  std::map<string, gtdynamics::LinkSharedPtr> link_map;
  for (auto&& link : links)
    link_map.insert(std::make_pair(link, spider.getLinkByName(link)));

  // Previous contact point goal.
  std::map<string, Point3> prev_cp;
  for (auto&& link : links)
    prev_cp.insert(std::make_pair(link,
      (link_map[link]->wTcom() * Pose3(Rot3(), c1.contact_point)).translation()));
  
  // Distance to move contact point during swing.
  auto contact_offset = Point3(0.15, 0, 0);
  
  // Add contact point objectives to factor graph.
  for (int p = 0; p < phase_cps.size(); p++) {
    // Phase start and end timesteps.
    int t_p_i = cum_phase_steps[p] - phase_steps[p];
    if (p != 0) t_p_i += 1;
    int t_p_f = cum_phase_steps[p];

    // Obtain the contact links and swing links for this phase.
    vector<string> phase_contact_links;
    for (auto&& cp : phase_cps[p])
      phase_contact_links.push_back(cp.name);
    vector<string> phase_swing_links;
    for (auto&& l : links) {
      if (std::find(phase_contact_links.begin(),
            phase_contact_links.end(), l) == phase_contact_links.end())
        phase_swing_links.push_back(l);
    }

    if (p==2)
      contact_offset = 2 * contact_offset;

    // Update the goal point for the swing links.
    for (auto && psl : phase_swing_links)
      prev_cp[psl] = prev_cp[psl] + contact_offset;

    for (int t = t_p_i; t <= t_p_f; t++) {
      // Normalized phase progress.
      double t_normed = (double) (t - t_p_i) / (double) (t_p_f - t_p_i);
      for (auto&& pcl : phase_contact_links)
        // TODO(aescontrela): Use correct contact point for each link.
        objective_factors.add(gtdynamics::PointGoalFactor(
          PoseKey(link_map[pcl]->getID(), t), objectives_model_3,
          Pose3(Rot3(), c1.contact_point), Point3(prev_cp[pcl].x(), prev_cp[pcl].y(), GROUND_HEIGHT - 0.03)));

      double h = GROUND_HEIGHT + 0.05 * std::pow(t_normed, 1.1) * std::pow(1 - t_normed, 0.7);

      for (auto&& psl : phase_swing_links)
        objective_factors.add(gtdynamics::PointGoalFactor(
          PoseKey(link_map[psl]->getID(), t), objectives_model_3,
          Pose3(Rot3(), c1.contact_point), Point3(prev_cp[psl].x(), prev_cp[psl].y(), h)));
    }
  }

  // Add base goal objectives to the factor graph.
  auto base_pose_model = gtsam::noiseModel::Diagonal::Sigmas(
    (Vector(6) << sigma_objectives * 3, sigma_objectives * 3,
                  sigma_objectives * 3, 10000, sigma_objectives,
                  sigma_objectives).finished());
  for (int t = 0; t <= t_f; t++)
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      PoseKey(base_link->getID(), t),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.13)),
      base_pose_model));

  // Add link boundary conditions to FG.
  for (auto&& link : spider.links()) {
    // Initial link pose, twists.
    objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        PoseKey(link->getID(), 0), link->wTcom(), dynamics_model_6));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->getID(), 0), Vector6::Zero(), dynamics_model_6));

    // Final link twists, accelerations.
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistKey(link->getID(), t_f), Vector6::Zero(), objectives_model_6));
    objective_factors.add(gtsam::PriorFactor<Vector6>(
        TwistAccelKey(link->getID(), t_f), Vector6::Zero(),
        objectives_model_6));
  }

  // Add joint boundary conditions to FG.
  for (auto&& joint : spider.joints()) {
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAngleKey(joint->getID(), 0), 0.0, dynamics_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->getID(), 0), 0.0, dynamics_model_1));

    objective_factors.add(gtsam::PriorFactor<double>(
        JointVelKey(joint->getID(), t_f), 0.0, objectives_model_1));
    objective_factors.add(gtsam::PriorFactor<double>(
        JointAccelKey(joint->getID(), t_f), 0.0, objectives_model_1));
  }

  // Add prior factor constraining all Phase keys to have duration of 1 / 240.
  for (int phase = 0; phase < phase_steps.size(); phase++)
    objective_factors.add(gtsam::PriorFactor<double>(
        PhaseKey(phase), dt_des,
        gtsam::noiseModel::Isotropic::Sigma(1, 1e-30)));

  // Add min torque objectives.
  for (int t = 0; t <= t_f; t++) {
    for (auto&& joint : spider.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          TorqueKey(joint->getID(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }
  graph.add(objective_factors);

   // Initialize solution.
  gtsam::Values init_vals;
  init_vals = gtdynamics::MultiPhaseZeroValuesTrajectory(
    robots, phase_steps, transition_graph_init, dt_des, gaussian_noise,
    phase_cps);

  std::ifstream ifile;
  ifile.open("../optimization_result.txt");
  if(!ifile) {
    // The file containing optimization results has not been generated yet,
    // so optimize and save the results to a text file.

    // Optimize!
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.setlambdaInitial(1e0);
    params.setlambdaLowerBound(1e-7);
    params.setlambdaUpperBound(1e10);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
    gtsam::Values results = optimizer.optimize();

    // Write result of optimization to file
    std::ofstream myfile ("../optimization_result.txt");
    if (myfile.is_open())
    {
      int t = 0;
      for (int phase = 0; phase < phase_steps.size(); phase++) {
        for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
          vector<string> vals;
          for (auto&& joint : spider.joints())
            myfile << std::to_string(results.atDouble(JointAngleKey(joint->getID(), t))) + "\n";
          for (auto&& joint : spider.joints())
            myfile << std::to_string(results.atDouble(JointVelKey(joint->getID(), t))) + "\n";
          for (auto&& joint : spider.joints())
            myfile << std::to_string(results.atDouble(JointAccelKey(joint->getID(), t))) + "\n";
          for (auto&& joint : spider.joints())
            myfile << std::to_string(results.atDouble(TorqueKey(joint->getID(), t))) + "\n";

          myfile << std::to_string(results.atDouble(PhaseKey(phase))) + "\n";

          t++;
        }
      }
      myfile.close();
    }
    else std::cout << "Unable to open file" << std::endl;
  }

  // Using variables populated from reading the file optimization_result.txt,
  // log the joint angles, velocities, accels, torques, and current goal pose.
  vector<string> jnames;
  for (auto&& joint : spider.joints()) jnames.push_back(joint->name());
  string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;
  traj_file.open("../traj.csv");
  // angles, vels, accels, torques, time.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",t"
            << "\n";
  std::ifstream myfile ("../optimization_result.txt");
  int t = 0;
  for (int phase = 0; phase < phase_steps.size(); phase++) {
    for (int phase_step = 0; phase_step < phase_steps[phase]; phase_step++) {
      
      //Read from optimization_result.txt
      string joint_angle_key, joint_vel_key, joint_accel_key, torque_key, phase_key;
      getline(myfile, joint_angle_key);
      getline(myfile, joint_vel_key);
      getline(myfile, joint_accel_key);
      getline(myfile, torque_key);
      getline(myfile, phase_key);
      vector<string> vals;
      for (auto&& joint : spider.joints())
        vals.push_back(joint_angle_key);
      for (auto&& joint : spider.joints())
        vals.push_back(joint_vel_key);
      for (auto&& joint : spider.joints())
        vals.push_back(joint_accel_key);
      for (auto&& joint : spider.joints())
        vals.push_back(torque_key);
      vals.push_back(phase_key);
      t++;
      string vals_str = boost::algorithm::join(vals, ",");
      traj_file << vals_str << "\n";
    }
  }
  traj_file.close();
  myfile.close();

  return 0;
}
