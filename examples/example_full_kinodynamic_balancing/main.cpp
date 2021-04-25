/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

using namespace gtdynamics;

int main(int argc, char** argv) {
  // Load the Vision 60 quadruped by Ghost robotics:
  // https://youtu.be/wrBNJKZKg10
  auto vision60 =
      CreateRobotFromFile(kUrdfPath + std::string("/vision60.urdf"));

  // Env parameters.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  double mu = 2.0;

  // Contact points at feet.
  ContactPoints contact_points;
  contact_points.emplace(
      "lower0", ContactPoint{gtsam::Point3(0.14, 0, 0), 0});
  contact_points.emplace(
      "lower1", ContactPoint{gtsam::Point3(0.14, 0, 0), 1});
  contact_points.emplace(
      "lower2", ContactPoint{gtsam::Point3(0.14, 0, 0), 2});
  contact_points.emplace(
      "lower3", ContactPoint{gtsam::Point3(0.14, 0, 0), 3});

  // Specify optimal control problem parameters.
  double T = 3.0;                                     // Time horizon (s.)
  double dt = 1. / 240;                               // Time step (s.)
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  double sigma_dynamics = 1e-5;    // Variance of dynamics constraints.
  double sigma_objectives = 1e-3;  // Variance of additional objectives.

  // Specify boundary conditions for base and joints.
  gtsam::Pose3 base_pose_init = vision60.link("body")->wTcom();
  gtsam::Vector6 base_twist_init = gtsam::Z_6x1,
                 base_twist_final = gtsam::Z_6x1,
                 base_accel_init = gtsam::Z_6x1,
                 base_accel_final = gtsam::Z_6x1;
  gtsam::Vector joint_angles_init = gtsam::Vector::Zero(12),
                joint_vels_init = gtsam::Vector::Zero(12),
                joint_accels_init = gtsam::Vector::Zero(12),
                joint_vels_final = gtsam::Vector::Zero(12),
                joint_accels_final = gtsam::Vector::Zero(12);

  // Specify target poses we want to reach and at what times
  // we want to reach them.
  std::vector<gtsam::Pose3> des_poses;
  std::vector<double> des_poses_t;
  auto des_pose_nm = gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives);

  des_poses.push_back(gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 8, 0.0, 0.0),
                                   gtsam::Point3(0, 0, 0.1)));
  des_poses_t.push_back(0.75);

  des_poses.push_back(gtsam::Pose3(gtsam::Rot3::RzRyRx(-M_PI / 8, 0.0, 0.0),
                                   gtsam::Point3(0, 0, 0.1)));
  des_poses_t.push_back(1.5);

  des_poses.push_back(
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, M_PI / 10, M_PI / 8),
                   gtsam::Point3(0.0, 0, 0.1)));
  des_poses_t.push_back(2.25);

  des_poses.push_back(
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, -M_PI / 10, -M_PI / 8),
                   gtsam::Point3(0.0, 0, 0.1)));
  des_poses_t.push_back(3.0);

  // Build the trajectory factor graph and add boundary condition and goal
  // pose factors.
  auto opt = OptimizerSetting();
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
  auto graph_builder = DynamicsGraph(opt, gravity);
  gtsam::NonlinearFactorGraph graph = graph_builder.trajectoryFG(
      vision60, t_steps, dt, CollocationScheme::Trapezoidal, contact_points,
      mu);

  auto base_link = vision60.link("body");
  gtsam::NonlinearFactorGraph objective_factors;

  // Add certain poses to be reached.
  for (size_t i = 0; i < des_poses.size(); i++)
    objective_factors.addPrior(
        internal::PoseKey(base_link->id(),
                          static_cast<int>(std::ceil(des_poses_t[i] / dt))),
        des_poses[i], des_pose_nm);

  // Add base boundary conditions to FG.
  objective_factors.addPrior(
      internal::PoseKey(base_link->id(), 0), base_pose_init,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics));
  objective_factors.addPrior<gtsam::Vector6>(
      internal::TwistKey(base_link->id(), 0), base_twist_init,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics));
  objective_factors.addPrior<gtsam::Vector6>(
      internal::TwistAccelKey(base_link->id(), 0), base_accel_init,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics));
  objective_factors.addPrior<gtsam::Vector6>(
      internal::TwistKey(base_link->id(), t_steps), base_twist_final,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives));
  objective_factors.addPrior<gtsam::Vector6>(
      internal::TwistAccelKey(base_link->id(), t_steps), base_accel_final,
      gtsam::noiseModel::Isotropic::Sigma(6, sigma_objectives));

  // Add joint boundary conditions to FG.
  for (auto&& joint : vision60.joints()) {
    objective_factors.addPrior(
        internal::JointAngleKey(joint->id(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics));
    objective_factors.addPrior(
        internal::JointVelKey(joint->id(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics));
    objective_factors.addPrior(
        internal::JointAccelKey(joint->id(), 0), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics));
    objective_factors.addPrior(
        internal::JointVelKey(joint->id(), t_steps), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_objectives));
    objective_factors.addPrior(
        internal::JointAccelKey(joint->id(), t_steps), 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_objectives));
  }

  // Add min torque objectives.
  for (int t = 0; t <= t_steps; t++) {
    for (auto&& joint : vision60.joints())
      objective_factors.add(MinTorqueFactor(
          internal::TorqueKey(joint->id(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }
  graph.add(objective_factors);

  // Initialize solution.
  gtsam::Values init_vals;
  std::string initialization_technique = "inverse_kinematics";
  if (initialization_technique == "interp")
    // TODO(aescontrela): Figure out why the linearly interpolated initial
    // trajectory fails to optimize. My initial guess is that the optimizer has
    // a difficult time optimizing the trajectory when the initial solution lies
    // in the infeasible region. This would make sense if I were using an IPM to
    // solve this problem...
    init_vals = InitializeSolutionInterpolationMultiPhase(
        vision60, "body", base_pose_init, des_poses, des_poses_t, dt, 0.0,
        contact_points);
  else if (initialization_technique == "zeros")
    init_vals = ZeroValuesTrajectory(vision60, t_steps, 0, 0.0, contact_points);
  else if (initialization_technique == "inverse_kinematics")
    init_vals = InitializeSolutionInverseKinematics(
        vision60, "body", base_pose_init, des_poses, des_poses_t, dt, 0.0,
        contact_points);

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  gtsam::Values results = optimizer.optimize();

  gtsam::Pose3 optimized_pose_init = Pose(results, base_link->id(), 0);
  gtsam::Pose3 optimized_pose_final =
      Pose(results, base_link->id(), t_steps - 1);

  std::cout << "Optimized Pose init trans: "
            << optimized_pose_init.translation()
            << "\n\tinit rot:" << optimized_pose_init.rotation().rpy()
            << std::endl;
  std::cout << "Optimized Pose final trans: "
            << optimized_pose_final.translation()
            << "\n\tfinal rot:" << optimized_pose_final.rotation().rpy()
            << std::endl;

  auto joint_vals_init = graph_builder.jointAnglesMap(vision60, results, 0);
  auto joint_vals_final =
      graph_builder.jointAnglesMap(vision60, results, t_steps);

  std::cout << "Joint vals init" << std::endl;
  for (auto&& jval : joint_vals_init)
    std::cout << "\t" << jval.first << ": " << jval.second << "," << std::endl;
  std::cout << "Joint vals final" << std::endl;
  for (auto&& jval : joint_vals_final)
    std::cout << "\t" << jval.first << ": " << jval.second << "," << std::endl;

  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::vector<std::string> jnames;
  for (auto&& joint : vision60.joints()) jnames.push_back(joint->name());
  std::string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;
  traj_file.open("traj.csv");
  // angles, vels, accels, torques.
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << ","
            << jnames_str << ",gol_x"
            << ",gol_y"
            << ",gol_z"
            << ",gol_qx"
            << ",gol_qy"
            << ",gol_qz"
            << ",gol_qw"
            << "\n";
  for (int t = 0; t <= t_steps; t++) {
    std::vector<std::string> vals;
    for (auto&& joint : vision60.joints())
      vals.push_back(std::to_string(JointAngle(results, joint->id(), t)));
    for (auto&& joint : vision60.joints())
      vals.push_back(std::to_string(JointVel(results, joint->id(), t)));
    for (auto&& joint : vision60.joints())
      vals.push_back(std::to_string(JointAccel(results, joint->id(), t)));
    for (auto&& joint : vision60.joints())
      vals.push_back(std::to_string(Torque(results, joint->id(), t)));

    for (size_t i = 0; i < des_poses.size(); i++) {
      gtsam::Pose3 dp = des_poses[i];
      if (t <= static_cast<int>(std::round(des_poses_t[i] / dt))) {
        vals.push_back(std::to_string(dp.x()));
        vals.push_back(std::to_string(dp.y()));
        vals.push_back(std::to_string(dp.z()));
        vals.push_back(std::to_string(dp.rotation().toQuaternion().x()));
        vals.push_back(std::to_string(dp.rotation().toQuaternion().y()));
        vals.push_back(std::to_string(dp.rotation().toQuaternion().z()));
        vals.push_back(std::to_string(dp.rotation().toQuaternion().w()));
        break;
      }
    }

    std::string vals_str = boost::algorithm::join(vals, ",");
    traj_file << vals_str << "\n";
  }
  traj_file.close();

  return 0;
}
