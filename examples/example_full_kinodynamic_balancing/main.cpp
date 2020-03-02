/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @Author: Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PoseGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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

  // Env parameters.
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();
  double mu = 1.0;

  // Contact points at feet.
  std::vector<gtdynamics::ContactPoint> contact_points;
  contact_points.push_back(gtdynamics::ContactPoint{
      "lower0", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT});
  contact_points.push_back(gtdynamics::ContactPoint{
      "lower1", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT});
  contact_points.push_back(gtdynamics::ContactPoint{
      "lower2", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT});
  contact_points.push_back(gtdynamics::ContactPoint{
      "lower3", gtsam::Point3(0.14, 0, 0), 0, GROUND_HEIGHT});

  auto graph_builder = gtdynamics::DynamicsGraph();

  gtsam::Pose3 base_pose_init = vision60.getLinkByName("body")->wTcom();
  gtsam::Vector6 base_twist_init = gtsam::Vector6::Zero(),
                 base_twist_final = gtsam::Vector6::Zero(),
                 base_accel_init = gtsam::Vector6::Zero(),
                 base_accel_final = gtsam::Vector6::Zero();

  // Specify boundary conditions for joints.
  gtsam::Vector joint_angles_init = gtsam::Vector::Zero(12),
                joint_vels_init = gtsam::Vector::Zero(12),
                joint_accels_init = gtsam::Vector::Zero(12),
                joint_vels_final = gtsam::Vector::Zero(12),
                joint_accels_final = gtsam::Vector::Zero(12);

  // Specify optimal control problem parameters.
  double T = 3.0;                                     // Time horizon (s.)
  double dt = 1. / 240;                               // Time step (s.)
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  // Specify poses which we want to hit and at what times.
  std::vector<gtsam::Pose3> des_poses;
  std::vector<gtsam::noiseModel::Base::shared_ptr> des_poses_nm;
  std::vector<double> des_poses_t;

  des_poses.push_back(gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 10, 0.0 , 0.0),
                   gtsam::Point3(0, 0, 0.1)));
  des_poses_nm.push_back(gtsam::noiseModel::Constrained::All(6));
  des_poses_t.push_back(0.9);

  des_poses.push_back(gtsam::Pose3(gtsam::Rot3::RzRyRx(-M_PI / 10, 0.0 , 0.0),
                   gtsam::Point3(0, 0, 0.1)));
  des_poses_nm.push_back(gtsam::noiseModel::Constrained::All(6));
  des_poses_t.push_back(1.9);

  des_poses.push_back(gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, M_PI / 26 , 0.0),
                   gtsam::Point3(0.0, 0, 0.1)));
  des_poses_nm.push_back(gtsam::noiseModel::Constrained::All(6));
  des_poses_t.push_back(2.8);

  gtsam::NonlinearFactorGraph graph = graph_builder.trajectoryFG(
      vision60, t_steps, dt,
      gtdynamics::DynamicsGraph::CollocationScheme::Trapezoidal, gravity,
      boost::none, contact_points, mu);

  auto base_link = vision60.getLinkByName("body");
  gtsam::NonlinearFactorGraph objective_factors;

  // Add base boundary conditions to FG.
  objective_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtdynamics::PoseKey(base_link->getID(), 0), base_pose_init,
      gtsam::noiseModel::Constrained::All(6)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistKey(base_link->getID(), 0), base_twist_init,
      gtsam::noiseModel::Constrained::All(6)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistAccelKey(base_link->getID(), 0), base_accel_init,
      gtsam::noiseModel::Constrained::All(6)));

  // Add certain poses to be reached.
  for (size_t i = 0; i < des_poses.size(); i++)
    objective_factors.add(gtdynamics::PoseGoalFactor(
      gtdynamics::PoseKey(base_link->getID(),
      static_cast<int>(std::ceil(des_poses_t[i] / dt))),
      des_poses_nm[i],
      des_poses[i]));

  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistKey(base_link->getID(), t_steps), base_twist_final,
      gtsam::noiseModel::Constrained::All(6)));
  objective_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
      gtdynamics::TwistAccelKey(base_link->getID(), t_steps), base_accel_final,
      gtsam::noiseModel::Constrained::All(6)));

  // Add joint boundary conditions to FG.
  for (auto&& joint : vision60.joints()) {
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointAngleKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Constrained::All(1)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointVelKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Constrained::All(1)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointAccelKey(joint->getID(), 0), 0.0,
        gtsam::noiseModel::Constrained::All(1)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointVelKey(joint->getID(), t_steps), 0.0,
        gtsam::noiseModel::Gaussian::Covariance(1e-3 * gtsam::I_1x1)));
    objective_factors.add(gtsam::PriorFactor<double>(
        gtdynamics::JointAccelKey(joint->getID(), t_steps), 0.0,
        gtsam::noiseModel::Gaussian::Covariance(1e-3 * gtsam::I_1x1)));
  }

  // Add min torque objective.
  for (int t = 0; t <= t_steps; t++) {
    for (auto&& joint : vision60.joints())
      objective_factors.add(gtdynamics::MinTorqueFactor(
          gtdynamics::TorqueKey(joint->getID(), t),
          gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));
  }

  graph.add(objective_factors);

  // Initial values.
  // TODO(aescontrela): Figure out why the linearly interpolated initial
  // trajectory fails to optimize. My initial guess is that the optimizer has
  // a difficult time optimizing the trajectory when the initial solution lies
  // in the infeasible region. This would make sense if I were using an IPM to
  // solve this problem...

  // gtsam::Values init_vals;
  // gtsam::Pose3 pose = base_pose_init;
  // des_poses.push_back(des_poses[des_poses.size() - 1]);
  // des_poses_t.push_back(T + dt);
  // double curr_t = 0.0;
  // for (size_t i = 0; i < des_poses.size(); i++) {
  //   init_vals.insert(initialize_solution_interpolation(vision60, "body",
  //     pose, des_poses[i], curr_t, des_poses_t[i], dt, contact_points));
  //   pose = des_poses[i];
  //   curr_t = des_poses_t[i];
  // }
  gtsam::Values init_vals =
      graph_builder.zeroValuesTrajectory(vision60, t_steps, 0, contact_points);
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-14);
  // params.setlambdaUpperBound(1e32);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  gtsam::Values results = optimizer.optimize();

  gtsam::Pose3 optimized_pose_init =
      results.at(gtdynamics::PoseKey(base_link->getID(), 0))
          .cast<gtsam::Pose3>();
  gtsam::Pose3 optimized_pose_final =
      results.at(gtdynamics::PoseKey(base_link->getID(), t_steps - 1))
          .cast<gtsam::Pose3>();

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

  // Write the joint angles, velocities, accels, and torques to an output file.
  std::vector<std::string> jnames;
  for (auto&& joint : vision60.joints()) jnames.push_back(joint->name());
  std::string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;
  traj_file.open("../traj.csv");
  // angles, vels, accels, torques.
  traj_file << jnames_str << "," << jnames_str << ","
            << jnames_str << "," << jnames_str << "\n";
  for (int t = 0; t <= t_steps; t++) {
    std::vector<std::string> jvals;
    for (auto&& joint : vision60.joints())
      jvals.push_back(std::to_string(
          results.atDouble(gtdynamics::JointAngleKey(joint->getID(), t))));
    for (auto&& joint : vision60.joints())
      jvals.push_back(std::to_string(
          results.atDouble(gtdynamics::JointVelKey(joint->getID(), t))));
    for (auto&& joint : vision60.joints())
      jvals.push_back(std::to_string(
          results.atDouble(gtdynamics::JointAccelKey(joint->getID(), t))));
    for (auto&& joint : vision60.joints())
      jvals.push_back(std::to_string(
          results.atDouble(gtdynamics::TorqueKey(joint->getID(), t))));
    std::string jvals_str = boost::algorithm::join(jvals, ",");
    traj_file << jvals_str << "\n";
  }
  traj_file.close();

  return 0;
}
