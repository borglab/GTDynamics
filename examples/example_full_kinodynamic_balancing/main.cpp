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

#define GROUND_HEIGHT -0.191839

/** @fn Initialize solution via linear interpolation of initial and final pose.
 *
 * @param[in] robot           A gtdynamics::Robot object.
 * @param[in] link_name       The name of the link whose pose to interpolate.
 * @param[in] wTl_i           The initial pose of the link.
 * @param[in] wTl_f           The final pose of the link.
 * @param[in] T               Duration of trajectory (s.).
 * @param[in] dt              The duration of a single timestep.
 * @param[in] contact_points  The duration of a single timestep.
 * @return Initial solution stored in gtsam::Values object.
 */
gtsam::Values initialize_solution_interpolation(
    const gtdynamics::Robot& robot, const std::string& link_name,
    const gtsam::Pose3& wTl_i, const gtsam::Pose3& wTl_f, const double& T,
    const double& dt,
    const boost::optional<std::vector<gtdynamics::ContactPoint>>&
        contact_points = boost::none) {
  gtsam::Values init_vals;

  int n_steps = static_cast<int>(std::ceil(T / dt));
  gtsam::Point3 wPl_i = wTl_i.translation(), wPl_f = wTl_f.translation();
  gtsam::Rot3 wRl_i = wTl_i.rotation(), wRl_f = wTl_f.rotation();

  // Initialize joint angles and velocities to 0.
  gtdynamics::Robot::JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert(std::make_pair(joint->name(), 0.0));
    jvels.insert(std::make_pair(joint->name(), 0.0));
  }

  gtsam::Vector zero_twist = gtsam::Vector6::Zero(),
    zero_accel = gtsam::Vector6::Zero(), zero_wrench = gtsam::Vector6::Zero(),
    zero_torque = gtsam::Vector1::Zero(), zero_q = gtsam::Vector1::Zero(),
    zero_v = gtsam::Vector1::Zero(), zero_a = gtsam::Vector1::Zero();

  double t_elapsed = 0.0;
  for (int t = 0; t <= n_steps; t++) {
    double s = t_elapsed / T;

    // Compute interpolated pose for link.
    gtsam::Point3 wPl_t = (1 - s) * wPl_i + s * wPl_f;
    gtsam::Rot3 wRl_t = wRl_i.slerp(s, wRl_f);
    gtsam::Pose3 wTl_t = gtsam::Pose3(wRl_t, wPl_t);

    // std::cout << "\t Pose: [" << wPl_t << " | " << wRl_t.rpy() << "]" <<
    // std::endl;

    // Compute forward dynamics to obtain remaining link poses.
    auto fk_results = robot.forwardKinematics(jangles, jvels, link_name, wTl_t);
    for (auto&& pose_result : fk_results.first)
      init_vals.insert(gtdynamics::PoseKey(
                           robot.getLinkByName(pose_result.first)->getID(), t),
                       pose_result.second);

    // Initialize link dynamics to 0.
    for (auto&& link : robot.links()) {
      init_vals.insert(gtdynamics::TwistKey(link->getID(), t), zero_twist);
      init_vals.insert(gtdynamics::TwistAccelKey(link->getID(), t), zero_accel);
    }

    // Initialize joint kinematics/dynamics to 0.
    for (auto&& joint : robot.joints()) {
      int j = joint->getID();
      init_vals.insert(
          gtdynamics::WrenchKey(joint->parentLink()->getID(), j, t),
          zero_wrench);
      init_vals.insert(gtdynamics::WrenchKey(joint->childLink()->getID(), j, t),
                       zero_wrench);
      init_vals.insert(gtdynamics::TorqueKey(j, t), zero_torque[0]);
      init_vals.insert(gtdynamics::JointAngleKey(j, t), zero_q[0]);
      init_vals.insert(gtdynamics::JointVelKey(j, t), zero_v[0]);
      init_vals.insert(gtdynamics::JointAccelKey(j, t), zero_a[0]);
    }

    // Initialize contacts to 0.
    if (contact_points) {
      for (auto&& contact_point : *contact_points) {
        int link_id = -1;
        for (auto& link : robot.links()) {
          if (link->name() == contact_point.name) link_id = link->getID();
        }
        if (link_id == -1) throw std::runtime_error("Link not found.");
        init_vals.insert(
            gtdynamics::ContactWrenchKey(link_id, contact_point.contact_id, t),
            zero_wrench);
      }
    }
    t_elapsed += dt;
  }

  return init_vals;
}

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

  // Specify boundary conditions for base.
  gtsam::Pose3 base_pose_init = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3());
  gtsam::Pose3 base_pose_final =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 10.0, 0.0, 0.0),
                   gtsam::Point3(0, 0, 0.1));
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

  gtsam::NonlinearFactorGraph graph = graph_builder.trajectoryFG(
      vision60, t_steps, dt,
      gtdynamics::DynamicsGraph::CollocationScheme::Euler, gravity, boost::none,
      contact_points, mu);

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
  objective_factors.add(gtdynamics::PoseGoalFactor(
      gtdynamics::PoseKey(base_link->getID(), t_steps / 2),
      gtsam::noiseModel::Constrained::All(6),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(M_PI / 10.0, 0.0, 0.0),
                   gtsam::Point3(0, 0, 0.1))));
  objective_factors.add(gtdynamics::PoseGoalFactor(
      gtdynamics::PoseKey(base_link->getID(), t_steps),
      gtsam::noiseModel::Constrained::All(6),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(-M_PI / 10.0, 0.0, 0.0),
                   gtsam::Point3(0, 0, 0.1))));

  objective_factors.add(gtdynamics::PoseGoalFactor(
      gtdynamics::PoseKey(base_link->getID(), t_steps),
      gtsam::noiseModel::Constrained::All(6), base_pose_final));

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

  // Optimize!
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
  //             angles                vels                accels
  traj_file << jnames_str << "," << jnames_str << "," << jnames_str << "," << jnames_str << "\n";
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
