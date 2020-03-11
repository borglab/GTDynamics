/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  initialize_solution_utils.cpp
 * @brief Utility methods for initializing trajectory optimization solutions.
 * @Author: Alejandro Escontrela and Yetong Zhang
 */

#include "gtdynamics/utils/initialize_solution_utils.h"

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PoseGoalFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>
#include <utility>
#include <vector>

namespace gtdynamics {

gtsam::Values initialize_solution_interpolation(
    const Robot& robot, const std::string& link_name,
    const gtsam::Pose3& wTl_i, const gtsam::Pose3& wTl_f, const double& T_i,
    const double& T_f, const double& dt,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  gtsam::Values init_vals;

  // Initial and final discretized timesteps.
  int n_steps_init = static_cast<int>(std::round(T_i / dt));
  int n_steps_final = static_cast<int>(std::round(T_f / dt));

  gtsam::Point3 wPl_i = wTl_i.translation(), wPl_f = wTl_f.translation();
  gtsam::Rot3 wRl_i = wTl_i.rotation(), wRl_f = wTl_f.rotation();

  // Initialize joint angles and velocities to 0.
  gtdynamics::Robot::JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert(std::make_pair(joint->name(), 0.0));
    jvels.insert(std::make_pair(joint->name(), 0.0));
  }

  gtsam::Vector zero_twist = gtsam::Vector6::Zero(),
                zero_accel = gtsam::Vector6::Zero(),
                zero_wrench = gtsam::Vector6::Zero(),
                zero_torque = gtsam::Vector1::Zero(),
                zero_q = gtsam::Vector1::Zero(),
                zero_v = gtsam::Vector1::Zero(),
                zero_a = gtsam::Vector1::Zero();

  double t_elapsed = T_i;
  for (int t = n_steps_init; t <= n_steps_final; t++) {
    double s = (t_elapsed - T_i) / (T_f - T_i);

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

gtsam::Values initialize_solution_interpolation_multi_phase(
    const Robot& robot, const std::string& link_name,
    const gtsam::Pose3& wTl_i, const std::vector<gtsam::Pose3>& wTl_t,
    const std::vector<double>& ts, const double& dt,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  gtsam::Values init_vals;
  gtsam::Pose3 pose = wTl_i;
  double curr_t = 0.0;
  for (size_t i = 0; i < wTl_t.size(); i++) {
    gtsam::Values phase_vals = initialize_solution_interpolation(
      robot, link_name, pose, wTl_t[i], curr_t, ts[i], dt, contact_points);
    for (auto&& key_value_pair : phase_vals)
      init_vals.tryInsert(key_value_pair.key, key_value_pair.value);
    pose = wTl_t[i];
    curr_t = ts[i];
  }
  return init_vals;
}

}  // namespace gtdynamics
