/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  InitializeSolutionUtils.cpp
 * @brief Utility methods for initializing trajectory optimization solutions.
 * @Author: Alejandro Escontrela and Yetong Zhang
 */

#include "gtdynamics/utils/InitializeSolutionUtils.h"

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
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

/// Create a random vector where each element's value is sampled
/// from a gaussian distribution N(mean, std).
inline gtsam::Vector GaussianRandomVector(int dim, double mean, double std) {
  if (std == 0.0) return gtsam::Vector::Constant(dim, mean);
  gtsam::Vector rand_vec = gtsam::Vector::Zero(dim);
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(mean, std);
  for (int i = 0; i < dim; i++) rand_vec[i] = distribution(generator);
  return rand_vec;
}

/// Add zero-mean gaussian noise to a Pose.
inline gtsam::Pose3 addGaussianNoise(const gtsam::Pose3& T, double std) {
  gtsam::Point3 p = gtsam::Point3(T.translation().vector() +
                                  GaussianRandomVector(3, 0.0, std));
  gtsam::Rot3 R = gtsam::Rot3::Expmap(gtsam::Rot3::Logmap(T.rotation()) +
                                      GaussianRandomVector(3, 0.0, std));
  return gtsam::Pose3(R, p);
}

gtsam::Values InitializeSolutionInterpolation(
    const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
    const gtsam::Pose3& wTl_f, const double& T_s, const double& T_f,
    const double& dt, const double& gaussian_noise,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  gtsam::Values init_vals;

  // Initial and final discretized timesteps.
  int n_steps_init = static_cast<int>(std::round(T_s / dt));
  int n_steps_final = static_cast<int>(std::round(T_f / dt));

  gtsam::Point3 wPl_i = wTl_i.translation(), wPl_f = wTl_f.translation();
  gtsam::Rot3 wRl_i = wTl_i.rotation(), wRl_f = wTl_f.rotation();

  // Initialize joint angles and velocities to 0.
  gtdynamics::JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert(joint->getKey(), GaussianRandomVector(1, 0, gaussian_noise)[0]);
    jvels.insert(joint->getKey(), GaussianRandomVector(1, 0, gaussian_noise)[0]);
  }

  double t_elapsed = T_s;
  for (int t = n_steps_init; t <= n_steps_final; t++) {
    double s = (t_elapsed - T_s) / (T_f - T_s);

    // Compute interpolated pose for link.
    gtsam::Point3 wPl_t = (1 - s) * wPl_i + s * wPl_f;
    gtsam::Rot3 wRl_t = wRl_i.slerp(s, wRl_f);
    gtsam::Pose3 wTl_t =
        addGaussianNoise(gtsam::Pose3(wRl_t, wPl_t), gaussian_noise);

    // Compute forward dynamics to obtain remaining link poses.
    auto fk_results = robot.forwardKinematics(jangles, jvels, link_name, wTl_t);
    for (auto&& pose_result : fk_results.first)
      init_vals.insert(gtdynamics::PoseKey(
                           robot.getLinkByName(pose_result.first)->getID(), t),
                       pose_result.second);

    // Initialize link dynamics to 0.
    for (auto&& link : robot.links()) {
      init_vals.insert(gtdynamics::TwistKey(link->getID(), t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
      init_vals.insert(gtdynamics::TwistAccelKey(link->getID(), t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
    }

    // Initialize joint kinematics/dynamics to 0.
    for (auto&& joint : robot.joints()) {
      int j = joint->getID();
      init_vals.insert(
          gtdynamics::WrenchKey(joint->parentLink()->getID(), j, t),
          GaussianRandomVector(6, 0.0, gaussian_noise));
      init_vals.insert(gtdynamics::WrenchKey(joint->childLink()->getID(), j, t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
      init_vals.insert(gtdynamics::TorqueKey(j, t),
                       GaussianRandomVector(1, 0, gaussian_noise)[0]);
      init_vals.insert(gtdynamics::JointAngleKey(j, t),
                       GaussianRandomVector(1, 0, gaussian_noise)[0]);
      init_vals.insert(gtdynamics::JointVelKey(j, t),
                       GaussianRandomVector(1, 0, gaussian_noise)[0]);
      init_vals.insert(gtdynamics::JointAccelKey(j, t),
                       GaussianRandomVector(1, 0, gaussian_noise)[0]);
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
            GaussianRandomVector(6, 0.0, gaussian_noise));
      }
    }
    t_elapsed += dt;
  }

  return init_vals;
}

gtsam::Values InitializeSolutionInterpolationMultiPhase(
    const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
    const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
    const double& dt, const double& gaussian_noise,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  gtsam::Values init_vals;
  gtsam::Pose3 pose = wTl_i;
  double curr_t = 0.0;
  for (size_t i = 0; i < wTl_t.size(); i++) {
    gtsam::Values phase_vals = InitializeSolutionInterpolation(
        robot, link_name, pose, wTl_t[i], curr_t, ts[i], dt, gaussian_noise,
        contact_points);
    for (auto&& key_value_pair : phase_vals)
      init_vals.tryInsert(key_value_pair.key, key_value_pair.value);
    pose = wTl_t[i];
    curr_t = ts[i];
  }
  return init_vals;
}

/** @fn Iteratively solve for the robot kinematics with contacts.
 *
 * @param[in] robot           A gtdynamics::Robot object.
 * @param[in] link_name       The name of the link whose pose to interpolate.
 * @param[in] wTl_i           The initial pose of the link.
 * @param[in] wTl_t           A vector of desired poses.
 * @param[in] ts              Times at which poses start and end.
 * @param[in] dt              The duration of a single timestep.
 * @param[in] contact_points  ContactPoint objects.
 * @return Initial solution stored in gtsam::Values object.
 */
gtsam::Values InitializeSolutionInverseKinematics(
    const Robot& robot, const std::string& link_name, const gtsam::Pose3& wTl_i,
    const std::vector<gtsam::Pose3>& wTl_t, const std::vector<double>& ts,
    const double& dt, const double& gaussian_noise,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  gtsam::Point3 wPl_i = wTl_i.translation();  // Initial translation.
  gtsam::Rot3 wRl_i = wTl_i.rotation();       // Initial rotation.
  double t_i = 0.0;                           // Time elapsed.

  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();

  // Linearly interpolated pose for link at each discretized timestep.
  std::vector<gtsam::Pose3> wTl_dt;
  for (size_t i = 0; i < ts.size(); i++) {
    gtsam::Point3 wPl_t = wTl_t[i].translation();  // des P.
    gtsam::Rot3 wRl_t = wTl_t[i].rotation();       // des R.
    double t_ti = t_i, t_t = ts[i];                // Initial and final times.

    for (int t = std::lround(t_i / dt); t < std::lround(t_t / dt); t++) {
      double s = (t_i - t_ti) / (t_t - t_ti);  // Normalized phase progress.

      // Compute interpolated pose for link.
      gtsam::Point3 wPl_s = (1 - s) * wPl_i + s * wPl_t;
      gtsam::Rot3 wRl_s = wRl_i.slerp(s, wRl_t);
      gtsam::Pose3 wTl_s = gtsam::Pose3(wRl_s, wPl_s);
      wTl_dt.push_back(wTl_s);
      t_i += dt;
    }

    wPl_i = wPl_t;
    wRl_i = wRl_t;
  }
  wTl_dt.push_back(wTl_t[wTl_t.size() - 1]);  // Add the final pose.

  gtsam::Pose3 wTl_i_processed;
  if (gaussian_noise > 0.0) {
    wTl_i_processed = addGaussianNoise(wTl_i, gaussian_noise);
    for (size_t i = 0; i < wTl_dt.size(); i++)
      wTl_dt[i] = addGaussianNoise(wTl_dt[i], gaussian_noise);
  } else {
    wTl_i_processed = wTl_i;
  }

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  gtsam::Values init_vals, init_vals_t;

  // Initial pose and joint angles are known a priori.
  gtsam::Vector z_six = GaussianRandomVector(6, 0, gaussian_noise),
                z_one = GaussianRandomVector(1, 0, gaussian_noise);

  gtdynamics::JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert(joint->getKey(), GaussianRandomVector(1, 0, gaussian_noise)[0]);
    jvels.insert(joint->getKey(), GaussianRandomVector(1, 0, gaussian_noise)[0]);
  }
  // Compute forward dynamics to obtain remaining link poses.
  auto fk_results =
      robot.forwardKinematics(jangles, jvels, link_name, wTl_i_processed);
  for (auto&& pose_result : fk_results.first)
    init_vals_t.insert(
        gtdynamics::PoseKey(robot.getLinkByName(pose_result.first)->getID(), 0),
        pose_result.second);
  for (auto&& joint : robot.joints())
    init_vals_t.insert(gtdynamics::JointAngleKey(joint->getID(), 0), z_one[0]);

  auto dgb = gtdynamics::DynamicsGraph();
  for (int t = 0; t <= std::lround(ts[ts.size() - 1] / dt); t++) {
    gtsam::NonlinearFactorGraph kfg =
        dgb.qFactors(robot, t, gravity, contact_points);
    kfg.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtdynamics::PoseKey(robot.getLinkByName(link_name)->getID(), t),
        wTl_dt[t], gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

    gtsam::LevenbergMarquardtOptimizer optimizer(kfg, init_vals_t);
    gtsam::Values results = optimizer.optimize();

    init_vals.insert(results);

    // Add zero initial values for remaining variables.
    for (auto&& link : robot.links()) {
      init_vals.insert(gtdynamics::TwistKey(link->getID(), t), z_six);
      init_vals.insert(gtdynamics::TwistAccelKey(link->getID(), t), z_six);
    }
    for (auto&& joint : robot.joints()) {
      int j = joint->getID();
      init_vals.insert(
          gtdynamics::WrenchKey(joint->parentLink()->getID(), j, t), z_six);
      init_vals.insert(gtdynamics::WrenchKey(joint->childLink()->getID(), j, t),
                       z_six);
      init_vals.insert(gtdynamics::TorqueKey(j, t), z_one[0]);
      init_vals.insert(gtdynamics::JointVelKey(j, t), z_one[0]);
      init_vals.insert(gtdynamics::JointAccelKey(j, t), z_one[0]);
    }
    if (contact_points) {
      for (auto&& contact_point : *contact_points) {
        int link_id = -1;
        for (auto& link : robot.links()) {
          if (link->name() == contact_point.name) link_id = link->getID();
        }
        if (link_id == -1) throw std::runtime_error("Link not found.");
        init_vals.insert(
            gtdynamics::ContactWrenchKey(link_id, contact_point.contact_id, t),
            z_six);
      }
    }

    // Update initial values for next timestep.
    init_vals_t.clear();
    for (auto&& link : robot.links())
      init_vals_t.insert(gtdynamics::PoseKey(link->getID(), t + 1),
                         results.at(gtdynamics::PoseKey(link->getID(), t))
                             .cast<gtsam::Pose3>());
    for (auto&& joint : robot.joints())
      init_vals_t.insert(
          gtdynamics::JointAngleKey(joint->getID(), t + 1),
          results.atDouble(gtdynamics::JointAngleKey(joint->getID(), t)));
  }

  return init_vals;
}

gtsam::Values ZeroValues(const Robot& robot, const int t,
                         const double& gaussian_noise,
                         const boost::optional<ContactPoints>& contact_points) {
  gtsam::Values zero_values;
  for (auto& link : robot.links()) {
    int i = link->getID();
    zero_values.insert(gtdynamics::PoseKey(i, t),
                       addGaussianNoise(link->wTcom(), gaussian_noise));
    zero_values.insert(gtdynamics::TwistKey(i, t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
    zero_values.insert(gtdynamics::TwistAccelKey(i, t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
  }
  for (auto& joint : robot.joints()) {
    int j = joint->getID();
    auto parent_link = joint->parentLink();
    auto child_link = joint->childLink();
    zero_values.insert(gtdynamics::WrenchKey(parent_link->getID(), j, t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
    zero_values.insert(gtdynamics::WrenchKey(child_link->getID(), j, t),
                       GaussianRandomVector(6, 0.0, gaussian_noise));
    zero_values.insert(gtdynamics::TorqueKey(j, t),
                       GaussianRandomVector(1, 0.0, gaussian_noise)[0]);
    zero_values.insert(gtdynamics::JointAngleKey(j, t),
                       GaussianRandomVector(1, 0.0, gaussian_noise)[0]);
    zero_values.insert(gtdynamics::JointVelKey(j, t),
                       GaussianRandomVector(1, 0.0, gaussian_noise)[0]);
    zero_values.insert(gtdynamics::JointAccelKey(j, t),
                       GaussianRandomVector(1, 0.0, gaussian_noise)[0]);
  }
  if (contact_points) {
    for (auto&& contact_point : *contact_points) {
      int link_id = -1;
      for (auto& link : robot.links()) {
        if (link->name() == contact_point.name) link_id = link->getID();
      }

      if (link_id == -1) throw std::runtime_error("Link not found.");

      zero_values.insert(
          gtdynamics::ContactWrenchKey(link_id, contact_point.contact_id, t),
          GaussianRandomVector(6, 0.0, gaussian_noise));
    }
  }

  return zero_values;
}

gtsam::Values ZeroValuesTrajectory(
    const Robot& robot, const int num_steps, const int num_phases,
    const double& gaussian_noise,
    const boost::optional<ContactPoints>& contact_points) {
  gtsam::Values z_values;
  for (int t = 0; t <= num_steps; t++) {
    z_values.insert(ZeroValues(robot, t, gaussian_noise, contact_points));
  }
  if (num_phases > 0) {
    for (int phase = 0; phase <= num_phases; phase++) {
      z_values.insert(gtdynamics::PhaseKey(phase), 0.0);
    }
  }
  return z_values;
}

}  // namespace gtdynamics
