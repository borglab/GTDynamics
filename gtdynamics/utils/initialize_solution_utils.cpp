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
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <string>
#include <utility>
#include <vector>

using gtsam::Pose3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector,
    gtsam::Point3, gtsam::Rot3, gtsam::Sampler, gtsam::Values;

namespace gtdynamics {

/// Add zero-mean gaussian noise to a Pose.
inline Pose3 addGaussianNoiseToPose(const Pose3& T, double std,
                                    Sampler sampler) {
  Vector rand_vec = sampler.sample();
  Point3 p = Point3(T.translation().vector() + rand_vec.head(3));
  Rot3 R = Rot3::Expmap(Rot3::Logmap(T.rotation()) + rand_vec.tail<3>());
  return Pose3(R, p);
}

Values InitializeSolutionInterpolation(
    const Robot& robot, const std::string& link_name, const Pose3& wTl_i,
    const Pose3& wTl_f, const double& T_s, const double& T_f, const double& dt,
    const double& gaussian_noise,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  Values init_vals;

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  // Initial and final discretized timesteps.
  int n_steps_init = static_cast<int>(std::round(T_s / dt));
  int n_steps_final = static_cast<int>(std::round(T_f / dt));

  Point3 wPl_i = wTl_i.translation(), wPl_f = wTl_f.translation();
  Rot3 wRl_i = wTl_i.rotation(), wRl_f = wTl_f.rotation();

  // Initialize joint angles and velocities to 0.
  JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert<double>(joint->getKey(), sampler.sample()[0]);
    jvels.insert<double>(joint->getKey(), sampler.sample()[0]);
  }

  double t_elapsed = T_s;
  for (int t = n_steps_init; t <= n_steps_final; t++) {
    double s = (t_elapsed - T_s) / (T_f - T_s);

    // Compute interpolated pose for link.
    Point3 wPl_t = (1 - s) * wPl_i + s * wPl_f;
    Rot3 wRl_t = wRl_i.slerp(s, wRl_f);
    Pose3 wTl_t =
        addGaussianNoiseToPose(Pose3(wRl_t, wPl_t), gaussian_noise, sampler);

    // Compute forward dynamics to obtain remaining link poses.
    auto fk_results = robot.forwardKinematics(jangles, jvels, link_name, wTl_t);
    for (auto&& pose_result : fk_results.first)
      init_vals.insert(
          PoseKey(robot.getLinkByName(pose_result.first)->getID(), t),
          pose_result.second);

    // Initialize link dynamics to 0.
    for (auto&& link : robot.links()) {
      init_vals.insert(TwistKey(link->getID(), t), sampler.sample());
      init_vals.insert(TwistAccelKey(link->getID(), t), sampler.sample());
    }

    // Initialize joint kinematics/dynamics to 0.
    for (auto&& joint : robot.joints()) {
      int j = joint->getID();
      init_vals.insert(WrenchKey(joint->parentLink()->getID(), j, t),
                       sampler.sample());
      init_vals.insert(WrenchKey(joint->childLink()->getID(), j, t),
                       sampler.sample());
      init_vals.insert(TorqueKey(j, t), sampler.sample()[0]);
      init_vals.insert(JointAngleKey(j, t), sampler.sample()[0]);
      init_vals.insert(JointVelKey(j, t), sampler.sample()[0]);
      init_vals.insert(JointAccelKey(j, t), sampler.sample()[0]);
    }

    // Initialize contacts to 0.
    if (contact_points) {
      for (auto&& contact_point : *contact_points) {
        int link_id = -1;
        for (auto& link : robot.links()) {
          if (link->name() == contact_point.name) link_id = link->getID();
        }
        if (link_id == -1) throw std::runtime_error("Link not found.");
        init_vals.insert(ContactWrenchKey(link_id, contact_point.contact_id, t),
                         sampler.sample());
      }
    }
    t_elapsed += dt;
  }

  return init_vals;
}

Values InitializeSolutionInterpolationMultiPhase(
    const Robot& robot, const std::string& link_name, const Pose3& wTl_i,
    const std::vector<Pose3>& wTl_t, const std::vector<double>& ts,
    const double& dt, const double& gaussian_noise,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  Values init_vals;
  Pose3 pose = wTl_i;
  double curr_t = 0.0;
  for (size_t i = 0; i < wTl_t.size(); i++) {
    Values phase_vals = InitializeSolutionInterpolation(
        robot, link_name, pose, wTl_t[i], curr_t, ts[i], dt, gaussian_noise,
        contact_points);
    for (auto&& key_value_pair : phase_vals)
      init_vals.tryInsert(key_value_pair.key, key_value_pair.value);
    pose = wTl_t[i];
    curr_t = ts[i];
  }
  return init_vals;
}

Values InitializeSolutionInverseKinematics(
    const Robot& robot, const std::string& link_name, const Pose3& wTl_i,
    const std::vector<Pose3>& wTl_t, const std::vector<double>& ts,
    const double& dt, const double& gaussian_noise,
    const boost::optional<std::vector<ContactPoint>>& contact_points) {
  Point3 wPl_i = wTl_i.translation();  // Initial translation.
  Rot3 wRl_i = wTl_i.rotation();       // Initial rotation.
  double t_i = 0.0;                    // Time elapsed.

  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  // Linearly interpolated pose for link at each discretized timestep.
  std::vector<Pose3> wTl_dt;
  for (size_t i = 0; i < ts.size(); i++) {
    Point3 wPl_t = wTl_t[i].translation();  // des P.
    Rot3 wRl_t = wTl_t[i].rotation();       // des R.
    double t_ti = t_i, t_t = ts[i];         // Initial and final times.

    for (int t = std::lround(t_i / dt); t < std::lround(t_t / dt); t++) {
      double s = (t_i - t_ti) / (t_t - t_ti);  // Normalized phase progress.

      // Compute interpolated pose for link.
      Point3 wPl_s = (1 - s) * wPl_i + s * wPl_t;
      Rot3 wRl_s = wRl_i.slerp(s, wRl_t);
      Pose3 wTl_s = Pose3(wRl_s, wPl_s);
      wTl_dt.push_back(wTl_s);
      t_i += dt;
    }

    wPl_i = wPl_t;
    wRl_i = wRl_t;
  }
  wTl_dt.push_back(wTl_t[wTl_t.size() - 1]);  // Add the final pose.

  Pose3 wTl_i_processed;
  if (gaussian_noise > 0.0) {
    wTl_i_processed = addGaussianNoiseToPose(wTl_i, gaussian_noise, sampler);
    for (size_t i = 0; i < wTl_dt.size(); i++)
      wTl_dt[i] = addGaussianNoiseToPose(wTl_dt[i], gaussian_noise, sampler);
  } else {
    wTl_i_processed = wTl_i;
  }

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  Values init_vals, init_vals_t;

  // Initial pose and joint angles are known a priori.
  JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert<double>(joint->getKey(), sampler.sample()[0]);
    jvels.insert<double>(joint->getKey(), sampler.sample()[0]);
  }
  // Compute forward dynamics to obtain remaining link poses.
  auto fk_results =
      robot.forwardKinematics(jangles, jvels, link_name, wTl_i_processed);
  for (auto&& pose_result : fk_results.first)
    init_vals_t.insert(
        PoseKey(robot.getLinkByName(pose_result.first)->getID(), 0),
        pose_result.second);
  for (auto&& joint : robot.joints())
    init_vals_t.insert(JointAngleKey(joint->getID(), 0), sampler.sample()[0]);

  auto dgb = DynamicsGraph();
  for (int t = 0; t <= std::lround(ts[ts.size() - 1] / dt); t++) {
    gtsam::NonlinearFactorGraph kfg =
        dgb.qFactors(robot, t, gravity, contact_points);
    kfg.add(gtsam::PriorFactor<Pose3>(
        PoseKey(robot.getLinkByName(link_name)->getID(), t), wTl_dt[t],
        gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

    gtsam::LevenbergMarquardtOptimizer optimizer(kfg, init_vals_t);
    Values results = optimizer.optimize();

    init_vals.insert(results);

    // Add zero initial values for remaining variables.
    for (auto&& link : robot.links()) {
      init_vals.insert(TwistKey(link->getID(), t), sampler.sample());
      init_vals.insert(TwistAccelKey(link->getID(), t), sampler.sample());
    }
    for (auto&& joint : robot.joints()) {
      int j = joint->getID();
      init_vals.insert(WrenchKey(joint->parentLink()->getID(), j, t),
                       sampler.sample());
      init_vals.insert(WrenchKey(joint->childLink()->getID(), j, t),
                       sampler.sample());
      init_vals.insert(TorqueKey(j, t), sampler.sample()[0]);
      init_vals.insert(JointVelKey(j, t), sampler.sample()[0]);
      init_vals.insert(JointAccelKey(j, t), sampler.sample()[0]);
    }
    if (contact_points) {
      for (auto&& contact_point : *contact_points) {
        int link_id = -1;
        for (auto& link : robot.links()) {
          if (link->name() == contact_point.name) link_id = link->getID();
        }
        if (link_id == -1) throw std::runtime_error("Link not found.");
        init_vals.insert(ContactWrenchKey(link_id, contact_point.contact_id, t),
                         sampler.sample());
      }
    }

    // Update initial values for next timestep.
    init_vals_t.clear();
    for (auto&& link : robot.links())
      init_vals_t.insert(PoseKey(link->getID(), t + 1),
                         results.at<Pose3>(PoseKey(link->getID(), t)));
    for (auto&& joint : robot.joints())
      init_vals_t.insert(JointAngleKey(joint->getID(), t + 1),
                         results.atDouble(JointAngleKey(joint->getID(), t)));
  }

  return init_vals;
}

Values ZeroValues(const Robot& robot, const int t, const double& gaussian_noise,
                  const boost::optional<ContactPoints>& contact_points) {
  Values zero_values;

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  for (auto& link : robot.links()) {
    int i = link->getID();
    zero_values.insert(
        PoseKey(i, t),
        addGaussianNoiseToPose(link->wTcom(), gaussian_noise, sampler));
    zero_values.insert(TwistKey(i, t), sampler.sample());
    zero_values.insert(TwistAccelKey(i, t), sampler.sample());
  }
  for (auto& joint : robot.joints()) {
    int j = joint->getID();
    auto parent_link = joint->parentLink();
    auto child_link = joint->childLink();
    zero_values.insert(WrenchKey(parent_link->getID(), j, t), sampler.sample());
    zero_values.insert(WrenchKey(child_link->getID(), j, t), sampler.sample());
    std::vector<gtsam::LabeledSymbol> keys = {
        TorqueKey(j, t), JointAngleKey(j, t), JointVelKey(j, t),
        JointAccelKey(j, t)};
    for (size_t i = 0; i < keys.size(); i++)
      zero_values.insert(keys[i], sampler.sample()[0]);
  }
  if (contact_points) {
    for (auto&& contact_point : *contact_points) {
      int link_id = -1;
      for (auto& link : robot.links()) {
        if (link->name() == contact_point.name) link_id = link->getID();
      }

      if (link_id == -1) throw std::runtime_error("Link not found.");

      zero_values.insert(ContactWrenchKey(link_id, contact_point.contact_id, t),
                         sampler.sample());
    }
  }

  return zero_values;
}

Values ZeroValuesTrajectory(
    const Robot& robot, const int num_steps, const int num_phases,
    const double& gaussian_noise,
    const boost::optional<ContactPoints>& contact_points) {
  Values z_values;
  for (int t = 0; t <= num_steps; t++)
    z_values.insert(ZeroValues(robot, t, gaussian_noise, contact_points));
  if (num_phases > 0) {
    for (int phase = 0; phase <= num_phases; phase++)
      z_values.insert(PhaseKey(phase), 0.0);
  }
  return z_values;
}

}  // namespace gtdynamics
