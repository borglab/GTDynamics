/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  initialize_solution_utils.cpp
 * @brief Utility methods for initializing trajectory optimization solutions.
 * @authors Alejandro Escontrela, Yetong Zhang, Varun Agrawal
 */

#include "gtdynamics/utils/initialize_solution_utils.h"

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

using gtsam::Pose3, gtsam::Vector3, gtsam::Vector6, gtsam::Vector,
    gtsam::Point3, gtsam::Rot3, gtsam::Sampler, gtsam::Values;

namespace gtdynamics {

Pose3 AddGaussianNoiseToPose(const Pose3& T, const Sampler& sampler) {
  Vector6 xi = sampler.sample();
  return T.expmap(xi);
}

std::vector<Pose3> InterpolatePoses(const Pose3& wTl_i,
                                    const std::vector<Pose3>& wTl_t, double t_i,
                                    const std::vector<double>& timesteps,
                                    double dt) {
  std::vector<Pose3> wTl_dt;
  Pose3 wTl = wTl_i;

  for (size_t i = 0; i < timesteps.size(); i++) {
    double t_f = timesteps[i];  // Final time for each step.

    for (double t = t_i; t <= t_f; t += dt) {
      // Normalized phase progress.
      double s = (t - t_i) / (t_f - t_i);

      // Compute interpolated pose for link.
      Pose3 wTl_s = gtsam::interpolate<Pose3>(wTl, wTl_t[i], s);

      wTl_dt.push_back(wTl_s);
    }
    wTl = wTl_t[i];
  }
  wTl_dt.push_back(wTl_t[wTl_t.size() - 1]);  // Add the final pose.

  return wTl_dt;
}

Values AddForwardKinematicsPoses(const Robot& robot, size_t t,
                                 const std::string& link_name,
                                 const JointValues& joint_angles,
                                 const JointValues& joint_velocities,
                                 const Pose3& wTl_i, Values values) {
  auto fk_results =
      robot.forwardKinematics(joint_angles, joint_velocities, link_name, wTl_i);
  for (auto&& pose_result : fk_results.first) {
    values.insert(PoseKey(robot.link(pose_result.first)->id(), t),
                  pose_result.second);
  }
  return values;
}

Values InitializePosesAndJoints(const Robot& robot, const Pose3& wTl_i,
                                const std::vector<Pose3>& wTl_t,
                                const std::string& link_name, double t_i,
                                const std::vector<double>& timesteps, double dt,
                                const Sampler& sampler,
                                std::vector<Pose3>& wTl_dt) {
  // Linearly interpolated pose for link at each discretized timestep.
  wTl_dt = InterpolatePoses(wTl_i, wTl_t, t_i, timesteps, dt);

  Pose3 wTl_i_processed = AddGaussianNoiseToPose(wTl_i, sampler);
  for (size_t i = 0; i < wTl_dt.size(); i++) {
    wTl_dt[i] = AddGaussianNoiseToPose(wTl_dt[i], sampler);
  }

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  Values init_vals_t;

  // Initial pose and joint angles are known a priori.
  JointValues joint_angles, joint_velocities;
  for (auto&& joint : robot.joints()) {
    joint_angles.insert(std::make_pair(joint->name(), sampler.sample()[0]));
    joint_velocities.insert(std::make_pair(joint->name(), sampler.sample()[0]));

    init_vals_t.insert(JointAngleKey(joint->id(), 0), sampler.sample()[0]);
  }

  // Compute forward dynamics to obtain remaining link poses.
  init_vals_t =
      AddForwardKinematicsPoses(robot, 0, link_name, joint_angles,
                                joint_velocities, wTl_i_processed, init_vals_t);

  return init_vals_t;
}

Values InitializeSolutionInterpolation(
    const Robot& robot, const std::string& link_name, const Pose3& wTl_i,
    const Pose3& wTl_f, double T_s, double T_f, double dt,
    double gaussian_noise,
    const boost::optional<ContactPoints>& contact_points) {
  Values init_vals;

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  // Initial and final discretized timesteps.
  int n_steps_init = std::lround(T_s / dt);
  int n_steps_final = std::lround(T_f / dt);

  double t_elapsed = T_s;

  // Initialize joint angles and velocities to 0.
  JointValues jangles, jvels;
  for (auto&& joint : robot.joints()) {
    jangles.insert(std::make_pair(joint->name(), sampler.sample()[0]));
    jvels.insert(std::make_pair(joint->name(), sampler.sample()[0]));
  }

  for (int t = n_steps_init; t <= n_steps_final; t++) {
    double s = (t_elapsed - T_s) / (T_f - T_s);

    // Compute interpolated pose for link.
    Pose3 wTl_t = AddGaussianNoiseToPose(
        gtsam::interpolate<Pose3>(wTl_i, wTl_f, s), sampler);

    // Compute forward dynamics to obtain remaining link poses.
    // TODO(Alejandro): forwardKinematics needs to get passed the prev link
    // twist
    init_vals = AddForwardKinematicsPoses(robot, t, link_name, jangles, jvels,
                                          wTl_t, init_vals);

    for (auto&& kvp : ZeroValues(robot, t, gaussian_noise, contact_points)) {
      init_vals.tryInsert(kvp.key, kvp.value);
    }

    t_elapsed += dt;
  }

  return init_vals;
}

Values InitializeSolutionInterpolationMultiPhase(
    const Robot& robot, const std::string& link_name, const Pose3& wTl_i,
    const std::vector<Pose3>& wTl_t, const std::vector<double>& ts, double dt,
    double gaussian_noise,
    const boost::optional<ContactPoints>& contact_points) {
  Values init_vals;
  Pose3 pose = wTl_i;
  double curr_t = 0.0;
  for (size_t i = 0; i < wTl_t.size(); i++) {
    Values phase_vals = InitializeSolutionInterpolation(
        robot, link_name, pose, wTl_t[i], curr_t, ts[i], dt, gaussian_noise,
        contact_points);

    for (auto&& key_value_pair : phase_vals) {
      init_vals.tryInsert(key_value_pair.key, key_value_pair.value);
    }
    pose = wTl_t[i];
    curr_t = ts[i];
  }
  return init_vals;
}

Values InitializeSolutionInverseKinematics(
    const Robot& robot, const std::string& link_name, const Pose3& wTl_i,
    const std::vector<Pose3>& wTl_t, const std::vector<double>& timesteps,
    double dt, double gaussian_noise,
    const boost::optional<ContactPoints>& contact_points) {
  double t_i = 0.0;  // Time elapsed.

  Vector3 gravity(0, 0, -9.8);

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  // Link pose at each step
  std::vector<Pose3> wTl_dt;

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  Values init_vals,
      init_vals_t = InitializePosesAndJoints(
          robot, wTl_i, wTl_t, link_name, t_i, timesteps, dt, sampler, wTl_dt);

  DynamicsGraph dgb;
  for (int t = 0; t <= std::round(timesteps[timesteps.size() - 1] / dt); t++) {
    gtsam::NonlinearFactorGraph kfg =
        dgb.qFactors(robot, t, gravity, contact_points);
    kfg.add(gtsam::PriorFactor<Pose3>(
        PoseKey(robot.link(link_name)->id(), t), wTl_dt[t],
        gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

    gtsam::LevenbergMarquardtOptimizer optimizer(kfg, init_vals_t);
    Values results = optimizer.optimize();

    // Add zero initial values for remaining variables.
    init_vals.insert(ZeroValues(robot, t, gaussian_noise, contact_points));

    // Update with the results of the optimizer.
    init_vals.update(results);

    // Update initial values for next timestep.
    init_vals_t.clear();

    for (auto&& link : robot.links()) {
      init_vals_t.insert(PoseKey(link->id(), t + 1),
                         results.at<Pose3>(PoseKey(link->id(), t)));
    }
    for (auto&& joint : robot.joints()) {
      init_vals_t.insert(JointAngleKey(joint->id(), t + 1),
                         results.atDouble(JointAngleKey(joint->id(), t)));
    }
  }

  return init_vals;
}

Values MultiPhaseZeroValuesTrajectory(
    const std::vector<Robot>& robots, const std::vector<int>& phase_steps,
    std::vector<Values> transition_graph_init, double dt_i,
    double gaussian_noise,
    const boost::optional<std::vector<ContactPoints>>& phase_contact_points) {
  Values zero_values;
  int num_phases = robots.size();

  int t = 0;
  if (phase_contact_points) {
    zero_values.insert(
        ZeroValues(robots[0], t, gaussian_noise, (*phase_contact_points)[0]));
  } else {
    zero_values.insert(ZeroValues(robots[0], t, gaussian_noise));
  }

  for (int phase = 0; phase < num_phases; phase++) {
    // in-phase
    for (int phase_step = 0; phase_step < phase_steps[phase] - 1;
         phase_step++) {
      if (phase_contact_points) {
        zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise,
                                      (*phase_contact_points)[phase]));
      } else {
        zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise));
      }
    }

    if (phase == num_phases - 1) {
      if (phase_contact_points) {
        zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise,
                                      (*phase_contact_points)[phase]));
      } else {
        zero_values.insert(ZeroValues(robots[phase], ++t, gaussian_noise));
      }
    } else {
      t++;
      zero_values.insert(transition_graph_init[phase]);
    }
  }

  for (int phase = 0; phase < num_phases; phase++) {
    zero_values.insert(PhaseKey(phase), dt_i);
  }

  return zero_values;
}

Values MultiPhaseInverseKinematicsTrajectory(
    const std::vector<Robot>& robots, const std::string& link_name,
    const std::vector<int>& phase_steps, const Pose3& wTl_i,
    const std::vector<Pose3>& wTl_t, const std::vector<double>& ts,
    std::vector<Values> transition_graph_init, double dt, double gaussian_noise,
    const boost::optional<std::vector<ContactPoints>>& phase_contact_points) {
  double t_i = 0;  // Time elapsed.
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  std::vector<Pose3> wTl_dt;

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  Values init_vals,
      init_vals_t = InitializePosesAndJoints(robots[0], wTl_i, wTl_t, link_name,
                                             t_i, ts, dt, sampler, wTl_dt);

  DynamicsGraph dgb;

  int t = 0;
  int num_phases = robots.size();

  for (int phase = 0; phase < num_phases; phase++) {
    // In-phase.
    int curr_phase_steps =
        phase == (num_phases - 1) ? phase_steps[phase] + 1 : phase_steps[phase];
    for (int phase_step = 0; phase_step < curr_phase_steps; phase_step++) {
      gtsam::NonlinearFactorGraph kfg = dgb.qFactors(
          robots[phase], t, gravity, (*phase_contact_points)[phase]);

      kfg.add(gtsam::PriorFactor<Pose3>(
          PoseKey(robots[phase].link(link_name)->id(), t), wTl_dt[t],
          gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

      gtsam::LevenbergMarquardtOptimizer optimizer(kfg, init_vals_t);
      Values results = optimizer.optimize();

      init_vals.insert(results);

      // Update initial values for next timestep.
      init_vals_t.clear();
      for (auto&& link : robots[phase].links()) {
        int link_id = link->id();
        init_vals_t.insert(PoseKey(link_id, t + 1),
                           results.at<Pose3>(PoseKey(link_id, t)));
      }

      for (auto&& joint : robots[phase].joints()) {
        int joint_id = joint->id();
        init_vals_t.insert(JointAngleKey(joint_id, t + 1),
                           results.atDouble(JointAngleKey(joint_id, t)));
      }
      t++;
    }
  }

  Values zero_values =
      MultiPhaseZeroValuesTrajectory(robots, phase_steps, transition_graph_init,
                                     dt, gaussian_noise, phase_contact_points);

  for (auto&& key_value_pair : zero_values) {
    init_vals.tryInsert(key_value_pair.key, key_value_pair.value);
  }

  return init_vals;
}

Values ZeroValues(const Robot& robot, const int t, double gaussian_noise,
                  const boost::optional<ContactPoints>& contact_points) {
  Values zero_values;

  auto sampler_noise_model =
      gtsam::noiseModel::Diagonal::Sigmas(Vector6::Constant(6, gaussian_noise));
  Sampler sampler(sampler_noise_model);

  // Initialize link dynamics to 0.
  for (auto&& link : robot.links()) {
    int i = link->id();
    zero_values.insert(PoseKey(i, t),
                       AddGaussianNoiseToPose(link->wTcom(), sampler));
    zero_values.insert(TwistKey(i, t), sampler.sample());
    zero_values.insert(TwistAccelKey(i, t), sampler.sample());
  }

  // Initialize joint kinematics/dynamics to 0.
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    zero_values.insert(WrenchKey(joint->parentID(), j, t), sampler.sample());
    zero_values.insert(WrenchKey(joint->childID(), j, t), sampler.sample());
    std::vector<DynamicsSymbol> keys = {TorqueKey(j, t), JointAngleKey(j, t),
                                        JointVelKey(j, t), JointAccelKey(j, t)};
    for (size_t i = 0; i < keys.size(); i++)
      zero_values.insert(keys[i], sampler.sample()[0]);
  }

  if (contact_points) {
    for (auto&& contact_point : *contact_points) {
      int link_id = -1;
      for (auto& link : robot.links()) {
        if (link->name() == contact_point.first) link_id = link->id();
      }

      if (link_id == -1) throw std::runtime_error("Link not found.");

      zero_values.insert(ContactWrenchKey(link_id, contact_point.second.id, t),
                         sampler.sample());
    }
  }

  return zero_values;
}

Values ZeroValuesTrajectory(
    const Robot& robot, const int num_steps, const int num_phases,
    double gaussian_noise,
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
