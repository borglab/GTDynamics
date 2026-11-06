/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file KinematicInitializer.cpp
 * @brief Run configuration and velocity initialization strategies.
 * @author Alejandro Escontrela, Yetong Zhang, GTDynamics contributors
 */
#include <gtdynamics/kinematics/KinematicInitializer.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Sampler;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
using gtsam::Vector6;
namespace gtdynamics {
ForwardKinematicsInitializer::ForwardKinematicsInitializer(const Robot& robot)
    : robot_(robot) {}

Values ForwardKinematicsInitializer::run(
    const std::string& rootLinkName, const gtsam::Pose3& initialPose,
    const gtsam::Values& measuredJoints,
    const std::vector<Slice>& slices) const {
  Values result;
  for (const auto& slice : slices) {
    Values known;
    InsertPose(&known, robot_.link(rootLinkName)->id(), slice.k, initialPose);
    InsertTwist(&known, robot_.link(rootLinkName)->id(), slice.k,
                Vector6::Zero());
    for (const auto& joint : robot_.joints()) {
      InsertJointAngle(&known, joint->id(), slice.k,
                       JointAngle(measuredJoints, joint->id(), slice.k));
      InsertJointVel(&known, joint->id(), slice.k, 0.0);
    }
    result.insert(robot_.forwardKinematics(known, slice.k, rootLinkName));
  }
  return result;
}
Pose3 addGaussianNoiseToPose(const Pose3& T, const Sampler& sampler) {
  Vector6 xi = sampler.sample();
  return T.expmap(xi);
}

std::vector<Pose3> interpolatePoses(const Pose3& wTl_i,
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

Values initializePosesAndJoints(const Robot& robot, const Pose3& wTl_i,
                                const std::vector<Pose3>& wTl_t,
                                const std::string& link_name, double t_i,
                                const std::vector<double>& timesteps, double dt,
                                const Sampler& sampler,
                                std::vector<Pose3>* wTl_dt) {
  // Linearly interpolated pose for link at each discretized timestep.
  *wTl_dt = interpolatePoses(wTl_i, wTl_t, t_i, timesteps, dt);

  for (auto& pose : *wTl_dt) {
    pose = addGaussianNoiseToPose(pose, sampler);
  }

  // TODO(frank): I'm not really understanding all this. Refactor in future?
  Values values, fk_input;

  // Initial pose and joint angles are known a priori.
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&fk_input, joint->id(), sampler.sample()[0]);
    InsertJointVel(&fk_input, joint->id(), sampler.sample()[0]);
    InsertJointAngle(&values, joint->id(), 0, sampler.sample()[0]);
  }

  // Compute forward dynamics to obtain remaining link poses.
  Pose3 wTl_i_processed = addGaussianNoiseToPose(wTl_i, sampler);
  InsertPose(&fk_input, robot.link(link_name)->id(), wTl_i_processed);
  InsertTwist(&fk_input, robot.link(link_name)->id(), gtsam::Z_6x1);
  auto fk_results = robot.forwardKinematics(fk_input, 0, link_name);

  for (auto&& link : robot.links()) {
    size_t i = link->id();
    InsertPose(&values, i, Pose(fk_results, i));
  }
  return values;
}

InterpolationInitializer::InterpolationInitializer(const Robot& robot,
                                                   double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values InterpolationInitializer::run(
    const std::string& linkName, const gtsam::Pose3& initialPose,
    const gtsam::Pose3& finalPose, double startTime, double endTime, double dt,
    const std::optional<PointOnLinks>& contactPoints) const {
  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussianNoise_);
  Sampler sampler(sampler_noise_model);

  // Initial and final discretized timesteps.
  int n_steps_init = std::lround(startTime / dt);
  int n_steps_final = std::lround(endTime / dt);

  double t_elapsed = startTime;

  Values init_vals;

  // Initialize joint angles and velocities to 0.
  for (int t = n_steps_init; t <= n_steps_final; t++) {
    double s = (t_elapsed - startTime) / (endTime - startTime);

    // Compute interpolated pose for link.
    Pose3 wTl_t = addGaussianNoiseToPose(
        gtsam::interpolate<Pose3>(initialPose, finalPose, s), sampler);

    // Compute forward dynamics to obtain remaining link poses.
    // TODO(Alejandro): forwardKinematics needs to get passed prev link twist
    // Keep FK input local to this timestep to avoid copying the whole
    // trajectory.
    Values fk_input;
    for (auto&& joint : robot_.joints()) {
      InsertJointAngle(&fk_input, joint->id(), t, sampler.sample()[0]);
      InsertJointVel(&fk_input, joint->id(), t, sampler.sample()[0]);
    }

    auto link = robot_.link(linkName);
    if (link->isFixed()) {
      throw std::invalid_argument("InitializeSolutionInterpolation: Link " +
                                  linkName + " is fixed.");
    }
    InsertPose(&fk_input, link->id(), t, wTl_t);
    InsertTwist(&fk_input, link->id(), t, gtsam::Z_6x1);
    init_vals.insert(robot_.forwardKinematics(fk_input, t, linkName));

    t_elapsed += dt;
  }

  return init_vals;
}

MultiPhaseInterpolationInitializer::MultiPhaseInterpolationInitializer(
    const Robot& robot, double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values MultiPhaseInterpolationInitializer::run(
    const std::string& linkName, const gtsam::Pose3& initialPose,
    const std::vector<gtsam::Pose3>& targetPoses,
    const std::vector<double>& times, double dt,
    const std::optional<PointOnLinks>& contactPoints) const {
  Values init_vals;
  Pose3 pose = initialPose;
  double curr_t = 0.0;
  for (size_t i = 0; i < targetPoses.size(); i++) {
    Values phase_vals = InterpolationInitializer(robot_, gaussianNoise_)
                            .run(linkName, pose, targetPoses[i], curr_t,
                                 times[i], dt, contactPoints);

    for (auto&& key : phase_vals.keys()) {
      if (!init_vals.exists(key)) {
        init_vals.insert(key, phase_vals.at(key));
      }
    }
    pose = targetPoses[i];
    curr_t = times[i];
  }
  return init_vals;
}

InverseKinematicsInitializer::InverseKinematicsInitializer(const Robot& robot,
                                                           double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values InverseKinematicsInitializer::run(
    const std::string& linkName, const gtsam::Pose3& initialPose,
    const std::vector<gtsam::Pose3>& targetPoses,
    const std::vector<double>& times, double dt,
    const std::optional<PointOnLinks>& contactPoints,
    const InitializationIterationCallback& iterationCallback) const {
  double t_i = 0.0;  // Time elapsed.

  Vector3 gravity(0, 0, -9.8);

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussianNoise_);
  Sampler sampler(sampler_noise_model);

  // Link pose at each step
  std::vector<Pose3> wTl_dt;

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  Values init_vals, values = initializePosesAndJoints(
                        robot_, initialPose, targetPoses, linkName, t_i, times,
                        dt, sampler, &wTl_dt);

  for (int t = 0; t <= std::round(times[times.size() - 1] / dt); t++) {
    const Slice slice(t);
    gtsam::NonlinearFactorGraph kfg =
        kinematics_.qFactors(slice, robot_, contactPoints, gravity);
    kfg.addPrior(PoseKey(robot_.link(linkName)->id(), t), wTl_dt[t],
                 gtsam::noiseModel::Isotropic::Sigma(6, 0.001));

    gtsam::LevenbergMarquardtParams params;
    gtsam::LevenbergMarquardtOptimizer* active_optimizer = nullptr;
    if (iterationCallback) {
      params.iterationHook = [&](size_t iteration, double, double error) {
        iterationCallback(t, iteration, error, active_optimizer->values());
      };
    }
    gtsam::LevenbergMarquardtOptimizer optimizer(kfg, values, params);
    active_optimizer = &optimizer;
    if (iterationCallback)
      iterationCallback(t, 0, optimizer.error(), optimizer.values());
    Values results = optimizer.optimize();

    // Add zero initial values for remaining variables.
    Values kinematics;
    for (const auto& link : robot_.links()) {
      InsertPose(&kinematics, link->id(), t, link->bMcom());
      InsertTwist(&kinematics, link->id(), t, Vector6::Zero());
    }
    for (const auto& joint : robot_.joints()) {
      InsertJointAngle(&kinematics, joint->id(), t, 0.0);
      InsertJointVel(&kinematics, joint->id(), t, 0.0);
    }
    init_vals.insert(kinematics);

    // Update with the results of the optimizer.
    init_vals.update(results);

    // Update initial values for next timestep.
    values.clear();

    for (auto&& link : robot_.links()) {
      InsertPose(&values, link->id(), t + 1, Pose(results, link->id(), t));
    }
    for (auto&& joint : robot_.joints()) {
      InsertJointAngle(&values, joint->id(), t + 1,
                       JointAngle(results, joint->id(), t));
    }
  }

  return init_vals;
}

MultiPhaseIKInitializer::MultiPhaseIKInitializer(const Robot& robot,
                                                 double gaussianNoise)
    : robot_(robot), gaussianNoise_(gaussianNoise) {}

Values MultiPhaseIKInitializer::run(
    const std::string& linkName, const std::vector<int>& phaseSteps,
    const gtsam::Pose3& initialPose,
    const std::vector<gtsam::Pose3>& targetPoses,
    const std::vector<double>& times,
    std::vector<gtsam::Values> transitionValues, double dt,
    const std::optional<std::vector<PointOnLinks>>& phaseContactPoints) const {
  double t_i = 0;  // Time elapsed.
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussianNoise_);
  Sampler sampler(sampler_noise_model);

  std::vector<Pose3> wTl_dt;

  // Iteratively solve the inverse kinematics problem while statisfying
  // the contact pose constraint.
  Values init_vals, values = initializePosesAndJoints(
                        robot_, initialPose, targetPoses, linkName, t_i, times,
                        dt, sampler, &wTl_dt);

  int t = 0;
  int num_phases = phaseSteps.size();

  for (int phase = 0; phase < num_phases; phase++) {
    // In-phase.
    int curr_phase_steps =
        phase == (num_phases - 1) ? phaseSteps[phase] + 1 : phaseSteps[phase];
    for (int phase_step = 0; phase_step < curr_phase_steps; phase_step++) {
      const Slice slice(t);
      gtsam::NonlinearFactorGraph kfg = kinematics_.qFactors(
          slice, robot_, (*phaseContactPoints)[phase], gravity);

      kfg.addPrior(PoseKey(robot_.link(linkName)->id(), t), wTl_dt[t],
                   gtsam::noiseModel::Isotropic::Sigma(6, 0.001));

      gtsam::LevenbergMarquardtOptimizer optimizer(kfg, values);
      Values results = optimizer.optimize();

      init_vals.insert(results);

      // Update initial values for next timestep.
      values.clear();
      for (auto&& link : robot_.links()) {
        int link_id = link->id();
        InsertPose(&values, link_id, t + 1, Pose(results, link_id, t));
      }

      for (auto&& joint : robot_.joints()) {
        int joint_id = joint->id();
        InsertJointAngle(&values, joint_id, t + 1,
                         JointAngle(results, joint_id, t));
      }
      t++;
    }
  }

  // Complete kinematic velocities and timing keys, without dynamics variables.
  for (int k = 0; k < t; ++k) {
    for (const auto& link : robot_.links())
      InsertTwist(&init_vals, link->id(), k, Vector6::Zero());
    for (const auto& joint : robot_.joints())
      InsertJointVel(&init_vals, joint->id(), k, 0.0);
  }
  for (int phase = 0; phase < num_phases; ++phase)
    init_vals.insert(PhaseKey(phase), dt);
  for (const auto& transition : transitionValues)
    for (gtsam::Key key : transition.keys()) {
      const auto label = DynamicsSymbol(key).label();
      if ((label == "q" || label == "p" || label == "v" || label == "V") &&
          !init_vals.exists(key))
        init_vals.insert(key, transition.at(key));
    }
  return init_vals;
}

}  // namespace gtdynamics
