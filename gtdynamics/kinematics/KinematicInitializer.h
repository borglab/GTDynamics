/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file KinematicInitializer.h
 * @brief Immutable FK, interpolation, and inverse-kinematics initialization.
 * @author Alejandro Escontrela, Yetong Zhang, GTDynamics contributors
 */
#pragma once
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/linear/Sampler.h>
namespace gtdynamics {
class Trajectory;
/** FK at a constant root COM-to-world pose with measured joint angles.
 * Returns only poses, twists, joint angles and joint velocities. Velocities
 * start at zero; measurements and the robot model are never modified.
 */
class ForwardKinematicsInitializer {
 public:
  /// Bind the fixed robot model for measured-joint forward kinematics.
  explicit ForwardKinematicsInitializer(const Robot& robot);

  /** Evaluate exactly the supplied source slices, without interpolation. */
  gtsam::Values run(const std::string& rootLinkName,
                    const gtsam::Pose3& initialPose,
                    const gtsam::Values& measuredJoints,
                    const std::vector<Slice>& slices) const;

 private:
  const Robot robot_;
};
using InitializationIterationCallback =
    std::function<void(size_t, size_t, double, const gtsam::Values&)>;
/** Internal interpolation/noise utility shared by initialization strategies. */
gtsam::Pose3 addGaussianNoiseToPose(const gtsam::Pose3& T,
                                    const gtsam::Sampler& sampler);

/** Internal interpolation/noise utility shared by initialization strategies. */
std::vector<gtsam::Pose3> interpolatePoses(
    const gtsam::Pose3& wTl_i, const std::vector<gtsam::Pose3>& wTl_t,
    double t_i, const std::vector<double>& timesteps, double dt);

/** Internal interpolation/noise utility shared by initialization strategies. */
gtsam::Values initializePosesAndJoints(const Robot& robot,
                                       const gtsam::Pose3& wTl_i,
                                       const std::vector<gtsam::Pose3>& wTl_t,
                                       const std::string& link_name, double t_i,
                                       const std::vector<double>& timesteps,
                                       double dt, const gtsam::Sampler& sampler,
                                       std::vector<gtsam::Pose3>* wTl_dt);

class InterpolationInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit InterpolationInitializer(const Robot& robot,
                                    double gaussianNoise = 0.0);

  /**
   * @fn Initialize solution via linear interpolation of initial and final pose.
   *
   * @param[in] linkName       The name of the link whose pose to interpolate.
   * @param[in] initialPose           The initial pose of the link.
   * @param[in] finalPose           The final pose of the link.
   * @param[in] startTime             Time at which to start interpolation.
   * @param[in] endTime             Time at which to end interpolation.
   * @param[in] dt              The duration of a single timestep.
   * @param[in] contactPoints  PointOnLink objects.
   * @return Initial solution stored in gtsam::Values object.
   */
  gtsam::Values run(
      const std::string& linkName, const gtsam::Pose3& initialPose,
      const gtsam::Pose3& finalPose, double startTime, double endTime,
      double dt, const std::optional<PointOnLinks>& contactPoints = {}) const;

 private:
  const Robot robot_;
  const double gaussianNoise_;
};
class MultiPhaseInterpolationInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit MultiPhaseInterpolationInitializer(const Robot& robot,
                                              double gaussianNoise = 0.0);

  /**
   * @fn Initialize interpolated solution for multiple phases.
   *
   * @param[in] linkName       The name of the link whose pose to interpolate.
   * @param[in] initialPose           The initial pose of the link.
   * @param[in] targetPoses           A vector of desired poses.
   * @param[in] times              Times at which poses start and end.
   * @param[in] dt              The duration of a single timestep.
   * @param[in] contactPoints  PointOnLink objects.
   * @return Initial solution stored in gtsam::Values object.
   */
  gtsam::Values run(
      const std::string& linkName, const gtsam::Pose3& initialPose,
      const std::vector<gtsam::Pose3>& targetPoses,
      const std::vector<double>& times, double dt,
      const std::optional<PointOnLinks>& contactPoints = {}) const;

 private:
  const Robot robot_;
  const double gaussianNoise_;
};
class InverseKinematicsInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit InverseKinematicsInitializer(const Robot& robot,
                                        double gaussianNoise = 1e-8);

  /**
   * @fn Iteratively solve for the robot kinematics with contacts.
   *
   * @param[in] linkName       The name of the link whose pose to interpolate.
   * @param[in] initialPose           The initial pose of the link.
   * @param[in] targetPoses           A vector of desired poses.
   * @param[in] times              Times at which poses start and end.
   * @param[in] dt              The duration of a single timestep.
   * @param[in] contactPoints  PointOnLink objects.
   * @param[in] iterationCallback Optional observer of each slice's seed and
   * completed LM iterations: (sample index, iteration, GTSAM error, values).
   * @return Initial solution stored in gtsam::Values object.
   */
  gtsam::Values run(
      const std::string& linkName, const gtsam::Pose3& initialPose,
      const std::vector<gtsam::Pose3>& targetPoses,
      const std::vector<double>& times, double dt,
      const std::optional<PointOnLinks>& contactPoints = {},
      const InitializationIterationCallback& iterationCallback = {}) const;

 private:
  const Robot robot_;
  const double gaussianNoise_;
  // Preserve the established IK factor weights.
  const Kinematics kinematics_{
      KinematicsParameters(.001, .001, .001, .00001, .001, .00001, .001, .001)};
};
class MultiPhaseIKInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit MultiPhaseIKInitializer(const Robot& robot,
                                   double gaussianNoise = 1e-8);

  /**
   * @fn Multi-phase initialize solution inverse kinematics.
   *
   * @param[in] linkName             The name of the link whose pose to
   * interpolate.
   * @param[in] phaseSteps           Number of steps for each phase.
   * @param[in] initialPose                 The initial pose of the link.
   * @param[in] targetPoses                 A vector of desired poses.
   * @param[in] times                    Discretized times at which poses start
   * and end.
   * @param[in] transitionValues Initial values for transition graphs.
   * @param[in] dt                  Initial value for phase duration.
   * @param[in] phaseContactPoints  Contact points at each phase.
   * @return Initial solution stored in gtsam::Values object.
   */
  gtsam::Values run(const std::string& linkName,
                    const std::vector<int>& phaseSteps,
                    const gtsam::Pose3& initialPose,
                    const std::vector<gtsam::Pose3>& targetPoses,
                    const std::vector<double>& times,
                    std::vector<gtsam::Values> transitionValues,
                    double dt = 1. / 240,
                    const std::optional<std::vector<PointOnLinks>>&
                        phaseContactPoints = {}) const;

 private:
  const Robot robot_;
  const double gaussianNoise_;
  // Preserve the established IK factor weights.
  const Kinematics kinematics_{
      KinematicsParameters(.001, .001, .001, .00001, .001, .00001, .001, .001)};
};
}  // namespace gtdynamics
