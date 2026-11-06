/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ContactInterpolationInitializer.h
 * @brief Initialize poses and velocities from contact poses and a Gaussian
 * process.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtdynamics/kinematics/PoseContactInitializer.h>
#include <gtdynamics/utils/FloatingBaseInterpolator.h>
namespace gtdynamics {
/** Construction settings copied by ContactInterpolationInitializer.
 * Angular scales are standard deviations in degrees; squared radian values
 * define the GP acceleration spectral densities (rad^2/s^3).
 */
struct ContactInterpolationOptions {
  pose_contact_init::Options contact;
  size_t contactBuffer = 0;
  double baseAngularProcessSigmaDegrees = 90.0;
  double baseLinearProcessSigma = 3.0;
  double jointProcessSigmaDegrees = 100.0;
  double jointMeasurementSigmaDegrees = 0.5;
};
/** Owning kinematic result. No acceleration, force, or torque keys are added.
 * The interpolation shares the fixed robot topology, which must remain
 * unchanged. contact retains original source indices and sparse diagnostics.
 * contactSeconds includes sparse contact output; interpolationSeconds includes
 * GP assembly, elimination and evaluation at requested slices. Times are wall
 * seconds.
 */
struct ContactInterpolationResult {
  gtsam::Values values;
  pose_contact_init::Result contact;
  std::shared_ptr<FloatingBaseInterpolation> interpolation;
  double contactSeconds = 0;
  double interpolationSeconds = 0;
};
/** Immutable contact-pose fitting followed by the existing floating-base GP.
 * Each run constructs independent solvers and leaves input Values unchanged.
 */
class ContactInterpolationInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit ContactInterpolationInitializer(
      const Robot& robot, const ContactInterpolationOptions& options = {});

  /** Source slices must cover the trajectory in increasing order; times are
   * their strictly increasing timestamps in seconds. Contact sampling uses dt
   * (the original uniform source period), not the sparse knot spacing.
   * Buffered contact poses form GP knots; boundary slices outside that span
   * use the nearest fitted root pose and measured joints as extra knots, with
   * no extrapolation. Dense encoders constrain the GP between knots. Root
   * poses map COM to world, and output twists are body-fixed at each COM.
   * selectedSlices optionally supplies this recording's explicit contact knots;
   * an empty vector uses the configured sampling policy.
   */
  ContactInterpolationResult run(
      const std::string& rootLinkName, const gtsam::Pose3& initialPose,
      const gtsam::Values& measuredJoints, const Trajectory& trajectory,
      const std::vector<Slice>& slices, const std::vector<double>& times,
      double dt, const std::vector<Slice>& selectedSlices = {}) const;

 private:
  const Robot robot_;
  const ContactInterpolationOptions options_;
};
}  // namespace gtdynamics
