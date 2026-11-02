/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/** A floating-base configuration and velocity at one query time.
 * basePose maps the base link's inertial COM frame into world coordinates.
 * baseTwist is [angular; linear], expressed at that COM in its own axes, in
 * rad/s and m/s. Joint coordinates/rates follow jointNames(), in rad/rad/s
 * for revolute joints and m/m/s for prismatic joints.
 */
struct FloatingBaseState {
  gtsam::Pose3 basePose;
  gtsam::Vector6 baseTwist;
  gtsam::Vector jointAngles;
  gtsam::Vector jointVelocities;
};

/** Estimate velocities and interpolate a fixed floating-base trajectory.
 *
 * One Pose3 white-noise-on-acceleration (WNOA) process models the base; one
 * runtime-sized Vector WNOA process models all scalar joints. Knot poses and
 * joint coordinates are EXACT inputs, not optimization variables. Dense joint
 * observations constrain the conditional GP mean between knots. With fixed
 * configurations the velocity problem is Gaussian and is solved once.
 *
 * Contact constraints and accelerations are deliberately separate from this
 * interpolator. A GP fit does not imply stationary feet. The caller supplies
 * unwrapped joint coordinates when crossing an angular branch cut.
 */
class FloatingBaseInterpolator {
 public:
  /** Construct for a connected tree with a free base and scalar joints.
   * Spectral densities are positive diagonal acceleration PSDs: base entries
   * are [angular; linear], in rad^2/s^3 and m^2/s^3. Joint PSDs and measurement
   * sigmas follow jointNames(), in the corresponding joint coordinate units.
   * Specify rotational standard deviations in degrees in calling code, then
   * convert to radians before forming these PSDs/sigmas. Rejects fixed links,
   * an unknown base, disconnected robots, and invalid noise dimensions.
   */
  FloatingBaseInterpolator(const Robot& robot, const std::string& baseLink,
                           const gtsam::Vector6& baseProcessSpectralDensity,
                           const gtsam::Vector& jointProcessSpectralDensity,
                           const gtsam::Vector& jointMeasurementSigmas);

  /** Append a knot in strictly increasing finite seconds. The base pose uses
   * the COM convention above. Adding input invalidates the previous solve.
   */
  void addKnot(double time, const gtsam::Pose3& basePose,
               const gtsam::Vector& jointAngles);

  /** Add a dense joint observation within the knot time range. Measurements
   * need not be ordered. Interpolated measurement noise includes the WNOA
   * conditional covariance; observations at knots contribute diagnostics but
   * cannot change the fixed knot coordinates. Noise augmentation treats each
   * observation separately, as in GTSAM's WNOA measurement wrappers.
   */
  void addJointMeasurement(double time, const gtsam::Vector& jointAngles);

  /** Observe the base COM's WORLD linear velocity at an existing knot, m/s.
   * The isotropic sigma is in m/s. Angular velocity remains inferred by the GP.
   */
  void addBaseVelocityMeasurement(double time,
                                  const gtsam::Vector3& worldLinearVelocity,
                                  double sigma);

  /** Estimate all knot velocities. Requires at least two knots. No finite
   * differences or pre-fitted splines enter the solve. Reported time covers
   * graph construction, Gaussian elimination, and residual evaluation.
   */
  void solve();

  /** Query the conditional pose/velocity mean inside the closed knot range.
   * Throws before solve or outside the range. Knot configurations are exact.
   */
  FloatingBaseState interpolate(double time) const;

  /** Return native link COM poses/twists and joint coordinates/rates at Slice
   * index k using Robot::forwardKinematics. No acceleration keys are inserted.
   */
  gtsam::Values values(double time, size_t k = 0) const;

  /// Stable joint order, sorted by the robot's joint IDs.
  std::vector<std::string> jointNames() const;
  /// Source joint IDs in the same order, without requiring contiguous IDs.
  std::vector<size_t> jointIds() const;
  /// Original, irregular knot timestamps in seconds.
  std::vector<double> knotTimes() const { return times_; }
  /// Total wall time of the last successful solve, seconds.
  double solveSeconds() const { return solveSeconds_; }
  /// Unweighted RMS dense joint residual in joint coordinate units.
  double jointMeasurementRms() const;
  /// Sum of whitened GP-prior errors, using GTSAM's one-half convention.
  double processError() const;

 private:
  struct JointMeasurement {
    double time;
    gtsam::Vector angles;
  };
  struct VelocityMeasurement {
    double time;
    gtsam::Vector3 velocity;
    double sigma;
  };
  Robot robot_;
  std::string baseLink_;
  std::vector<JointSharedPtr> joints_;
  gtsam::Vector6 basePsd_;
  gtsam::Vector jointPsd_, jointSigmas_;
  std::vector<double> times_;
  std::vector<gtsam::Pose3> poses_;
  std::vector<gtsam::Vector> angles_;
  std::vector<JointMeasurement> jointMeasurements_;
  std::vector<VelocityMeasurement> velocityMeasurements_;
  std::vector<gtsam::Vector6> twists_;
  std::vector<gtsam::Vector> rates_;
  bool solved_ = false;
  double solveSeconds_ = 0, jointRms_ = 0, processError_ = 0;

  size_t interval(double time) const;
  void checkAngles(const gtsam::Vector& angles) const;
};

}  // namespace gtdynamics
