/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <memory>

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

class FloatingBaseInterpolator;

/** A fitted floating-base trajectory with const interpolation queries.
 *
 * Returned only by FloatingBaseInterpolator::solve(). Owns its numerical knot
 * data and fitted velocities, so changing or destroying the input problem does
 * not affect this result. The fixed robot model is shared for native FK; its
 * links, joints, IDs, and geometry must remain unchanged while results exist.
 * The result retains no measurements or solver graph.
 */
class FloatingBaseInterpolation {
 public:
  /** Query the conditional configuration/velocity mean in the closed knot
   * range. Throws outside the range. Knot configurations remain exact.
   */
  FloatingBaseState interpolate(double time) const;

  /** Return native link COM poses/twists and joint coordinates/rates at Slice
   * index k using Robot::forwardKinematics. No acceleration keys are inserted.
   */
  gtsam::Values values(double time, size_t k = 0) const;

  /// Joint names in robot joint-ID order, matching the state vectors.
  std::vector<std::string> jointNames() const;
  /// Source joint IDs in that order, without requiring contiguous IDs.
  std::vector<size_t> jointIds() const;
  /// Original irregular knot timestamps in seconds.
  std::vector<double> knotTimes() const { return times_; }
  /// Accumulated wall time of successful add calls, including factor assembly.
  double graphConstructionSeconds() const { return graphConstructionSeconds_; }
  /// Wall time for elimination, result construction, and residual evaluation.
  double solveSeconds() const { return solveSeconds_; }
  /// Unweighted RMS dense joint residual in joint coordinate units.
  double jointMeasurementRms() const { return jointRms_; }
  /// Sum of whitened GP-prior errors, using GTSAM's one-half convention.
  double processError() const { return processError_; }

 private:
  friend class FloatingBaseInterpolator;
  explicit FloatingBaseInterpolation(const FloatingBaseInterpolator& problem);

  std::shared_ptr<const Robot> robot_;
  std::string baseLink_;
  gtsam::Vector6 basePsd_;
  gtsam::Vector jointPsd_;
  std::vector<double> times_;
  std::vector<gtsam::Pose3> poses_;
  std::vector<gtsam::Vector> angles_;
  std::vector<gtsam::Vector6> twists_;
  std::vector<gtsam::Vector> rates_;
  double graphConstructionSeconds_ = 0;
  double solveSeconds_ = 0, jointRms_ = 0, processError_ = 0;
};

/** Collect inputs for a fixed floating-base trajectory interpolation problem.
 *
 * One Pose3 white-noise-on-acceleration (WNOA) process models the base; one
 * runtime-sized Vector WNOA process models all scalar joints. Knot poses and
 * joint coordinates are EXACT inputs, not optimization variables. Dense joint
 * observations constrain the conditional GP mean between knots. With fixed
 * configurations the velocity problem is Gaussian. Each add call builds its
 * factors immediately; solve() eliminates the accumulated graph without
 * reconstructing factors or retaining another copy of the measurements.
 *
 * Contact constraints and accelerations are separate from this interpolator.
 * A GP fit does not imply stationary feet. The caller supplies unwrapped joint
 * coordinates when crossing an angular branch cut. Joint vectors follow
 * robot.orderedJoints(). The robot's links/joints are shared with the caller;
 * do not modify their topology, IDs, names, or geometry while using this
 * problem or its results. The original Robot object need not remain alive.
 */
class FloatingBaseInterpolator {
 public:
  /** Construct for a connected tree with a free base and scalar joints.
   * Spectral densities are positive diagonal acceleration PSDs: base entries
   * are [angular; linear], in rad^2/s^3 and m^2/s^3. Joint PSDs and measurement
   * sigmas follow robot.orderedJoints(), in joint coordinate units. Specify
   * rotational standard deviations in degrees in calling code, then convert
   * to radians before forming PSDs/sigmas. Rejects fixed links, an unknown
   * base, disconnected robots, and invalid noise dimensions.
   */
  FloatingBaseInterpolator(const Robot& robot, const std::string& baseLink,
                           const gtsam::Vector6& baseProcessSpectralDensity,
                           const gtsam::Vector& jointProcessSpectralDensity,
                           const gtsam::Vector& jointMeasurementSigmas);

  /** Append a knot in strictly increasing finite seconds, using the base COM
   * convention above, and add process factors to the previous knot. Existing
   * solve results are unaffected by added inputs.
   */
  void addKnot(double time, const gtsam::Pose3& basePose,
               const gtsam::Vector& jointAngles);

  /** Add a dense joint observation within the knot time range. Measurements
   * need not be ordered. Interpolated measurement noise includes the WNOA
   * conditional covariance; observations at knots contribute diagnostics but
   * cannot change the fixed knot coordinates. Noise augmentation treats each
   * observation separately, as in GTSAM's WNOA measurement wrappers. Factors
   * are added immediately; raw observation vectors are not retained.
   */
  void addJointMeasurement(double time, const gtsam::Vector& jointAngles);

  /** Observe the base COM's WORLD linear velocity at an existing knot, m/s.
   * Adds its factor immediately. The isotropic sigma is in m/s. Angular
   * velocity remains inferred by the GP.
   */
  void addBaseVelocityMeasurement(double time,
                                  const gtsam::Vector3& worldLinearVelocity,
                                  double sigma);

  /** Solve without modifying the inputs and return an independent result.
   * Requires at least two knots. No finite differences or pre-fitted splines
   * enter the solve. Each result owns its interpolation data and diagnostics;
   * failed or subsequent solves cannot invalidate a previously returned one.
   */
  FloatingBaseInterpolation solve() const;

 private:
  friend class FloatingBaseInterpolation;
  std::shared_ptr<const Robot> robot_;
  std::string baseLink_;
  gtsam::Vector6 basePsd_;
  gtsam::Vector jointPsd_, jointSigmas_;
  std::vector<double> times_;
  std::vector<gtsam::Pose3> poses_;
  std::vector<gtsam::Vector> angles_;
  gtsam::GaussianFactorGraph graph_;
  std::vector<size_t> processFactorIndices_, jointFactorIndices_;
  size_t jointMeasurementCount_ = 0;
  double fixedJointSquaredError_ = 0, graphConstructionSeconds_ = 0;

  void checkAngles(const gtsam::Vector& angles) const;
};

}  // namespace gtdynamics
