/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file PoseContactInitializer.h
 * @brief Pose-contact initialization and shared phase sampling.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtdynamics/kinematics/ContactInitializer.h>
#include <gtdynamics/kinematics/KinematicInitializer.h>
#include <gtdynamics/utils/PoseOnLink.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <map>

namespace gtdynamics {
class Trajectory;
/** Internal stance association shared by pose graph builders and
 * initialization. Identity follows a material frame through adjacent phases; a
 * lift-off creates a fresh stance. Shared phase endpoints contain the union of
 * active frames. */
struct TrackedPoseContact {
  PoseOnLink contact;
  size_t stance;
};
std::map<size_t, std::vector<TrackedPoseContact>> contactPoseFrames(
    const Trajectory& trajectory);

/** Select original slices from every phase with at least one pose contact.
 * Short phases (sample count * dt <= min(longPhaseSeconds, maxSliceInterval))
 * contribute their central slice, choosing the earlier of two central samples.
 * Long phases retain inward-buffered endpoints and evenly spaced slices between
 * them, with gaps <= maxSliceInterval seconds. The buffer is capped to retain
 * short phases. The interval bound applies within each phase's buffered
 * interior; unsupported phases and buffered boundaries can create larger gaps.
 * Returns ordered unique source indices; stance association uses the original
 * trajectory. Requires finite dt > 0, maxSliceInterval >= dt, longPhaseSeconds
 * > 0.
 */
std::vector<Slice> contactInitializationSlices(const Trajectory& trajectory,
                                               double dt,
                                               double maxSliceInterval = 0.2,
                                               size_t contactBuffer = 1,
                                               double longPhaseSeconds = 0.2);

namespace pose_contact_init {
/** Contact-pose observation noise: radians and metres. Priors are explicit soft
 * assumptions. The first selected root pose is an exact gauge, even if
 * buffering removes the recording's first slice. No reference poses enter the
 * solve. */
struct Options {
  size_t maxIterations = 100;
  bool recordHistory = true;
  /// Maximum automatic sampling interval in seconds, bounded below by dt.
  double maxSliceInterval = 0.2;
  /// False returns FK Values ONLY at selected slices, without interpolation.
  bool interpolateOutput = true;
  /// Starting values only: propagate relative yaw and contact translation,
  /// preserving the anchor's tilt instead of accumulating sole rocking.
  /// Independent of addUprightFactors; the optimized poses remain full Pose3.
  bool initializeUpright = false;
  bool addBetweenFactors = false;
  /// Reuse the point initializer's world-up prior on each free root COM pose.
  bool addUprightFactors = false;
  /// Small-angle scale: 15 degrees, converted to radians for UprightFactor.
  double uprightSigma = 15.0 * 3.141592653589793 / 180.0;
  double rotationSigma = 0.05;
  double translationSigma = 0.01;
  double betweenTranslationSigma = 10.0;
  double betweenRotationSigma = 3.141592653589793;
  bool addGroundFactors = true;
  /// With ground factors enabled, also align sole normals. Disable for height
  /// only.
  bool addNormalFactors = true;
  double groundSigma = 0.01;
  double normalSigma = 0.05;
  std::optional<double> groundHeight;
};
/** One sparse LM iterate. Wrenches are not estimated. Contact poses are world
 * material-frame poses in stance-discovery order. RMS is translation in metres.
 */
struct Snapshot {
  size_t iteration = 0;
  double objective = 0;
  double rms = 0;
  std::vector<gtsam::Pose3> poses;
  std::vector<gtsam::Pose3> contactPoses;
};
/** A frozen FK observation. Indices address Result::poses/contactPoses;
 * rootTcontact maps the material contact frame into the root COM frame.
 * Exposed for graph inspection without reconstructing stance association. */
struct Observation {
  size_t poseIndex = 0;
  size_t contactIndex = 0;
  gtsam::Pose3 rootTcontact;
};
/** Sparse diagnostics; source indices/times are retained, never renumbered.
 * Dense interpolation performed by PoseContactInitializer is excluded from
 * these slices. Objectives are twice GTSAM error; rotational RMS is reported
 * separately. */
struct Result {
  std::vector<Slice> selectedSlices;
  std::vector<double> times;
  std::vector<gtsam::Pose3> poses;
  std::vector<gtsam::Pose3> contactPoses;
  std::vector<Snapshot> history;
  std::vector<Observation> observations;
  std::vector<PoseOnLink> contactFrames;
  gtsam::Values values;
  gtsam::NonlinearFactorGraph
      graph;  ///< Sparse graph; first root is held fixed.
  contact_init::Timings timings;
  size_t iterations = 0;
  bool converged = false;
  /// LM returned no decrease with a nonzero gradient; not convergence.
  bool stalled = false;
  double objectiveInitial = 0;
  double objectiveFinal = 0;
  double residualRms = 0;
  double rotationRms = 0;
  std::vector<Slice> slices() const { return selectedSlices; }
};
}  // namespace pose_contact_init

class PoseContactInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit PoseContactInitializer(
      const Robot& robot, const pose_contact_init::Options& options = {},
      size_t contactBuffer = 0);

  /** Initialize from FK contact poses and phase association. Use
   * contactInitializationSlices with phase buffering; return selected indices
   * in diagnostics. dt is the uniform source period; selected intervals may
   * vary. Callers can supply explicit selectedSlices (with contactBuffer=0) and
   * disable dense interpolation via options.interpolateOutput=false. Stance IDs
   * always come from the original complete trajectory, including unselected
   * phases. An empty selectedSlices input uses the configured sampling policy;
   * explicit slices must be ordered and require contactBuffer=0. Disconnected
   * support requires explicit motion priors or is rejected. */
  gtsam::Values run(const std::string& rootLinkName,
                    const gtsam::Pose3& initialPose,
                    const gtsam::Values& measuredJoints,
                    const Trajectory& trajectory, double dt = 1.0,
                    pose_contact_init::Result* diagnostics = nullptr,
                    const std::vector<Slice>& selectedSlices = {}) const;

 private:
  const Robot robot_;
  const pose_contact_init::Options options_;
  const size_t contactBuffer_;
};
class PoseIKInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit PoseIKInitializer(const Robot& robot, size_t maxIterations = 10);

  /** Per-slice IK using the same material contact frames as the contact
   * initializer. rootGuesses follows trajectory slices and sets root priors;
   * measured joint angles supply soft priors. Each stance target is frozen at
   * its first FK observation. The callback receives global slice keys. */
  gtsam::Values run(
      const std::string& rootLinkName,
      const std::vector<gtsam::Pose3>& rootGuesses,
      const gtsam::Values& measuredJoints, const Trajectory& trajectory,
      const InitializationIterationCallback& iterationCallback = nullptr) const;

 private:
  const Robot robot_;
  const size_t maxIterations_;
};
}  // namespace gtdynamics
