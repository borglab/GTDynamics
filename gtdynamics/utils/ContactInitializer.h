#pragma once

/**
 * @file ContactInitializer.h
 * Sparse root-pose initialization from fixed forward-kinematics measurements.
 *
 * Each selected slice supplies contact-point positions on the robot's links,
 * expressed in the root frame.
 * Repeated observations of the same stance point share one world landmark. The
 * solver estimates root-to-world Pose3 states and those landmarks; joint
 * measurements and contact labels remain fixed. No floating-base reference
 * poses are inputs.
 */

#include <gtdynamics/utils/Slice.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace gtdynamics::contact_init {

/** Landmark parameterization and intended contact geometry.
 *
 * The caller supplies patch or point observations; this enum does not select or
 * discard measurements. Only Ground changes the landmark variable dimension.
 */
enum class Model {
  /// Point2 XY landmarks on a hard plane; requires noncollinear sole points.
  Ground,
  /// Point3 landmarks for a rigid patch of noncollinear contact points.
  Patch,
  /// Point3 landmarks for point contacts, with attitude ambiguities.
  Point
};

/** One FK measurement of a contact point fixed on a link.
 *
 * The pair (stanceId, pointIndex) identifies a shared world landmark. A foot's
 * next touchdown must have a fresh stanceId, even if it touches the same place.
 */
struct Observation {
  /// Unique touchdown ID, shared across frames during this uninterrupted
  /// stance.
  std::string stanceId;
  /// Foot or link label, retained for diagnostics rather than data association.
  std::string foot;
  /// Stable contact-point index within the stance, not a collision centroid
  /// ID.
  size_t pointIndex = 0;
  /// Known contact-point position in the chosen root frame, in metres.
  gtsam::Point3 localPoint;
};

/** Frozen FK contact measurements associated with one selected Slice.
 *
 * Supply measurements in temporal order, at contact boundaries and supplemental
 * samples. Slice::k retains the original source index, independently of the
 * sparse vector ordinal. The adapter supplements gaps longer than one second.
 */
struct ContactMeasurements {
  /// Original source slice, rather than the ordinal in the sparse selection.
  Slice slice;
  /// Source time in seconds; optional motion factors follow vector order.
  double time = 0;
  /// Active stance-point observations, with no duplicate pair within a slice.
  std::vector<Observation> observations;
};

/** Contact model, optional priors, LM settings, and caller key conventions.
 *
 * All optional priors are disabled by default. They add explicit assumptions;
 * LM damping itself does not resolve missing measurement information. Every
 * regularizer sigma must be finite and positive, even when its prior is off.
 */
struct Options {
  /// Select Point3 landmarks or the hard Point2 level-ground parameterization.
  Model model = Model::Ground;
  /// Maximum number of LM iterations; reaching this cap need not be
  /// convergence.
  size_t maxIterations = 100;
  /// Apply deterministic translation perturbations to non-anchor seeds for
  /// demos.
  bool perturb = false;
  /// Save the initial state and each LM iterate; disable for isolated
  /// benchmarks.
  bool recordHistory = true;
  /// Add identity BetweenFactors on adjacent poses and retain only one anchor.
  bool addBetweenFactors = false;
  /// Penalize body Z minus world Z on every pose, leaving yaw and position
  /// free.
  bool addUprightFactors = false;
  /// Penalize each Point3 landmark's height once, leaving XY free.
  bool addGroundFactors = false;
  /// Translation standard deviation of identity motion factors, in metres.
  double betweenTranslationSigma = 10.0;
  /// Rotation standard deviation in Pose3's tangent coordinates, in radians.
  double betweenRotationSigma = 3.141592653589793;
  /// Small-angle upright standard deviation, in radians; default is 15 degrees.
  double uprightSigma = 0.2617993877991494;
  /// Landmark-height standard deviation, in metres.
  double groundSigma = 0.02;
  /** Optional world Z target, in metres, for the soft landmark-height prior.
   *
   * Defaults to the first stance's mean height under the first fixed root pose.
   * A supplied height must agree with the fixed plane in Model::Ground. No
   * reference trajectory determines either height.
   */
  std::optional<double> groundPriorHeight;
  /// Stop when the relative decrease of GTSAM's half-squared cost is this
  /// small.
  double relativeErrorTolerance = 1e-8;
  /// Stop when the absolute decrease of GTSAM's half-squared cost is this
  /// small.
  double absoluteErrorTolerance = 1e-12;
  /** Exact root-to-world pose for the first frame, excluded from optimization.
   *
   * Defaults to zero translation and identity rotation, except that hard-ground
   * mode levels the first sole patch. Other contact components have independent
   * arbitrary gauges unless identity motion factors explicitly connect them.
   */
  std::optional<gtsam::Pose3> anchorPose;
  /** Map a selected-slice ordinal to its Pose3 key, not a source sample index.
   *
   * Keys must be unique and disjoint from landmark keys. The GTDynamics adapter
   * supplies PoseKey(rootLinkId, measurements[i].slice.k) through this
   * callback.
   */
  std::function<gtsam::Key(size_t)> poseKey = [](size_t i) {
    return gtsam::Symbol('x', i);
  };
  /// Map each unique landmark ordinal to a key disjoint from every pose key.
  std::function<gtsam::Key(size_t)> landmarkKey = [](size_t i) {
    return gtsam::Symbol('c', i);
  };
};

/// Identifiers and diagnostic labels for a contact point shared across a
/// stance.
struct Landmark {
  /// Generated stanceId:pointIndex label.
  std::string id;
  /// Touchdown ID used to associate observations.
  std::string stanceId;
  /// Foot or link label copied from the first observation.
  std::string foot;
  /// Contact-point index within this stance.
  size_t pointIndex = 0;
};

/// An FK measurement associated with its ordinal in Graph::landmarks.
struct IndexedObservation {
  /// Index of the shared world landmark.
  size_t landmarkIndex;
  /// Unchanged local FK measurement and stance ID.
  Observation measurement;
};

/** Indexed factor graph, initialization state, and structural diagnostics.
 *
 * All pose vectors follow measurements order; all landmark vectors use landmark
 * ordinals. The physical positions returned for hard-ground landmarks are
 * Point3 values, although their optimized Values entries are Point2.
 */
struct Graph {
  /// Selected input measurements and their source slices, in caller order.
  std::vector<ContactMeasurements> measurements;
  /// Indexed contact-point observations for each frame.
  std::vector<std::vector<IndexedObservation>> observations;
  /// Unique stance points in discovery order.
  std::vector<Landmark> landmarks;
  /// Exactly fixed frame ordinals.
  std::vector<size_t> anchors;
  /// Components after optional motion edges; each starts with its anchor frame.
  std::vector<std::vector<size_t>> components;
  /// Components from shared contacts alone, before adding any motion priors.
  std::vector<std::vector<size_t>> contactComponents;
  /// Per-frame contact rank against known world landmarks, not global graph
  /// rank.
  std::vector<int> frameRanks;
  /// Root-to-world seeds selected using contact error, before perturbations.
  std::vector<gtsam::Pose3> seedPoses;
  /// Seeds after optional demo perturbation.
  std::vector<gtsam::Pose3> initialPoses;
  /// Averaged world landmarks; projected onto Z only in the hard-ground model.
  std::vector<gtsam::Point3> initialLandmarks;
  /// Hard plane height in world metres, present only for Model::Ground.
  std::optional<double> groundHeight;
  /// Soft prior's world Z target; also records an already exact hard-floor
  /// target.
  std::optional<double> groundPriorHeight;
  /// All contact and prior factors.
  gtsam::NonlinearFactorGraph factors;
  /// Unit-sigma geometric residuals.
  gtsam::NonlinearFactorGraph contactFactors;
  /// Optional identity motion costs.
  gtsam::NonlinearFactorGraph betweenFactors;
  /// Includes constant anchor costs.
  gtsam::NonlinearFactorGraph uprightFactors;
  /// One soft factor per Point3 landmark; empty if hard ground already fixes Z.
  gtsam::NonlinearFactorGraph groundFactors;
  /// Optimizer inputs, containing free poses and landmarks but excluding
  /// anchors.
  gtsam::Values initialValues;
};

/// One saved optimizer state, including the exact fixed poses.
struct Snapshot {
  /// Zero denotes the initial state, before LM.
  size_t iteration = 0;
  /// Sum of whitened squared residuals, twice GTSAM error.
  double objective = 0;
  /// RMS Euclidean contact error in metres per point, excluding all prior
  /// costs.
  double rms = 0;
  /// Root-to-world poses, in frame order.
  std::vector<gtsam::Pose3> poses;
  /// World contact-point positions, in metres.
  std::vector<gtsam::Point3> landmarks;
};

/** Native wall-clock durations in milliseconds, excluding caller-side work.
 *
 * FK extraction, dense expansion, file/JSON IO, and reference evaluation happen
 * outside this core. With recordHistory enabled, the optimization scope
 * includes per-iterate history capture. The benchmark adapter disables it for
 * timed runs.
 */
struct Timings {
  /// Index observations and build contacts and priors.
  double graphBuildMs = 0;
  /// Register contact components and seed landmarks.
  double seedMs = 0;
  /// Construct LM and run its iteration loop.
  double optimizationMs = 0;
  /// Whole sparse initializer, including result packing.
  double totalMs = 0;
};

/** Sparse solution, native graph diagnostics, convergence status, and costs.
 *
 * Objectives are sums of whitened squared residuals (twice GTSAM's error).
 * The four factor-family objectives sum to objectiveFinal. Contact RMS remains
 * a metric in metres and excludes all prior penalties. A small residual or an
 * LM stopping criterion does not certify physical accuracy or observability.
 */
struct Result {
  /**
   * Source slices actually used by the sparse solve, in the same order as
   * poses and graph.measurements. Indices are not renumbered. With the
   * trajectory adapter, their times are slice.k * dt; contactBuffer may trim
   * the first/last slices and the intervals need not be uniform. Interpolated
   * dense output slices are excluded. Empty before initialization.
   */
  std::vector<Slice> slices() const {
    std::vector<Slice> selected;
    selected.reserve(graph.measurements.size());
    for (const auto& measurement : graph.measurements)
      selected.push_back(measurement.slice);
    return selected;
  }

  /// Indexed measurements, exact anchors, factors, and seed state.
  Graph graph;
  /// Final free variables with exact anchor poses reinserted for downstream
  /// use.
  gtsam::Values values;
  /// Final root-to-world poses, in frame order.
  std::vector<gtsam::Pose3> poses;
  /// Final world contact-point positions.
  std::vector<gtsam::Point3> landmarks;
  /// Initial state plus each completed iterate; empty when recordHistory is
  /// false.
  std::vector<Snapshot> history;
  /// Phase and total durations in milliseconds.
  Timings timings;
  /// Number of completed LM iterations.
  size_t iterations = 0;
  /// Whether an error-based stopping condition was met.
  bool converged = false;
  /// Complete weighted cost before LM.
  double objectiveInitial = 0;
  /// Complete weighted cost after LM.
  double objectiveFinal = 0;
  /// Contact-only RMS Euclidean distance, in metres.
  double residualRms = 0;
  /// Squared geometric contact residuals.
  double contactObjective = 0;
  /// Whitened identity motion cost.
  double betweenObjective = 0;
  /// Whitened body-up cost, including fixed anchors.
  double uprightObjective = 0;
  /// Whitened landmark-height cost; zero for hard Z.
  double groundObjective = 0;
};

/** FK/contact-only alternative to per-frame inverse kinematics.
 *
 * The core registers frames through their shared stance points. Patch mode
 * also tries centroids computed from the same contact points in each frame and
 * selects the lower-cost seed using the original observations, without adding
 * priors. It fixes one
 * exact pose per contact component, and jointly optimizes free poses and
 * landmarks. Optional identity motion factors connect those components and
 * retain only the first fixed pose. Upright and landmark-height factors do not
 * connect components. These priors express assumptions rather than contact
 * evidence.
 *
 * Input contains measured geometry, not floating-base reference poses. This
 * class does not infer contacts, sample frames, compute joint FK, or expand a
 * dense trajectory. Use Initializer::InitializeSolutionContacts for the robot
 * adapter. Point-contact rotations can remain unobservable despite LM damping.
 */
class ContactInitializer {
 public:
  /** Register and optimize selected measurements, retaining exact pose gauges.
   *
   * Supply slices in temporal order, fixed local measurements in metres, and
   * unique keys through Options. Returns sparse states and optional history.
   *
   * Throws std::invalid_argument for empty input, invalid observation IDs or
   * duplicate stance points within a frame, nonfinite geometry or prior scales,
   * an invalid hard-ground patch, or conflicting hard/soft plane heights.
   * Throws std::runtime_error if an LM iterate has nonfinite or increasing
   * cost; GTSAM can also report linear-solver errors for an unsuitable graph.
   */
  Result Initialize(const std::vector<ContactMeasurements>& measurements,
                    const Options& options = {}) const;

  /** Return Initialize(measurements, options).values with the same gauges and
   * keys.
   *
   * Entries contain sparse root poses and stance landmarks, including fixed
   * poses; this convenience function does not produce dense robot-link states.
   */
  gtsam::Values InitializeSolutionContactFrames(
      const std::vector<ContactMeasurements>& measurements,
      const Options& options = {}) const {
    return Initialize(measurements, options).values;
  }
};

/** Return the pose rank supplied by points with known world correspondences.
 *
 * Returns 0 for no points, 3 for coincident points, 5 for distinct collinear
 * points, and 6 for noncollinear points. This local geometric diagnostic does
 * not establish the rank of a graph with unknown shared landmarks.
 */
int ContactPoseRank(const std::vector<gtsam::Point3>& points);

/** Fit a root-to-world transform to corresponding local and world points.
 *
 * Noncollinear geometry uses SVD registration with reflection correction.
 * Collinear geometry aligns its observed axis while preserving unconstrained
 * rotation through previous. Coincident points retain previous's rotation;
 * translation always matches centroids. Throws std::invalid_argument for empty
 * point sets or mismatched sizes.
 */
gtsam::Pose3 FitRigidTransform(const std::vector<gtsam::Point3>& local,
                               const std::vector<gtsam::Point3>& world,
                               const gtsam::Pose3& previous = {});
}  // namespace gtdynamics::contact_init
