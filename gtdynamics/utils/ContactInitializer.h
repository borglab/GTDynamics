#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace gtdynamics::contact_init {

enum class Model { Ground, Patch, Point };
struct Observation {
  std::string stanceId;  // A new ID on every touchdown, even for the same foot.
  std::string foot;
  size_t pointIndex = 0;
  gtsam::Point3 localPoint;  // Known FK, relative to the chosen root frame.
};
struct EventFrame {
  size_t sampleIndex = 0;
  double time = 0;
  std::vector<Observation> observations;
};
struct Options {
  Model model = Model::Ground;
  size_t maxIterations = 100;
  bool perturb = false;
  bool recordHistory = true;
  double relativeErrorTolerance = 1e-8;
  double absoluteErrorTolerance = 1e-12;
  std::optional<gtsam::Pose3> anchorPose;
  // Callbacks permit existing PoseKey(link_id,t) conventions without a Robot
  // dependency. They must produce disjoint keys for poses and landmarks.
  std::function<gtsam::Key(size_t)> poseKey = [](size_t i) { return gtsam::Symbol('x', i); };
  std::function<gtsam::Key(size_t)> landmarkKey = [](size_t i) { return gtsam::Symbol('c', i); };
};
struct Landmark {
  std::string id, stanceId, foot;
  size_t pointIndex = 0;
};
struct IndexedObservation {
  size_t landmarkIndex;
  Observation measurement;
};
struct Graph {
  std::vector<EventFrame> frames;
  std::vector<std::vector<IndexedObservation>> observations;
  std::vector<Landmark> landmarks;
  std::vector<size_t> anchors;
  std::vector<std::vector<size_t>> components;
  std::vector<int> frameRanks;
  std::vector<gtsam::Pose3> seedPoses, initialPoses;
  std::vector<gtsam::Point3> initialLandmarks;
  std::optional<double> groundHeight;
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initialValues;  // Anchors are excluded from optimization.
};
struct Snapshot {
  size_t iteration = 0;
  double objective = 0, rms = 0;
  std::vector<gtsam::Pose3> poses;
  std::vector<gtsam::Point3> landmarks;
};
struct Timings { double graphBuildMs = 0, seedMs = 0, optimizationMs = 0, totalMs = 0; };
struct Result {
  Graph graph;
  gtsam::Values values;  // Includes fixed anchors, for downstream initialization.
  std::vector<gtsam::Pose3> poses;
  std::vector<gtsam::Point3> landmarks;
  std::vector<Snapshot> history;
  Timings timings;
  size_t iterations = 0;
  bool converged = false;
  double objectiveInitial = 0, objectiveFinal = 0, residualRms = 0;
};

/** FK/contact-only alternative to per-frame inverse kinematics.
 *
 * Input contains measured geometry, not floating-base reference poses. The
 * solver changes only root poses and stance landmarks. It cannot infer contact
 * from joint angles, recover a global placement without a gauge choice, or make
 * point-contact attitudes observable. No motion or attitude priors are added.
 */
class ContactInitializer {
 public:
  Result Initialize(const std::vector<EventFrame>& frames, const Options& options = {}) const;
  gtsam::Values InitializeSolutionContactFrames(const std::vector<EventFrame>& frames,
                                               const Options& options = {}) const {
    return Initialize(frames, options).values;
  }
};

int ContactPoseRank(const std::vector<gtsam::Point3>& points);
gtsam::Pose3 FitRigidTransform(const std::vector<gtsam::Point3>& local,
                             const std::vector<gtsam::Point3>& world,
                             const gtsam::Pose3& previous = {});
}  // namespace gtdynamics::contact_init
