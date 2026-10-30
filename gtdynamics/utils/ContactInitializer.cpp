#include "ContactInitializer.h"
#include "ContactInitializationFactors.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace gtdynamics::contact_init {
namespace {
using Clock = std::chrono::steady_clock;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
double elapsed(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }
Point3 centroid(const std::vector<Point3>& points) {
  Point3 result = Point3::Zero();
  for (const auto& point : points) result += point;
  return result / static_cast<double>(points.size());
}
Rot3 align(const Point3& a, const Point3& b) {
  if (a.norm() < 1e-12 || b.norm() < 1e-12) return Rot3();
  return Rot3(Eigen::Quaterniond::FromTwoVectors(a, b));
}
Rot3 levelRotation(const EventFrame& frame) {
  if (frame.observations.empty()) throw std::invalid_argument("Level ground requires supported event frames.");
  const std::string& id = frame.observations.front().stanceId;
  std::vector<Point3> points;
  for (const auto& o : frame.observations) if (o.stanceId == id) points.push_back(o.localPoint);
  if (points.size() < 3 || ContactPoseRank(points) != 6) throw std::invalid_argument("Level ground requires a noncollinear sole patch.");
  Point3 normal = (points[1] - points[0]).cross(points[2] - points[0]).normalized();
  if (normal.z() < 0) normal = -normal;
  return align(normal, Point3::UnitZ());
}
Point3 projected(const Graph& graph, const Point3& point) {
  Point3 result = point;
  if (graph.groundHeight) result.z() = *graph.groundHeight;
  return result;
}
std::vector<Point3> averageLandmarks(const Graph& graph, const std::vector<Pose3>& poses) {
  std::vector<Point3> result(graph.landmarks.size(), Point3::Zero());
  std::vector<size_t> counts(result.size(), 0);
  for (size_t i = 0; i < graph.frames.size(); ++i) for (const auto& o : graph.observations[i]) {
    result[o.landmarkIndex] += poses[i].transformFrom(o.measurement.localPoint);
    ++counts[o.landmarkIndex];
  }
  for (size_t i = 0; i < result.size(); ++i) result[i] = projected(graph, result[i] / static_cast<double>(counts[i]));
  return result;
}
void indexObservations(Graph& graph) {
  std::map<std::pair<std::string, size_t>, size_t> indices;
  for (const auto& frame : graph.frames) {
    std::vector<IndexedObservation> observations;
    std::vector<Point3> points;
    std::set<std::pair<std::string, size_t>> seen;
    for (const auto& o : frame.observations) {
      if (o.stanceId.empty() || !o.localPoint.allFinite()) throw std::invalid_argument("Each observation needs a stance ID and finite local FK point.");
      const auto key = std::make_pair(o.stanceId, o.pointIndex);
      if (!seen.insert(key).second) throw std::invalid_argument("Duplicate stance point in an event frame.");
      const auto [iterator, inserted] = indices.emplace(key, graph.landmarks.size());
      if (inserted) graph.landmarks.push_back({o.stanceId + ":" + std::to_string(o.pointIndex), o.stanceId, o.foot, o.pointIndex});
      observations.push_back({iterator->second, o});
      points.push_back(o.localPoint);
    }
    graph.observations.push_back(std::move(observations));
    graph.frameRanks.push_back(ContactPoseRank(points));
  }
}
void seed(Graph& graph, const Options& options) {
  const size_t n = graph.frames.size();
  graph.seedPoses.resize(n);
  std::vector<bool> assigned(n, false), known(graph.landmarks.size(), false);
  std::vector<Point3> landmarks(graph.landmarks.size(), Point3::Zero());
  if (options.model == Model::Ground) {
    const auto level = levelRotation(graph.frames[0]);  // Validate even with a supplied anchor.
    const auto rotation = options.anchorPose ? options.anchorPose->rotation() : level;
    const Pose3 anchor(rotation, options.anchorPose ? options.anchorPose->translation() : Point3::Zero());
    std::vector<Point3> points;
    const auto& id = graph.frames[0].observations.front().stanceId;
    for (const auto& o : graph.frames[0].observations) if (o.stanceId == id) points.push_back(anchor.transformFrom(o.localPoint));
    graph.groundHeight = centroid(points).z();
  }
  for (size_t anchor = 0; anchor < n; ++anchor) {
    if (assigned[anchor]) continue;
    Point3 position(1.2 * static_cast<double>(graph.components.size()), 0, 0);
    Rot3 rotation;
    if (anchor == 0 && options.anchorPose) { rotation = options.anchorPose->rotation(); position = options.anchorPose->translation(); }
    else if (graph.groundHeight) {
      rotation = levelRotation(graph.frames[anchor]);
      std::vector<Point3> points;
      const auto& id = graph.frames[anchor].observations.front().stanceId;
      for (const auto& o : graph.frames[anchor].observations) if (o.stanceId == id) points.push_back(rotation.rotate(o.localPoint));
      position.z() = *graph.groundHeight - centroid(points).z();
    }
    graph.seedPoses[anchor] = Pose3(rotation, position);
    graph.anchors.push_back(anchor);
    graph.components.push_back({anchor});
    assigned[anchor] = true;
    auto introduce = [&](size_t frame) {
      for (const auto& o : graph.observations[frame]) if (!known[o.landmarkIndex]) {
        known[o.landmarkIndex] = true;
        landmarks[o.landmarkIndex] = projected(graph, graph.seedPoses[frame].transformFrom(o.measurement.localPoint));
      }
    };
    introduce(anchor);
    bool changed = true;
    while (changed) {
      changed = false;
      for (size_t i = 0; i < n; ++i) if (!assigned[i]) {
        std::vector<Point3> local, world;
        for (const auto& o : graph.observations[i]) if (known[o.landmarkIndex]) { local.push_back(o.measurement.localPoint); world.push_back(landmarks[o.landmarkIndex]); }
        if (local.empty()) continue;
        graph.seedPoses[i] = FitRigidTransform(local, world);
        assigned[i] = true;
        graph.components.back().push_back(i);
        introduce(i);
        changed = true;
      }
    }
  }
  graph.initialPoses = graph.seedPoses;
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  if (options.perturb) for (size_t i = 0; i < n; ++i) if (!anchors.count(i)) {
    Point3 translation = graph.initialPoses[i].translation();
    for (size_t j = 0; j < 3; ++j) translation[j] += 0.035 * std::sin(i * 2.3 + j * 1.7);
    graph.initialPoses[i] = Pose3(graph.initialPoses[i].rotation(), translation);
  }
  graph.initialLandmarks = averageLandmarks(graph, graph.initialPoses);
}
void buildFactors(Graph& graph, const Options& options) {
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  for (size_t i = 0; i < graph.frames.size(); ++i) if (!anchors.count(i)) graph.initialValues.insert(options.poseKey(i), graph.initialPoses[i]);
  for (size_t i = 0; i < graph.landmarks.size(); ++i) {
    if (graph.groundHeight) graph.initialValues.insert(options.landmarkKey(i), gtsam::Point2(graph.initialLandmarks[i].head<2>()));
    else graph.initialValues.insert(options.landmarkKey(i), graph.initialLandmarks[i]);
  }
  for (size_t i = 0; i < graph.frames.size(); ++i) for (const auto& o : graph.observations[i]) {
    const auto key = options.landmarkKey(o.landmarkIndex);
    if (anchors.count(i)) {
      const auto target = graph.seedPoses[i].transformFrom(o.measurement.localPoint);
      if (graph.groundHeight) graph.factors.emplace_shared<AnchoredContactFactor2>(key, target, *graph.groundHeight);
      else graph.factors.emplace_shared<AnchoredContactFactor3>(key, target);
    } else {
      if (graph.groundHeight) graph.factors.emplace_shared<ContactFactor2>(options.poseKey(i), key, o.measurement.localPoint, *graph.groundHeight);
      else graph.factors.emplace_shared<ContactFactor3>(options.poseKey(i), key, o.measurement.localPoint);
    }
  }
}
Snapshot snapshot(const Graph& graph, const Options& options, const gtsam::Values& values, size_t iteration) {
  Snapshot state;
  state.iteration = iteration;
  state.objective = 2 * graph.factors.error(values);  // GTSAM error includes 1/2.
  size_t count = 0;
  for (const auto& frame : graph.observations) count += frame.size();
  state.rms = std::sqrt(state.objective / std::max<size_t>(1, count));
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  for (size_t i = 0; i < graph.frames.size(); ++i) state.poses.push_back(anchors.count(i) ? graph.seedPoses[i] : values.at<Pose3>(options.poseKey(i)));
  for (size_t i = 0; i < graph.landmarks.size(); ++i) {
    if (graph.groundHeight) { const auto& xy = values.at<gtsam::Point2>(options.landmarkKey(i)); state.landmarks.emplace_back(xy.x(), xy.y(), *graph.groundHeight); }
    else state.landmarks.push_back(values.at<Point3>(options.landmarkKey(i)));
  }
  return state;
}
}  // namespace

int ContactPoseRank(const std::vector<Point3>& points) {
  if (points.empty()) return 0;
  Point3 axis = Point3::Zero();
  for (const auto& p : points) if ((p - points[0]).norm() > 1e-8) { axis = p - points[0]; break; }
  if (axis.norm() == 0) return 3;
  for (const auto& p : points) if (axis.cross(p - points[0]).norm() > 1e-9) return 6;
  return 5;
}

Pose3 FitRigidTransform(const std::vector<Point3>& local, const std::vector<Point3>& world, const Pose3& previous) {
  if (local.empty() || local.size() != world.size()) throw std::invalid_argument("Rigid registration needs matching nonempty point sets.");
  const Point3 a = centroid(local), b = centroid(world);
  const int rank = ContactPoseRank(local);
  Rot3 rotation = previous.rotation();
  if (rank == 5) {
    Point3 axis = Point3::Zero();
    for (const auto& p : local) if ((p - local[0]).norm() > axis.norm()) axis = p - local[0];
    axis.normalize();
    Point3 target = Point3::Zero();
    for (size_t i = 0; i < local.size(); ++i) target += (local[i] - a).dot(axis) * (world[i] - b);
    rotation = align(rotation.rotate(axis), target).compose(rotation);
  } else if (rank == 6) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < local.size(); ++i) H += (local[i] - a) * (world[i] - b).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0 ? -1 : 1;
    rotation = Rot3(svd.matrixV() * correction * svd.matrixU().transpose());
  }
  return Pose3(rotation, b - rotation.rotate(a));
}

Result ContactInitializer::Initialize(const std::vector<EventFrame>& frames, const Options& options) const {
  if (frames.empty()) throw std::invalid_argument("At least one contact event frame is required.");
  Result result;
  const auto totalStart = Clock::now();
  auto phaseStart = Clock::now();
  result.graph.frames = frames;
  indexObservations(result.graph);
  result.timings.graphBuildMs = elapsed(phaseStart);
  phaseStart = Clock::now();
  seed(result.graph, options);
  result.timings.seedMs = elapsed(phaseStart);
  phaseStart = Clock::now();
  buildFactors(result.graph, options);
  result.timings.graphBuildMs += elapsed(phaseStart);
  phaseStart = Clock::now();
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = options.maxIterations;
  params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  params.setDiagonalDamping(true);  // Numerical LM damping, not a pose prior.
  params.lambdaInitial = 1e-3;
  params.relativeErrorTol = options.relativeErrorTolerance;
  params.absoluteErrorTol = options.absoluteErrorTolerance;
  gtsam::LevenbergMarquardtOptimizer optimizer(result.graph.factors, result.graph.initialValues, params);
  result.objectiveInitial = 2 * optimizer.error();
  if (options.recordHistory) result.history.push_back(snapshot(result.graph, options, optimizer.values(), 0));
  for (size_t iteration = 0; iteration < options.maxIterations; ++iteration) {
    const double before = optimizer.error();
    if (before < 1e-20) { result.converged = true; break; }
    optimizer.iterate();
    ++result.iterations;
    const double after = optimizer.error();
    if (!std::isfinite(after) || after > before + 1e-12 * std::max(1.0, before)) throw std::runtime_error("Native LM objective became non-finite or increased.");
    if (options.recordHistory) result.history.push_back(snapshot(result.graph, options, optimizer.values(), result.iterations));
    if (before - after <= options.absoluteErrorTolerance || (before - after) / before <= options.relativeErrorTolerance) { result.converged = true; break; }
  }
  result.timings.optimizationMs = elapsed(phaseStart);
  auto final = snapshot(result.graph, options, optimizer.values(), result.iterations);
  result.poses = std::move(final.poses);
  result.landmarks = std::move(final.landmarks);
  result.objectiveFinal = final.objective;
  result.residualRms = final.rms;
  result.values = optimizer.values();
  for (const size_t anchor : result.graph.anchors) result.values.insert(options.poseKey(anchor), result.graph.seedPoses[anchor]);
  result.timings.totalMs = elapsed(totalStart);
  return result;
}
}  // namespace gtdynamics::contact_init
