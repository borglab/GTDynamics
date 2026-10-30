/**
 * @file ContactInitializer.cpp
 * Contact association, rigid-registration seeds, and sparse GTSAM optimization.
 *
 * Shared stance points define the measured connectivity. Each contact component
 * starts in an explicit gauge; only optional motion priors join those gauges.
 * Optimization changes root poses and landmarks while all local FK stays fixed.
 */

#include "ContactInitializer.h"

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

#include "ContactInitializationFactors.h"

namespace gtdynamics::contact_init {
namespace {
using Clock = std::chrono::steady_clock;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
// Use one monotonic clock for all native phase durations, reported in
// milliseconds.
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}
Point3 centroid(const std::vector<Point3>& points) {
  Point3 result = Point3::Zero();
  for (const auto& point : points) result += point;
  return result / static_cast<double>(points.size());
}
// Choose the shortest rotation aligning two directions, with a zero-vector
// fallback.
Rot3 align(const Point3& a, const Point3& b) {
  if (a.norm() < 1e-12 || b.norm() < 1e-12) return Rot3();
  return Rot3(Eigen::Quaterniond::FromTwoVectors(a, b));
}
// Level the first observed stance patch without using a reference root pose.
Rot3 levelRotation(const EventFrame& frame) {
  if (frame.observations.empty())
    throw std::invalid_argument(
        "Level ground requires supported event frames.");
  const std::string& id = frame.observations.front().stanceId;
  std::vector<Point3> points;
  for (const auto& o : frame.observations)
    if (o.stanceId == id) points.push_back(o.localPoint);
  if (points.size() < 3 || ContactPoseRank(points) != 6)
    throw std::invalid_argument(
        "Level ground requires a noncollinear sole patch.");
  Point3 normal =
      (points[1] - points[0]).cross(points[2] - points[0]).normalized();
  if (normal.z() < 0) normal = -normal;
  return align(normal, Point3::UnitZ());
}
// Hard ground changes the variable parameterization; soft height priors do not.
Point3 projected(const Graph& graph, const Point3& point) {
  Point3 result = point;
  if (graph.groundHeight) result.z() = *graph.groundHeight;
  return result;
}
// Average repeated world predictions to initialize each shared stance landmark.
std::vector<Point3> averageLandmarks(const Graph& graph,
                                     const std::vector<Pose3>& poses) {
  std::vector<Point3> result(graph.landmarks.size(), Point3::Zero());
  std::vector<size_t> counts(result.size(), 0);
  for (size_t i = 0; i < graph.frames.size(); ++i)
    for (const auto& o : graph.observations[i]) {
      result[o.landmarkIndex] +=
          poses[i].transformFrom(o.measurement.localPoint);
      ++counts[o.landmarkIndex];
    }
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = projected(graph, result[i] / static_cast<double>(counts[i]));
  return result;
}
// Associate by touchdown ID and material-point index, never by foot name alone.
void indexObservations(Graph& graph) {
  std::map<std::pair<std::string, size_t>, size_t> indices;
  for (const auto& frame : graph.frames) {
    std::vector<IndexedObservation> observations;
    std::vector<Point3> points;
    std::set<std::pair<std::string, size_t>> seen;
    for (const auto& o : frame.observations) {
      if (o.stanceId.empty() || !o.localPoint.allFinite())
        throw std::invalid_argument(
            "Each observation needs a stance ID and finite local FK point.");
      const auto key = std::make_pair(o.stanceId, o.pointIndex);
      if (!seen.insert(key).second)
        throw std::invalid_argument(
            "Duplicate stance point in an event frame.");
      const auto [iterator, inserted] =
          indices.emplace(key, graph.landmarks.size());
      if (inserted)
        graph.landmarks.push_back(
            {o.stanceId + ":" + std::to_string(o.pointIndex), o.stanceId,
             o.foot, o.pointIndex});
      observations.push_back({iterator->second, o});
      points.push_back(o.localPoint);
    }
    graph.observations.push_back(std::move(observations));
    graph.frameRanks.push_back(ContactPoseRank(points));
  }
}
// Traverse shared contacts, registering unknown frames against introduced
// points. A component's first frame fixes its gauge; subsequent frames inherit
// placement through measured overlaps. Disconnected components have arbitrary
// placements.
void seed(Graph& graph, const Options& options) {
  const size_t n = graph.frames.size();
  graph.seedPoses.resize(n);
  std::vector<bool> assigned(n, false), known(graph.landmarks.size(), false);
  std::vector<Point3> landmarks(graph.landmarks.size(), Point3::Zero());
  if (options.model == Model::Ground) {
    const auto level = levelRotation(
        graph.frames[0]);  // Validate even with a supplied anchor.
    const auto rotation =
        options.anchorPose ? options.anchorPose->rotation() : level;
    const Pose3 anchor(rotation, options.anchorPose
                                     ? options.anchorPose->translation()
                                     : Point3::Zero());
    std::vector<Point3> points;
    const auto& id = graph.frames[0].observations.front().stanceId;
    for (const auto& o : graph.frames[0].observations)
      if (o.stanceId == id)
        points.push_back(anchor.transformFrom(o.localPoint));
    graph.groundHeight = centroid(points).z();
  }
  for (size_t anchor = 0; anchor < n; ++anchor) {
    if (assigned[anchor]) continue;
    Point3 position(1.2 * static_cast<double>(graph.components.size()), 0, 0);
    Rot3 rotation;
    if (anchor > 0 && options.addBetweenFactors) {
      position = graph.seedPoses[anchor - 1].translation();
      rotation = graph.seedPoses[anchor - 1].rotation();
    }
    if (anchor == 0 && options.anchorPose) {
      rotation = options.anchorPose->rotation();
      position = options.anchorPose->translation();
    } else if (graph.groundHeight) {
      rotation = levelRotation(graph.frames[anchor]);
      std::vector<Point3> points;
      const auto& id = graph.frames[anchor].observations.front().stanceId;
      for (const auto& o : graph.frames[anchor].observations)
        if (o.stanceId == id) points.push_back(rotation.rotate(o.localPoint));
      position.z() = *graph.groundHeight - centroid(points).z();
    }
    graph.seedPoses[anchor] = Pose3(rotation, position);
    graph.anchors.push_back(anchor);
    graph.components.push_back({anchor});
    assigned[anchor] = true;
    auto introduce = [&](size_t frame) {
      for (const auto& o : graph.observations[frame])
        if (!known[o.landmarkIndex]) {
          known[o.landmarkIndex] = true;
          landmarks[o.landmarkIndex] = projected(
              graph,
              graph.seedPoses[frame].transformFrom(o.measurement.localPoint));
        }
    };
    introduce(anchor);
    bool changed = true;
    while (changed) {
      changed = false;
      for (size_t i = 0; i < n; ++i)
        if (!assigned[i]) {
          std::vector<Point3> local, world;
          for (const auto& o : graph.observations[i])
            if (known[o.landmarkIndex]) {
              local.push_back(o.measurement.localPoint);
              world.push_back(landmarks[o.landmarkIndex]);
            }
          if (local.empty()) continue;
          graph.seedPoses[i] = FitRigidTransform(local, world);
          assigned[i] = true;
          graph.components.back().push_back(i);
          introduce(i);
          changed = true;
        }
    }
  }
  graph.contactComponents = graph.components;
  if (options.addBetweenFactors) {
    // Connectivity from motion is an explicit prior, not a recovered contact.
    graph.anchors = {0};
    graph.components = {std::vector<size_t>()};
    for (size_t i = 0; i < n; ++i) graph.components.front().push_back(i);
  }
  graph.initialPoses = graph.seedPoses;
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  if (options.perturb)
    for (size_t i = 0; i < n; ++i)
      if (!anchors.count(i)) {
        Point3 translation = graph.initialPoses[i].translation();
        for (size_t j = 0; j < 3; ++j)
          translation[j] += 0.035 * std::sin(i * 2.3 + j * 1.7);
        graph.initialPoses[i] =
            Pose3(graph.initialPoses[i].rotation(), translation);
      }
  graph.initialLandmarks = averageLandmarks(graph, graph.initialPoses);
}
// Eliminate exact pose anchors by substituting their world predictions into
// unary landmark factors. Free poses retain the binary pose/landmark factors.
void buildFactors(Graph& graph, const Options& options) {
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  for (size_t i = 0; i < graph.frames.size(); ++i)
    if (!anchors.count(i))
      graph.initialValues.insert(options.poseKey(i), graph.initialPoses[i]);
  for (size_t i = 0; i < graph.landmarks.size(); ++i) {
    if (graph.groundHeight)
      graph.initialValues.insert(
          options.landmarkKey(i),
          gtsam::Point2(graph.initialLandmarks[i].head<2>()));
    else
      graph.initialValues.insert(options.landmarkKey(i),
                                 graph.initialLandmarks[i]);
  }
  for (size_t i = 0; i < graph.frames.size(); ++i)
    for (const auto& o : graph.observations[i]) {
      const auto key = options.landmarkKey(o.landmarkIndex);
      if (anchors.count(i)) {
        const auto target =
            graph.seedPoses[i].transformFrom(o.measurement.localPoint);
        if (graph.groundHeight)
          graph.factors.emplace_shared<AnchoredContactFactor2>(
              key, target, *graph.groundHeight);
        else
          graph.factors.emplace_shared<AnchoredContactFactor3>(key, target);
      } else {
        if (graph.groundHeight)
          graph.factors.emplace_shared<ContactFactor2>(options.poseKey(i), key,
                                                       o.measurement.localPoint,
                                                       *graph.groundHeight);
        else
          graph.factors.emplace_shared<ContactFactor3>(
              options.poseKey(i), key, o.measurement.localPoint);
      }
    }
  graph.contactFactors = graph.factors;
}
// Keep each prior family separate so contact-only RMS stays in geometric units
// and weighted objective contributions can be reported independently.
void buildRegularizers(Graph& graph, const Options& options) {
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  if (options.addBetweenFactors) {
    gtsam::Vector6 sigmas;
    sigmas.head<3>().setConstant(options.betweenRotationSigma);
    sigmas.tail<3>().setConstant(options.betweenTranslationSigma);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    for (size_t i = 1; i < graph.frames.size(); ++i) {
      if (anchors.count(i - 1)) {
        // Substituting the fixed first pose makes this identity edge a prior.
        graph.betweenFactors.emplace_shared<gtsam::PriorFactor<Pose3>>(
            options.poseKey(i), graph.seedPoses[i - 1], noise);
      } else {
        graph.betweenFactors.emplace_shared<gtsam::BetweenFactor<Pose3>>(
            options.poseKey(i - 1), options.poseKey(i), Pose3(), noise);
      }
    }
  }
  if (options.addUprightFactors) {
    const auto noise =
        gtsam::noiseModel::Isotropic::Sigma(3, options.uprightSigma);
    for (size_t i = 0; i < graph.frames.size(); ++i) {
      const UprightFactor factor(options.poseKey(i), options.uprightSigma);
      if (anchors.count(i)) {
        // Keep the gauge pose exact and retain its constant upright cost.
        const gtsam::Vector3 error = factor.evaluateError(graph.seedPoses[i]);
        graph.uprightFactors
            .emplace_shared<gtsam::ExpressionFactor<gtsam::Vector3>>(
                noise, gtsam::Vector3::Zero(),
                gtsam::Expression<gtsam::Vector3>(error));
      } else {
        graph.uprightFactors.add(factor);
      }
    }
  }
  if (options.addGroundFactors) {
    if (graph.groundHeight) {
      // Point2 landmarks already enforce the same height exactly.
      graph.groundPriorHeight = *graph.groundHeight;
      if (options.groundPriorHeight &&
          std::abs(*options.groundPriorHeight - *graph.groundHeight) > 1e-8)
        throw std::invalid_argument(
            "Ground prior height conflicts with the hard floor.");
    } else {
      std::vector<Point3> points;
      const auto& first = graph.frames.front();
      if (first.observations.empty())
        throw std::invalid_argument(
            "Ground prior requires a supported first frame.");
      const auto& stance = first.observations.front().stanceId;
      for (const auto& o : first.observations)
        if (o.stanceId == stance)
          points.push_back(graph.seedPoses.front().transformFrom(o.localPoint));
      graph.groundPriorHeight =
          options.groundPriorHeight.value_or(centroid(points).z());
      for (size_t i = 0; i < graph.landmarks.size(); ++i)
        graph.groundFactors.emplace_shared<LandmarkGroundFactor>(
            options.landmarkKey(i), *graph.groundPriorHeight,
            options.groundSigma);
    }
  }
  graph.factors.push_back(graph.betweenFactors);
  graph.factors.push_back(graph.uprightFactors);
  graph.factors.push_back(graph.groundFactors);
}
// Expand Point2 landmarks to world XYZ and restore fixed poses for consumers.
// RMS divides squared 3D contact distances by point count, not coordinate
// count.
Snapshot snapshot(const Graph& graph, const Options& options,
                  const gtsam::Values& values, size_t iteration) {
  Snapshot state;
  state.iteration = iteration;
  state.objective =
      2 * graph.factors.error(values);  // GTSAM error includes 1/2.
  size_t count = 0;
  for (const auto& frame : graph.observations) count += frame.size();
  state.rms = std::sqrt(2 * graph.contactFactors.error(values) /
                        std::max<size_t>(1, count));
  const std::set<size_t> anchors(graph.anchors.begin(), graph.anchors.end());
  for (size_t i = 0; i < graph.frames.size(); ++i)
    state.poses.push_back(anchors.count(i)
                              ? graph.seedPoses[i]
                              : values.at<Pose3>(options.poseKey(i)));
  for (size_t i = 0; i < graph.landmarks.size(); ++i) {
    if (graph.groundHeight) {
      const auto& xy = values.at<gtsam::Point2>(options.landmarkKey(i));
      state.landmarks.emplace_back(xy.x(), xy.y(), *graph.groundHeight);
    } else
      state.landmarks.push_back(values.at<Point3>(options.landmarkKey(i)));
  }
  return state;
}
}  // namespace

int ContactPoseRank(const std::vector<Point3>& points) {
  if (points.empty()) return 0;
  Point3 axis = Point3::Zero();
  for (const auto& p : points)
    if ((p - points[0]).norm() > 1e-8) {
      axis = p - points[0];
      break;
    }
  if (axis.norm() == 0) return 3;
  for (const auto& p : points)
    if (axis.cross(p - points[0]).norm() > 1e-9) return 6;
  return 5;
}

Pose3 FitRigidTransform(const std::vector<Point3>& local,
                        const std::vector<Point3>& world,
                        const Pose3& previous) {
  if (local.empty() || local.size() != world.size())
    throw std::invalid_argument(
        "Rigid registration needs matching nonempty point sets.");
  const Point3 a = centroid(local), b = centroid(world);
  const int rank = ContactPoseRank(local);
  Rot3 rotation = previous.rotation();
  if (rank == 5) {
    // Collinear correspondences observe an axis, not rotation around that axis.
    Point3 axis = Point3::Zero();
    for (const auto& p : local)
      if ((p - local[0]).norm() > axis.norm()) axis = p - local[0];
    axis.normalize();
    Point3 target = Point3::Zero();
    for (size_t i = 0; i < local.size(); ++i)
      target += (local[i] - a).dot(axis) * (world[i] - b);
    rotation = align(rotation.rotate(axis), target).compose(rotation);
  } else if (rank == 6) {
    // Orthogonal Procrustes registration, correcting reflections to stay in
    // SO(3).
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < local.size(); ++i)
      H += (local[i] - a) * (world[i] - b).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) =
        (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0 ? -1 : 1;
    rotation = Rot3(svd.matrixV() * correction * svd.matrixU().transpose());
  }
  return Pose3(rotation, b - rotation.rotate(a));
}

Result ContactInitializer::Initialize(const std::vector<EventFrame>& frames,
                                      const Options& options) const {
  if (frames.empty())
    throw std::invalid_argument(
        "At least one contact event frame is required.");
  for (double sigma :
       {options.betweenTranslationSigma, options.betweenRotationSigma,
        options.uprightSigma, options.groundSigma})
    if (!(sigma > 0) || !std::isfinite(sigma))
      throw std::invalid_argument(
          "Regularizer standard deviations must be finite and positive.");
  if (options.groundPriorHeight && !std::isfinite(*options.groundPriorHeight))
    throw std::invalid_argument("Ground prior height must be finite.");
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
  buildRegularizers(result.graph, options);
  result.timings.graphBuildMs += elapsed(phaseStart);
  phaseStart = Clock::now();
  // Native optimization timing includes LM construction and the iteration loop.
  // Optional history capture is inside this scope; benchmarks turn it off.
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = options.maxIterations;
  params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  params.setDiagonalDamping(true);  // Numerical LM damping, not a pose prior.
  params.lambdaInitial = 1e-3;
  params.relativeErrorTol = options.relativeErrorTolerance;
  params.absoluteErrorTol = options.absoluteErrorTolerance;
  gtsam::LevenbergMarquardtOptimizer optimizer(
      result.graph.factors, result.graph.initialValues, params);
  result.objectiveInitial = 2 * optimizer.error();
  if (options.recordHistory)
    result.history.push_back(
        snapshot(result.graph, options, optimizer.values(), 0));
  for (size_t iteration = 0; iteration < options.maxIterations; ++iteration) {
    const double before = optimizer.error();
    if (before < 1e-20) {
      result.converged = true;
      break;
    }
    optimizer.iterate();
    ++result.iterations;
    const double after = optimizer.error();
    if (!std::isfinite(after) || after > before + 1e-12 * std::max(1.0, before))
      throw std::runtime_error(
          "Native LM objective became non-finite or increased.");
    if (options.recordHistory)
      result.history.push_back(snapshot(result.graph, options,
                                        optimizer.values(), result.iterations));
    if (before - after <= options.absoluteErrorTolerance ||
        (before - after) / before <= options.relativeErrorTolerance) {
      result.converged = true;
      break;
    }
  }
  result.timings.optimizationMs = elapsed(phaseStart);
  auto final =
      snapshot(result.graph, options, optimizer.values(), result.iterations);
  result.poses = std::move(final.poses);
  result.landmarks = std::move(final.landmarks);
  result.objectiveFinal = final.objective;
  result.contactObjective =
      2 * result.graph.contactFactors.error(optimizer.values());
  result.betweenObjective =
      2 * result.graph.betweenFactors.error(optimizer.values());
  result.uprightObjective =
      2 * result.graph.uprightFactors.error(optimizer.values());
  result.groundObjective =
      2 * result.graph.groundFactors.error(optimizer.values());
  result.residualRms = final.rms;
  result.values = optimizer.values();
  // Downstream code receives ordinary Values containing the fixed gauges too.
  for (const size_t anchor : result.graph.anchors)
    result.values.insert(options.poseKey(anchor),
                         result.graph.seedPoses[anchor]);
  result.timings.totalMs = elapsed(totalStart);
  return result;
}
}  // namespace gtdynamics::contact_init
