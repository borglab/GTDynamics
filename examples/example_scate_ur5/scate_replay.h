/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_replay.h
 * @author Harrison Jenkins
 *
 * Writes a closed-loop run (or several, e.g. online and blind) to one JSON
 * file for plot_scate.py and the 3D replay viewer. No robot model needed: all geometry
 * comes from the CollisionModel's drawing hooks, so the viewer shows exactly
 * the spheres and link frames the planner used, not a re-derived FK.
 *
 * Contents, per run:
 *   frames  executed motion sampled `substeps` times per interval (exact:
 *           each interval is constant acceleration), with joint angles,
 *           sphere centers, link poses, obstacle and self clearance
 *   steps   one per planner call (steps[0] = offline plan): detection flag,
 *           obstacle estimate and covariance, the replanned future tool path,
 *           solve statistics
 *   summary the ClosedLoopResult scalars
 * Units: meters, radians, seconds. Quaternions are [x, y, z, w].
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "scate_online.h"

namespace scate {

/// One run to write, and what to call it.
struct ReplayRun {
  std::string name;   ///< short id, e.g. "online"
  std::string label;  ///< shown in the plots and viewer
  const ClosedLoopResult* result = nullptr;  ///< the run (not owned)
  double sensor_range = 0.0;                 ///< m; 0 for a blind run
};

/// Scenario numbers that are not in GraphSpec/OnlineConfig.
struct ReplayScenario {
  std::string source;  ///< free text: robot file, seed, build
  gtsam::Point3 obstacle_center = gtsam::Point3::Zero();  ///< ground truth
  double obstacle_radius = 0.0;    ///< ground truth
  /**
   * Moving obstacles (scate_reactive.h). If not empty, frames are scored
   * against these at their own time, and the replay also gets "obstacles",
   * per-frame "obstacle_centers" and per-step "obstacle_estimates".
   */
  std::vector<MovingObstacle> moving_obstacles;
  /**
   * Optional: the wrench on the base, [torque; force] (N m, N), from joint
   * angles, velocities and accelerations (base_wrench.h). If set, every
   * frame gets a "base" field.
   */
  std::function<gtsam::Vector6(const Vector&, const Vector&, const Vector&)> base_wrench;
  double obstacle_standoff = 0.0;  ///< epsilon of the obstacle factors
  double self_standoff = 0.0;      ///< epsilon of the self-collision pairs
  int substeps = 10;               ///< frames per interval
  int plan_samples = 4;            ///< planned-path points per interval
};

namespace replay_detail {

/// Minimal JSON writer: the caller writes the punctuation between items.
class JsonWriter {
 public:
  std::ostringstream stream;  ///< the JSON text so far

  /// Six significant digits, like the original output.
  JsonWriter() { stream << std::setprecision(6); }

  /// A number, or null if it is not finite.
  void writeNumber(double x) {
    if (std::isfinite(x)) {
      stream << x;
    } else {
      stream << "null";
    }
  }

  /// An array of numbers.
  void writeVector(const Vector& values) {
    stream << '[';
    for (long i = 0; i < values.size(); ++i) {
      if (i) stream << ',';
      writeNumber(values(i));
    }
    stream << ']';
  }

  /// A point as a 3-number array.
  void writePoint(const gtsam::Point3& point) { writeVector(Vector(point)); }

  /// A quoted string with quotes, backslashes and newlines escaped.
  void writeString(const std::string& text) {
    stream << '"';
    for (unsigned char c : text) {
      if (c == '"' || c == '\\') {
        stream << '\\' << static_cast<char>(c);
      } else if (c < 0x20) {
        constexpr char hex[] = "0123456789abcdef";
        stream << "\\u00" << hex[c >> 4] << hex[c & 0x0f];
      } else {
        stream << static_cast<char>(c);
      }
    }
    stream << '"';
  }

  /// An object key followed by a colon.
  void writeKey(const char* key) {
    writeString(key);
    stream << ':';
  }
};

/// Problem-level fields: format, timing, start/goal, limits, obstacle.
inline void writeHeader(JsonWriter& json, const ReplayScenario& scenario,
                        const GraphSpec& spec, const OnlineConfig& config) {
  auto& out = json.stream;
  json.writeKey("format"); json.writeString("scate-replay"); out << ',';
  json.writeKey("version"); out << 1 << ',';
  json.writeKey("source"); json.writeString(scenario.source); out << ',';
  json.writeKey("dt"); json.writeNumber(spec.dt); out << ',';
  json.writeKey("num_poses"); out << spec.num_poses << ',';
  json.writeKey("q_start"); json.writeVector(spec.q_start); out << ',';
  json.writeKey("q_goal"); json.writeVector(spec.q_goal); out << ',';
  json.writeKey("q_lower"); json.writeVector(spec.q_lower); out << ',';
  json.writeKey("q_upper"); json.writeVector(spec.q_upper); out << ',';
  json.writeKey("u_limit"); json.writeVector(spec.u_limit); out << ',';
  json.writeKey("obstacle_standoff"); json.writeNumber(scenario.obstacle_standoff); out << ',';
  json.writeKey("self_standoff"); json.writeNumber(scenario.self_standoff); out << ',';
  json.writeKey("planner_obstacle_radius"); json.writeNumber(config.obstacle_radius); out << ',';
  json.writeKey("obstacle");
  out << '{';
  json.writeKey("center"); json.writePoint(scenario.obstacle_center); out << ',';
  json.writeKey("radius"); json.writeNumber(scenario.obstacle_radius);
  out << "},";
  if (!scenario.moving_obstacles.empty()) {
    json.writeKey("obstacles");
    out << '[';
    for (size_t j = 0; j < scenario.moving_obstacles.size(); ++j) {
      const MovingObstacle& obstacle = scenario.moving_obstacles[j];
      if (j) out << ',';
      out << '{';
      json.writeKey("start"); json.writePoint(obstacle.start); out << ',';
      json.writeKey("velocity"); json.writeVector(obstacle.velocity); out << ',';
      json.writeKey("radius"); json.writeNumber(obstacle.radius);
      out << '}';
    }
    out << "],";
  }
}

/// Smallest gap to the obstacle(s) at time t: the static one, or every moving one.
inline double frameClearance(const CollisionModel& model, const ReplayScenario& scenario,
                             double t, const Vector& q) {
  if (scenario.moving_obstacles.empty()) {
    return model.obstacleClearance(q, scenario.obstacle_center, scenario.obstacle_radius);
  }
  double best = std::numeric_limits<double>::infinity();
  for (const MovingObstacle& obstacle : scenario.moving_obstacles) {
    best = std::min(best, model.obstacleClearance(q, obstacle.center(t), obstacle.radius));
  }
  return best;
}

/// Sphere radii, tool sphere, self-collision pairs and link names.
inline void writeStaticGeometry(JsonWriter& json, const CollisionModel& model,
                                const GraphSpec& spec) {
  auto& out = json.stream;
  json.writeKey("sphere_radii"); json.writeVector(model.sphereRadii()); out << ',';
  json.writeKey("tool_sphere"); out << model.toolSphere() << ',';
  json.writeKey("pairs");
  out << '[';
  const auto pairs = model.selfCollisionPairs();
  for (size_t p = 0; p < pairs.size(); ++p) {
    if (p) out << ',';
    out << '[' << pairs[p].first << ',' << pairs[p].second << ']';
  }
  out << "],";
  json.writeKey("links");
  out << '[';
  const auto frames = model.linkFrames(spec.q_start);
  for (size_t l = 0; l < frames.size(); ++l) {
    if (l) out << ',';
    json.writeString(frames[l].first);
  }
  out << "],";
}

/// Link poses as [x, y, z, qx, qy, qz, qw] arrays.
inline void writeLinkPoses(JsonWriter& json, const CollisionModel& model, const Vector& q) {
  auto& out = json.stream;
  out << '[';
  const auto frames = model.linkFrames(q);
  for (size_t l = 0; l < frames.size(); ++l) {
    if (l) out << ',';
    const gtsam::Pose3& pose = frames[l].second;
    const gtsam::Quaternion rotation = pose.rotation().toQuaternion();
    out << '[';
    json.writeNumber(pose.x()); out << ','; json.writeNumber(pose.y()); out << ',';
    json.writeNumber(pose.z()); out << ',';
    json.writeNumber(rotation.x()); out << ','; json.writeNumber(rotation.y()); out << ',';
    json.writeNumber(rotation.z()); out << ','; json.writeNumber(rotation.w());
    out << ']';
  }
  out << ']';
}

/// One frame of the executed motion: time, state, clearances and geometry.
inline void writeFrame(JsonWriter& json, const CollisionModel& model,
                       const ReplayScenario& scenario, double t, const Vector& q,
                       const Vector& v, const Vector& acceleration) {
  auto& out = json.stream;
  out << '{';
  json.writeKey("t"); json.writeNumber(t); out << ',';
  json.writeKey("q"); json.writeVector(q); out << ',';
  json.writeKey("v"); json.writeVector(v); out << ',';
  if (scenario.base_wrench) {
    json.writeKey("base"); json.writeVector(scenario.base_wrench(q, v, acceleration)); out << ',';
  }
  json.writeKey("clear"); json.writeNumber(frameClearance(model, scenario, t, q)); out << ',';
  if (!scenario.moving_obstacles.empty()) {
    json.writeKey("obstacle_centers");
    out << '[';
    for (size_t j = 0; j < scenario.moving_obstacles.size(); ++j) {
      if (j) out << ',';
      json.writePoint(scenario.moving_obstacles[j].center(t));
    }
    out << "],";
  }
  json.writeKey("self"); json.writeNumber(model.selfClearance(q)); out << ',';
  json.writeKey("spheres");
  out << '[';
  const gtsam::Matrix centers = model.sphereCenters(q);
  for (long s = 0; s < centers.cols(); ++s) {
    if (s) out << ',';
    json.writeVector(centers.col(s));
  }
  out << "],";
  json.writeKey("links");
  writeLinkPoses(json, model, q);
  out << '}';
}

/// Executed motion: exact constant-acceleration samples, then the final state.
inline void writeExecutedFrames(JsonWriter& json, const CollisionModel& model,
                                const ReplayScenario& scenario, const GraphSpec& spec,
                                const ClosedLoopResult& result) {
  auto& out = json.stream;
  const int last = spec.num_poses - 1;
  const double dt = spec.dt;
  json.writeKey("frames");
  out << '[';
  bool first = true;
  for (int k = 0; k < last; ++k) {
    for (int s = 0; s < scenario.substeps; ++s) {
      const double tau = dt * s / scenario.substeps;
      const Vector q = result.q[k] + tau * result.v[k] + 0.5 * tau * tau * result.u[k];
      const Vector v = result.v[k] + tau * result.u[k];
      if (!first) out << ',';
      first = false;
      writeFrame(json, model, scenario, k * dt + tau, q, v, result.u[k]);
    }
  }
  out << ',';
  // At the final state the arm has stopped accelerating.
  writeFrame(json, model, scenario, last * dt, result.q[last], result.v[last],
             Vector::Zero(result.q[last].size()));
  out << "],";
}

/// The executed controls.
inline void writeControls(JsonWriter& json, const ClosedLoopResult& result) {
  auto& out = json.stream;
  json.writeKey("u");
  out << '[';
  for (size_t k = 0; k < result.u.size(); ++k) {
    if (k) out << ',';
    json.writeVector(result.u[k]);
  }
  out << "],";
}

/// The step's estimate of l and its covariance (row-major), or nulls.
inline void writeObstacleEstimate(JsonWriter& json, const StepInfo& step) {
  auto& out = json.stream;
  json.writeKey("l");
  if (step.have_obstacle) {
    json.writePoint(step.l_estimate);
  } else {
    out << "null";
  }
  out << ',';
  json.writeKey("l_cov");
  if (step.l_covariance) {
    out << '[';
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        if (row || col) out << ',';
        json.writeNumber((*step.l_covariance)(row, col));
      }
    }
    out << ']';
  } else {
    out << "null";
  }
  out << ',';
}

/**
 * Planned tool path of a step ("plan_t" times and "plan_tool" positions),
 * interpolated between the planned states the same way the collision factors
 * interpolate them.
 */
inline void writePlannedToolPath(JsonWriter& json, const CollisionModel& model,
                                 const ReplayScenario& scenario, const GraphSpec& spec,
                                 const StepInfo& step) {
  auto& out = json.stream;
  const int last = spec.num_poses - 1;
  const double dt = spec.dt;
  const int tool = model.toolSphere();
  std::ostringstream tool_path;
  tool_path << std::setprecision(6);
  bool first = true;
  const auto addSample = [&](double t, const Vector& q) {
    if (!first) out << ',';
    json.writeNumber(t);
    if (tool >= 0) {
      const gtsam::Matrix centers = model.sphereCenters(q);
      if (!first) tool_path << ',';
      tool_path << '[' << centers(0, tool) << ',' << centers(1, tool) << ','
                << centers(2, tool) << ']';
    }
    first = false;
  };

  json.writeKey("plan_t");
  out << '[';
  for (size_t i = 0; i + 1 < step.plan_q.size(); ++i) {
    for (int p = 0; p < scenario.plan_samples; ++p) {
      const double tau = dt * p / scenario.plan_samples;
      addSample((step.k + static_cast<int>(i)) * dt + tau,
                interpolatePosition(step.plan_q[i], step.plan_v[i], step.plan_q[i + 1],
                                    step.plan_v[i + 1], dt, tau));
    }
  }
  if (!step.plan_q.empty()) addSample(last * dt, step.plan_q.back());
  out << "],";
  json.writeKey("plan_tool"); out << '[' << tool_path.str() << ']';
}

/// One planner step: detection, estimate, solve statistics, replanned future.
inline void writeStep(JsonWriter& json, const CollisionModel& model,
                      const ReplayScenario& scenario, const GraphSpec& spec,
                      const StepInfo& step) {
  auto& out = json.stream;
  out << '{';
  json.writeKey("k"); out << step.k << ',';
  json.writeKey("detected"); out << (step.detected ? "true" : "false") << ',';
  writeObstacleEstimate(json, step);
  if (!step.obstacle_estimates.empty()) {
    json.writeKey("obstacle_estimates");
    out << '[';
    for (size_t j = 0; j < step.obstacle_estimates.size(); ++j) {
      if (j) out << ',';
      if (step.obstacle_estimates[j]) json.writePoint(*step.obstacle_estimates[j]);
      else out << "null";
    }
    out << "],";
  }
  json.writeKey("solves"); out << step.solves << ',';
  json.writeKey("solve_ms"); json.writeNumber(step.solve_ms); out << ',';
  json.writeKey("graph_error"); json.writeNumber(step.graph_error); out << ',';
  json.writeKey("factors"); out << step.num_factors << ',';
  json.writeKey("plan_q");
  out << '[';
  for (size_t i = 0; i < step.plan_q.size(); ++i) {
    if (i) out << ',';
    json.writeVector(step.plan_q[i]);
  }
  out << "],";
  writePlannedToolPath(json, model, scenario, spec, step);
  out << '}';
}

/// The ClosedLoopResult scalars (closes the summary object only).
inline void writeSummary(JsonWriter& json, const ClosedLoopResult& result) {
  auto& out = json.stream;
  json.writeKey("summary");
  out << '{';
  json.writeKey("min_obstacle_clearance"); json.writeNumber(result.min_obstacle_clearance); out << ',';
  json.writeKey("min_self_clearance"); json.writeNumber(result.min_self_clearance); out << ',';
  json.writeKey("first_detection"); out << result.first_detection << ',';
  json.writeKey("goal_error"); json.writeNumber(result.goal_error); out << ',';
  json.writeKey("final_speed"); json.writeNumber(result.final_speed); out << ',';
  json.writeKey("max_solve_ms"); json.writeNumber(result.max_solve_ms); out << ',';
  json.writeKey("collided"); out << (result.collided() ? "true" : "false");
  out << '}';
}

/// One run: names, executed frames, controls, planner steps and summary.
inline void writeRun(JsonWriter& json, const CollisionModel& model,
                     const ReplayScenario& scenario, const GraphSpec& spec,
                     const ReplayRun& run) {
  if (!run.result) throw std::invalid_argument("writeReplayJson: null result");
  const ClosedLoopResult& result = *run.result;
  const int last = spec.num_poses - 1;
  if (static_cast<int>(result.q.size()) != last + 1 ||
      static_cast<int>(result.u.size()) != last) {
    throw std::invalid_argument("writeReplayJson: run '" + run.name +
                                "' does not have N+1 states and N controls");
  }
  auto& out = json.stream;
  out << '{';
  json.writeKey("name"); json.writeString(run.name); out << ',';
  json.writeKey("label"); json.writeString(run.label); out << ',';
  json.writeKey("sensor_range"); json.writeNumber(run.sensor_range); out << ',';
  writeExecutedFrames(json, model, scenario, spec, result);
  writeControls(json, result);
  json.writeKey("steps");
  out << '[';
  for (size_t s = 0; s < result.steps.size(); ++s) {
    if (s) out << ',';
    writeStep(json, model, scenario, spec, result.steps[s]);
  }
  out << "],";
  writeSummary(json, result);
  out << '}';
}

}  // namespace replay_detail

/**
 * Write one or more closed-loop runs, with the scenario and the model's
 * geometry, to a JSON file at `path` (read by plot_scate.py and the viewer).
 */
inline void writeReplayJson(const std::string& path, const ReplayScenario& scenario,
                            const GraphSpec& spec, const OnlineConfig& config,
                            const CollisionModel& model, const std::vector<ReplayRun>& runs) {
  if (scenario.substeps < 1 || scenario.plan_samples < 1) {
    throw std::invalid_argument("writeReplayJson: substeps and plan_samples must be >= 1");
  }
  replay_detail::JsonWriter json;
  auto& out = json.stream;
  out << '{';
  replay_detail::writeHeader(json, scenario, spec, config);
  replay_detail::writeStaticGeometry(json, model, spec);
  json.writeKey("runs");
  out << '[';
  for (size_t r = 0; r < runs.size(); ++r) {
    if (r) out << ',';
    replay_detail::writeRun(json, model, scenario, spec, runs[r]);
  }
  out << "]}";

  std::ofstream file(path);
  if (!file) throw std::runtime_error("writeReplayJson: cannot open '" + path + "'");
  file << out.str() << '\n';
}

}  // namespace scate
