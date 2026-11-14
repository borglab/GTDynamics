/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file live_scenario.h
 * @author Harrison Jenkins
 *
 * Reactive SCATE with the base-disturbance comparison on a start, goal and
 * moving obstacles chosen by the user, for scate_live and the SCATE UR10e
 * Base app. The app reads a robot description (link frames at rest, joint
 * screw axes, spheres, limits and the default experiment) to pose and drag
 * arms in the browser, and sends back a request; scate_live solves it and
 * writes a replay in the format of scate_replay.h.
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <istream>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "base_wrench.h"
#include "scate_reactive_scenario.h"
#include "ur10e_collision.h"
#include "ur10e_scenarios.h"
#include "ur5_collision.h"

namespace scate {

/// A robot the live app can solve for, with its collision geometry and default experiment.
struct LiveRobot {
  std::string key;                  ///< request name: "ur5" or "ur10e"
  std::string urdf;                 ///< robot file
  SphereModel sphere_model;         ///< spheres, pairs and motion-capture markers
  SphereCollisionParams params;     ///< collision factors and SDF grid
  ReactiveScenario scenario;        ///< the default experiment (goal and crossings)
  std::string rig_name;             ///< the lab rig around this robot, if any
  std::vector<HalfSpace> rig;       ///< its walls and floor (empty = none)
  /// An experiment that suits the rig (the app's starting point for hardware runs).
  Vector rig_q_start, rig_q_goal;
  std::vector<MovingObstacle> rig_obstacles;
};

/**
 * A UR10e experiment for the MK 111 rig. The default experiment's goal
 * reaches toward base +y and ends 0.3 cm from the floor, so this one sweeps
 * the other way (pan -1.6 rad, toward base -y: up, in the lab) with the
 * wrist turned in. One static obstacle sits just above the straight
 * joint-space path at its midpoint, and one descends slowly toward the end
 * of the path. Its ghost runs (encoders, sigma 0 and 3, seeds 0-2) keep
 * 4-12 cm from the obstacles and 14 cm from the wall and floor.
 */
inline void setMk111RigExperiment(Vector* q_start, Vector* q_goal,
                                  std::vector<MovingObstacle>* obstacles) {
  *q_start = (Vector(6) << 0, -1.5708, 1.5708, -1.5708, -1.5708, 0).finished();
  *q_goal = (Vector(6) << -1.6, -1.2, 1.3, -1.9, -1.8, 0.4).finished();
  MovingObstacle still, falling;
  still.start = gtsam::Point3(0.72, -0.47, 0.78);
  still.velocity = gtsam::Vector3::Zero();
  still.radius = 0.06;
  falling.start = gtsam::Point3(0.45, -0.75, 1.0);
  falling.velocity = gtsam::Vector3(0, 0, -0.012);
  falling.radius = 0.06;
  *obstacles = {still, falling};
}

/// The UR5 version of ur10eReactiveScenario: the same sweep and crossings.
inline ReactiveScenario ur5ReactiveScenario() {
  ReactiveScenario scenario = ur10eReactiveScenario();
  scenario.robot_name = "UR5";
  return scenario;
}

/// The live robot named key ("ur5" or "ur10e"); throws for any other name.
inline LiveRobot liveRobot(const std::string& key) {
  LiveRobot robot;
  robot.key = key;
  if (key == "ur10e") {
    robot.urdf = ur10eUrdfPath();
    robot.sphere_model = ur10eSphereModel();
    robot.params = ur10eCollisionParams();
    robot.scenario = ur10eReactiveScenario();
    robot.rig_name = "MK 111 wall and floor";
    robot.rig = mk111RigEnvironment();
    setMk111RigExperiment(&robot.rig_q_start, &robot.rig_q_goal, &robot.rig_obstacles);
  } else if (key == "ur5") {
    robot.urdf = ur5UrdfPath();
    robot.sphere_model = ur5SphereModel();
    robot.sphere_model.markers = ur5Markers();
    robot.scenario = ur5ReactiveScenario();
  } else {
    throw std::invalid_argument("unknown robot '" + key + "' (expected ur5 or ur10e)");
  }
  robot.scenario.program = "scate_live";
  return robot;
}

/// A live solve: start, goal, moving obstacles and base-wrench sigmas (0 = cost off).
struct LiveRequest {
  std::string robot = "ur10e";                  ///< "ur5" or "ur10e"
  Vector q_start;                               ///< rad, one per joint
  Vector q_goal;                                ///< rad, one per joint
  std::vector<MovingObstacle> obstacles;        ///< at least one
  std::vector<double> sigmas{0.0, 3.0, 0.3, 0.1};  ///< one run each
  unsigned seed = 0;                            ///< noise seed shared by every run
  std::vector<HalfSpace> environment;           ///< walls and floor (empty = free space)
  /// Joint speed limit of the planner, rad/s, every joint (unset = kVelocityLimit).
  std::optional<double> v_limit;
  /// Joint acceleration limit of the planner, rad/s^2, every joint (unset = kAccelerationLimit).
  std::optional<double> u_limit;
  /**
   * How the arm's state is measured: "markers" (motion capture, the
   * scenario's default) or "encoders" (joint encoders only, as on hardware).
   */
  std::string sensing = "markers";
  /// Prior of an executed control on the commanded one (rad/s^2): the process noise.
  double sigma_executed_control = 1e-2;
  /// Base-cost sigma of the run to execute on hardware (unset = the first of sigmas).
  std::optional<double> hardware_sigma;
};

/// Highest limits a request may ask for (rad/s and rad/s^2): a margin below the UR10e's own.
inline constexpr double kMaxVelocityLimit = 1.0;
inline constexpr double kMaxAccelerationLimit = 2.0;

/**
 * Read a request, one item per line, '#' starting a comment:
 *   robot ur10e
 *   q_start q1 ... q6
 *   q_goal q1 ... q6
 *   obstacle x y z vx vy vz radius     (once per obstacle; m, m/s)
 *   sigmas 0 3 0.3 0.1
 *   seed 0
 *   plane name nx ny nz offset         (optional, once per wall or floor: free
 *                                       space is n . p >= offset; n unit length)
 *   v_limit 0.5                        (optional, rad/s, every joint)
 *   u_limit 1.0                        (optional, rad/s^2, every joint)
 *   sensing encoders                   (optional: markers or encoders)
 *   sigma_executed_control 0.01        (optional, rad/s^2)
 *   hardware_sigma 3                   (optional: the run executed on hardware)
 */
inline LiveRequest parseLiveRequest(std::istream& in) {
  LiveRequest request;
  bool have_sigmas = false;
  std::string line;
  int line_number = 0;
  auto numbers = [&](std::istringstream& fields) {
    std::vector<double> values;
    std::string field;
    while (fields >> field) {
      try {
        size_t used = 0;
        values.push_back(std::stod(field, &used));
        if (used != field.size()) throw std::invalid_argument(field);
      } catch (const std::exception&) {
        throw std::invalid_argument("line " + std::to_string(line_number) + ": '" + field +
                                    "' is not a number");
      }
    }
    return values;
  };
  auto toVector = [](const std::vector<double>& values) {
    Vector vector(values.size());
    for (size_t i = 0; i < values.size(); ++i) vector(i) = values[i];
    return vector;
  };
  while (std::getline(in, line)) {
    ++line_number;
    const size_t comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    std::istringstream fields(line);
    std::string key;
    if (!(fields >> key)) continue;
    if (key == "robot") {
      fields >> request.robot;
    } else if (key == "q_start" || key == "q_goal") {
      (key == "q_start" ? request.q_start : request.q_goal) = toVector(numbers(fields));
    } else if (key == "obstacle") {
      const std::vector<double> values = numbers(fields);
      if (values.size() != 7 || values[6] <= 0.0) {
        throw std::invalid_argument("line " + std::to_string(line_number) +
                                    ": obstacle needs x y z vx vy vz radius (radius > 0)");
      }
      MovingObstacle obstacle;
      obstacle.start = gtsam::Point3(values[0], values[1], values[2]);
      obstacle.velocity = gtsam::Vector3(values[3], values[4], values[5]);
      obstacle.radius = values[6];
      request.obstacles.push_back(obstacle);
    } else if (key == "plane") {
      HalfSpace boundary;
      if (!(fields >> boundary.name)) {
        throw std::invalid_argument("line " + std::to_string(line_number) +
                                    ": plane needs name nx ny nz offset");
      }
      const std::vector<double> values = numbers(fields);
      const gtsam::Vector3 normal =
          values.size() == 4 ? gtsam::Vector3(values[0], values[1], values[2])
                             : gtsam::Vector3::Zero();
      if (values.size() != 4 || std::abs(normal.norm() - 1.0) > 1e-3) {
        throw std::invalid_argument("line " + std::to_string(line_number) +
                                    ": plane needs name nx ny nz offset with a unit normal");
      }
      boundary.normal = normal.normalized();
      boundary.offset = values[3];
      request.environment.push_back(boundary);
    } else if (key == "v_limit" || key == "u_limit" || key == "sigma_executed_control" ||
               key == "hardware_sigma") {
      const std::vector<double> values = numbers(fields);
      const double cap = key == "v_limit"   ? kMaxVelocityLimit
                         : key == "u_limit" ? kMaxAccelerationLimit
                                            : std::numeric_limits<double>::infinity();
      const bool zero_ok = key == "hardware_sigma";
      if (values.size() != 1 || !(zero_ok ? values[0] >= 0.0 : values[0] > 0.0) ||
          values[0] > cap) {
        throw std::invalid_argument("line " + std::to_string(line_number) + ": " + key +
                                    " needs one value in " + (zero_ok ? "[0, " : "(0, ") +
                                    (std::isfinite(cap) ? std::to_string(cap).substr(0, 3) : "inf") +
                                    "]");
      }
      if (key == "v_limit") request.v_limit = values[0];
      else if (key == "u_limit") request.u_limit = values[0];
      else if (key == "sigma_executed_control") request.sigma_executed_control = values[0];
      else request.hardware_sigma = values[0];
    } else if (key == "sensing") {
      if (!(fields >> request.sensing) || (request.sensing != "markers" && request.sensing != "encoders")) {
        throw std::invalid_argument("line " + std::to_string(line_number) +
                                    ": sensing must be markers or encoders");
      }
    } else if (key == "sigmas") {
      request.sigmas = numbers(fields);
      have_sigmas = true;
    } else if (key == "seed") {
      const std::vector<double> values = numbers(fields);
      if (values.size() != 1 || values[0] < 0) {
        throw std::invalid_argument("line " + std::to_string(line_number) + ": seed needs one value");
      }
      request.seed = static_cast<unsigned>(values[0]);
    } else {
      throw std::invalid_argument("line " + std::to_string(line_number) + ": unknown item '" +
                                  key + "'");
    }
  }
  if (request.q_start.size() == 0 || request.q_goal.size() != request.q_start.size()) {
    throw std::invalid_argument("the request needs q_start and q_goal of the same length");
  }
  if (request.obstacles.empty()) throw std::invalid_argument("the request needs an obstacle");
  if (have_sigmas && request.sigmas.empty()) throw std::invalid_argument("sigmas is empty");
  for (double sigma : request.sigmas) {
    if (sigma < 0.0) throw std::invalid_argument("sigmas must be >= 0 (0 = cost off)");
  }
  if (request.hardware_sigma &&
      std::find(request.sigmas.begin(), request.sigmas.end(), *request.hardware_sigma) ==
          request.sigmas.end()) {
    throw std::invalid_argument("hardware_sigma must be one of sigmas");
  }
  return request;
}

/// The default experiment of a robot: start, IK goal and the crossing obstacles.
struct LiveDefaults {
  Vector q_start;                         ///< rad
  Vector q_goal;                          ///< rad
  std::vector<MovingObstacle> obstacles;  ///< placed across the cost-free plan
};

/// Everything the solver needs for one robot: model, graph spec and base wrench.
struct LiveSetup {
  gtdynamics::Robot robot;
  GraphSpec spec;
  gtsam::Values initial;  ///< smooth rest-to-rest guess
  std::shared_ptr<SphereCollisionModel> model;
  std::shared_ptr<const BaseWrenchModel> base_wrench;
};

/**
 * The problem from q_start to exactly q_goal (no IK), with the smooth
 * initial guess of initializeProblem.
 */
inline LiveSetup makeLiveSetup(const LiveRobot& live, const Vector& q_start, const Vector& q_goal,
                               const std::vector<HalfSpace>& environment = {}) {
  LiveSetup setup{loadRobot(live.urdf), {}, {}, nullptr, nullptr};
  const size_t num_joints = setup.robot.joints().size();
  if (q_start.size() != static_cast<long>(num_joints) ||
      q_goal.size() != static_cast<long>(num_joints)) {
    throw std::invalid_argument("q_start and q_goal need " + std::to_string(num_joints) +
                                " joint angles for " + live.scenario.robot_name);
  }
  InitialProblem problem;
  problem.q_start = q_start;
  problem.q_goal = q_goal;
  TrajectoryConfig trajectory;
  trajectory.num_poses = kNumPoses;
  trajectory.dt = kDt;
  trajectory.bump_amplitude = kBumpAmplitude;
  problem.trajectory = makeInitialTrajectory(q_start, q_goal, trajectory);
  problem.values = trajectoryValues(problem.trajectory);
  setup.spec = makeGraphSpec(setup.robot, problem);
  setup.initial = problem.values;
  SphereCollisionParams params = live.params;
  params.dt = setup.spec.dt;
  params.environment = environment;
  setup.model = std::make_shared<SphereCollisionModel>(setup.robot, live.sphere_model, params);
  setup.base_wrench = std::make_shared<const BaseWrenchModel>(setup.robot);
  return setup;
}

/// The setup of a request: makeLiveSetup with its walls and floor and its limits.
inline LiveSetup makeLiveSetup(const LiveRobot& live, const LiveRequest& request) {
  LiveSetup setup = makeLiveSetup(live, request.q_start, request.q_goal, request.environment);
  const long n = setup.spec.q_start.size();
  if (request.v_limit) setup.spec.v_limit = Vector::Constant(n, *request.v_limit);
  if (request.u_limit) setup.spec.u_limit = Vector::Constant(n, *request.u_limit);
  return setup;
}

/// Planner settings of a live solve: the scenario's, with one radius per obstacle.
inline ReactiveConfig liveConfig(const LiveRobot& live,
                                 const std::vector<MovingObstacle>& obstacles) {
  ReactiveConfig config;
  for (const MovingObstacle& obstacle : obstacles) config.obstacle_radii.push_back(obstacle.radius);
  config.tau_fractions = evenlySpacedFractions(live.scenario.checks_per_interval);
  config.effort_sigmas = live.scenario.effort_sigmas;
  return config;
}

/// Planner settings of a request: liveConfig plus its executed-control sigma.
inline ReactiveConfig liveConfig(const LiveRobot& live, const LiveRequest& request) {
  ReactiveConfig config = liveConfig(live, request.obstacles);
  config.sigma_executed_control = request.sigma_executed_control;
  return config;
}

/// The planner settings of one run: base cost on at sigma (> 0) or off (0).
inline ReactiveConfig runConfig(ReactiveConfig config, const LiveSetup& setup, double sigma) {
  if (sigma > 0.0) {
    config.base_wrench = setup.base_wrench;
    config.sigma_base_torque = sigma;
    config.sigma_base_force = sigma;
  }
  return config;
}

/// The simulated sensing of a request's closed loop (ghost run).
inline ViconSimulationConfig liveSimulation(const LiveRobot& live, const LiveRequest& request) {
  ViconSimulationConfig simulation;
  simulation.obstacles = request.obstacles;
  simulation.use_encoders = live.scenario.use_encoders || request.sensing == "encoders";
  simulation.use_markers = request.sensing == "markers";
  simulation.seed = request.seed;
  return simulation;
}

/// The default experiment: the scenario's start, IK goal and crossing obstacles.
inline LiveDefaults liveDefaults(const LiveRobot& live) {
  const gtdynamics::Robot robot = loadRobot(live.urdf);
  const InitialProblem problem =
      initializeProblem(makeKinematics(), robot, startJoints(), live.scenario.goal_joints);
  LiveSetup setup = makeLiveSetup(live, problem.q_start, problem.q_goal);
  ReactivePlanner probe(setup.spec, reactiveConfig(live.scenario), setup.model, setup.initial);
  probe.planOffline();
  return {problem.q_start, problem.q_goal,
          placeCrossingObstacles(live.scenario, *setup.model, probe.values(), setup.spec)};
}

namespace live_detail {

/// A number with enough digits to round-trip the poses.
inline void number(std::ostream& out, double value) {
  out << std::setprecision(9) << (std::abs(value) < 1e-12 ? 0.0 : value);
}

inline void array(std::ostream& out, const double* values, size_t count) {
  out << '[';
  for (size_t i = 0; i < count; ++i) {
    if (i) out << ',';
    number(out, values[i]);
  }
  out << ']';
}

inline void vector(std::ostream& out, const Vector& values) {
  array(out, values.data(), static_cast<size_t>(values.size()));
}

inline void point(std::ostream& out, const gtsam::Point3& p) { array(out, p.data(), 3); }

/// [x, y, z, qx, qy, qz, qw], as in the replay's link poses.
inline void pose(std::ostream& out, const gtsam::Pose3& pose) {
  const gtsam::Quaternion rotation = pose.rotation().toQuaternion();
  const double values[7] = {pose.x(), pose.y(), pose.z(), rotation.x(), rotation.y(),
                            rotation.z(), rotation.w()};
  array(out, values, 7);
}

inline void string(std::ostream& out, const std::string& text) { out << '"' << text << '"'; }

}  // namespace live_detail

/**
 * Space-frame screw axis [w; v] of each joint, at q = 0: the twist that
 * moves the joint's child link frame per radian. With these and the link
 * frames at rest, the browser poses link l by the product of exponentials
 * exp(S_1 q_1) ... exp(S_l q_l) T_l(0) (links in chain order, link 0 the base).
 */
inline std::vector<gtsam::Vector6> spaceScrewAxes(const SphereCollisionModel& model,
                                                   size_t num_joints) {
  const Vector rest = Vector::Zero(num_joints);
  const auto rest_frames = model.linkFrames(rest);
  std::vector<gtsam::Vector6> axes;
  constexpr double kStep = 1e-3;  // a joint's motion is a pure screw: no truncation error
  for (size_t j = 0; j < num_joints; ++j) {
    Vector q = rest;
    q(j) = kStep;
    const gtsam::Pose3 moved = model.linkFrames(q)[j + 1].second;
    axes.push_back(gtsam::Pose3::Logmap(moved * rest_frames[j + 1].second.inverse()) / kStep);
  }
  return axes;
}

/**
 * Robot description for the live app: links and their frames at rest,
 * joint names, limits and screw axes, collision spheres (link index and
 * center in the link frame), the tool sphere, the SDF grid bounds the
 * obstacles must stay in, and the default experiment.
 */
inline void writeLiveDescription(std::ostream& out, const LiveRobot& live,
                                 const LiveDefaults& defaults) {
  using namespace live_detail;
  LiveSetup setup = makeLiveSetup(live, defaults.q_start, defaults.q_goal);
  const size_t num_joints = static_cast<size_t>(defaults.q_start.size());
  const auto rest_frames = setup.model->linkFrames(Vector::Zero(num_joints));
  const auto joints = jointsInChainOrder(setup.robot);
  const auto axes = spaceScrewAxes(*setup.model, num_joints);
  const SdfGrid& grid = live.params.grid;
  const double duration = (setup.spec.num_poses - 1) * setup.spec.dt;

  out << '{';
  out << "\"format\":\"scate-live-robot\",\"version\":1,";
  out << "\"key\":"; string(out, live.key); out << ',';
  out << "\"name\":"; string(out, live.scenario.robot_name); out << ',';
  out << "\"links\":[";
  for (size_t l = 0; l < rest_frames.size(); ++l) {
    if (l) out << ',';
    out << "{\"name\":"; string(out, rest_frames[l].first);
    out << ",\"rest\":"; pose(out, rest_frames[l].second); out << '}';
  }
  out << "],\"joints\":[";
  for (size_t j = 0; j < num_joints; ++j) {
    if (j) out << ',';
    out << "{\"name\":"; string(out, joints[j]->name());
    out << ",\"lower\":"; number(out, setup.spec.q_lower(j));
    out << ",\"upper\":"; number(out, setup.spec.q_upper(j));
    out << ",\"screw\":"; array(out, axes[j].data(), 6); out << '}';
  }
  out << "],\"spheres\":[";
  for (size_t s = 0; s < live.sphere_model.spheres.size(); ++s) {
    const CollisionSphere& sphere = live.sphere_model.spheres[s];
    size_t link = 0;
    while (link < rest_frames.size() && rest_frames[link].first != sphere.link) ++link;
    if (link == rest_frames.size()) throw std::runtime_error("sphere on unknown link " + sphere.link);
    if (s) out << ',';
    out << "{\"link\":" << link << ",\"center\":"; point(out, sphere.center_in_link);
    out << ",\"radius\":"; number(out, sphere.radius); out << '}';
  }
  out << "],\"tool_sphere\":" << live.sphere_model.tool_sphere << ',';
  const gtsam::Point3 far_corner =
      grid.origin + grid.cell_size * gtsam::Point3(grid.num_x - 1, grid.num_y - 1, grid.num_z - 1);
  out << "\"workspace\":{\"min\":"; point(out, grid.origin);
  out << ",\"max\":"; point(out, far_corner); out << "},";
  out << "\"obstacle_standoff\":"; number(out, live.params.obstacle_epsilon); out << ',';
  // The lab rig the app can switch on: its walls and floor, sent back as
  // "plane" lines (free space is normal . p >= offset).
  out << "\"rig\":";
  if (live.rig.empty()) {
    out << "null,";
  } else {
    out << "{\"name\":"; string(out, live.rig_name); out << ",\"planes\":[";
    for (size_t b = 0; b < live.rig.size(); ++b) {
      if (b) out << ',';
      out << "{\"name\":"; string(out, live.rig[b].name);
      out << ",\"normal\":"; array(out, live.rig[b].normal.data(), 3);
      out << ",\"offset\":"; number(out, live.rig[b].offset); out << '}';
    }
    out << "],\"defaults\":{\"q_start\":"; vector(out, live.rig_q_start);
    out << ",\"q_goal\":"; vector(out, live.rig_q_goal);
    out << ",\"obstacles\":[";
    for (size_t j = 0; j < live.rig_obstacles.size(); ++j) {
      const MovingObstacle& obstacle = live.rig_obstacles[j];
      if (j) out << ',';
      out << "{\"start\":"; point(out, obstacle.start);
      out << ",\"end\":"; point(out, obstacle.center(duration));
      out << ",\"radius\":"; number(out, obstacle.radius); out << '}';
    }
    out << "]}},";
  }
  // The planner's joint limits and the most a request may ask for.
  out << "\"limits\":{\"v_limit\":"; number(out, setup.spec.v_limit.maxCoeff());
  out << ",\"u_limit\":"; number(out, setup.spec.u_limit.maxCoeff());
  out << ",\"max_v_limit\":"; number(out, kMaxVelocityLimit);
  out << ",\"max_u_limit\":"; number(out, kMaxAccelerationLimit); out << "},";
  out << "\"duration\":"; number(out, duration); out << ',';
  out << "\"dt\":"; number(out, setup.spec.dt); out << ',';
  out << "\"defaults\":{\"q_start\":"; vector(out, defaults.q_start);
  out << ",\"q_goal\":"; vector(out, defaults.q_goal);
  out << ",\"obstacles\":[";
  for (size_t j = 0; j < defaults.obstacles.size(); ++j) {
    const MovingObstacle& obstacle = defaults.obstacles[j];
    if (j) out << ',';
    out << "{\"start\":"; point(out, obstacle.start);
    out << ",\"end\":"; point(out, obstacle.center(duration));
    out << ",\"radius\":"; number(out, obstacle.radius); out << '}';
  }
  out << "],\"sigmas\":[0,3,0.3,0.1]}}\n";
}

/**
 * Solve a request: one reactive SCATE run per sigma on the same noise, with
 * the obstacles as ground truth and VICON readings every step, written as
 * one replay (scate_reactive_scenario.h writeBaseWrenchReplay). Reports
 * "PROGRESS done total label" lines on progress before each run and at the end.
 */
inline std::vector<std::pair<double, ClosedLoopResult>> runLiveRequest(
    const LiveRequest& request, const std::string& replay_path, std::ostream* progress) {
  const LiveRobot live = liveRobot(request.robot);
  LiveSetup setup = makeLiveSetup(live, request);
  for (const MovingObstacle& obstacle : request.obstacles) {
    if (!(obstacle.radius > 0.0)) throw std::invalid_argument("obstacle radius must be > 0");
  }
  const ReactiveConfig plain = liveConfig(live, request);
  std::vector<std::pair<double, ClosedLoopResult>> runs;
  const size_t total = request.sigmas.size();
  for (size_t r = 0; r < total; ++r) {
    const double sigma = request.sigmas[r];
    if (progress) *progress << "PROGRESS " << r << ' ' << total << ' ' << baseWrenchRunLabel(sigma)
                            << std::endl;
    ReactivePlanner planner(setup.spec, runConfig(plain, setup, sigma), setup.model,
                            setup.initial);
    ClosedLoopResult result =
        runReactiveClosedLoop(planner, *setup.model, liveSimulation(live, request));
    result.safety = assessRun(result, setup.spec);
    runs.emplace_back(sigma, std::move(result));
  }
  if (progress) *progress << "PROGRESS " << total << ' ' << total << " writing replay" << std::endl;
  writeBaseWrenchReplay(replay_path, live.scenario, live.urdf, setup.spec, *setup.model,
                        setup.base_wrench, request.obstacles, runs, "scate_live");
  return runs;
}

}  // namespace scate
