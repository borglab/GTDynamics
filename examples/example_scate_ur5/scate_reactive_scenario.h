/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_reactive_scenario.h
 * @author Harrison Jenkins
 *
 * The reactive SCATE example (scate_reactive.h) as a function, for any arm
 * with a sphere model and motion-capture markers:
 *   1. initialize: start pose, IK for the goal, initial trajectory  (scate_init.h)
 *   2. plan with no obstacle known, and place moving obstacles so that each
 *      crosses that plan at a chosen state, through a chosen robot sphere
 *   3. run reactive SCATE in closed loop with motion-capture readings of the
 *      arm's markers and the obstacles at every state
 *   4. repeat with the obstacles seen only at the start ("frozen") and never
 *      ("blind") as negative controls, on the same noise
 *   5. write CSVs and reactive_replay.json
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "base_wrench.h"
#include "scate_init.h"
#include "scate_reactive.h"
#include "scate_replay.h"
#include "scate_scenario.h"
#include "sphere_collision.h"

namespace scate {

// ============================================================
// 1. Scenario settings
// ============================================================

/// How a moving obstacle crosses the offline plan.
struct ObstacleCrossing {
  int state = 12;            ///< the obstacle reaches the plan at this state's time
  int sphere = -1;           ///< robot sphere it crosses (-1: the tool sphere)
  gtsam::Vector3 offset = gtsam::Vector3(0, 0, 0.03);  ///< m, from that sphere's center
  /**
   * Direction of motion: "inward" (horizontal, toward the base's vertical
   * axis), "outward", or "down".
   */
  std::string direction = "inward";
  double speed = 0.03;   ///< m/s
  double radius = 0.08;  ///< m
};

/// Everything about the reactive example except the robot and its spheres.
struct ReactiveScenario {
  std::string program;     ///< program name, for the replay's source line
  std::string robot_name;  ///< e.g. "UR10e"
  Vector goal_joints = sweepGoalJoints();     ///< FK of this is the goal pose
  std::vector<ObstacleCrossing> crossings;    ///< one per obstacle
  int checks_per_interval = 6;                ///< collision checks per interval
  Vector effort_sigmas = shoulderFirstEffortSigmas();  ///< per joint, rad/s^2
  bool use_encoders = false;  ///< also read joint encoders (default: markers only)
  bool minimize_base_wrench = false;  ///< add the base-disturbance cost (base_wrench.h)
};

/// Planner settings of the reactive scenario.
inline ReactiveConfig reactiveConfig(const ReactiveScenario& scenario) {
  ReactiveConfig config;
  for (const ObstacleCrossing& crossing : scenario.crossings) {
    config.obstacle_radii.push_back(crossing.radius);
  }
  config.tau_fractions = evenlySpacedFractions(scenario.checks_per_interval);
  config.effort_sigmas = scenario.effort_sigmas;
  return config;
}

/// Unit direction of motion of a crossing obstacle at a point.
inline gtsam::Vector3 crossingDirection(const std::string& direction, const gtsam::Point3& at) {
  if (direction == "down") return gtsam::Vector3(0, 0, -1);
  const gtsam::Vector3 radial(at.x(), at.y(), 0.0);
  if (radial.norm() < 1e-6) throw std::invalid_argument("crossingDirection: point on the base axis");
  if (direction == "inward") return -radial.normalized();
  if (direction == "outward") return radial.normalized();
  throw std::invalid_argument("crossingDirection: unknown direction '" + direction + "'");
}

/**
 * The moving obstacles: each reaches its chosen sphere's position on the
 * offline plan (plus offset) at the time of its chosen state.
 */
inline std::vector<MovingObstacle> placeCrossingObstacles(const ReactiveScenario& scenario,
                                                          const SphereCollisionModel& model,
                                                          const gtsam::Values& plan,
                                                          const GraphSpec& spec) {
  std::vector<MovingObstacle> obstacles;
  for (const ObstacleCrossing& crossing : scenario.crossings) {
    const size_t sphere = crossing.sphere < 0 ? static_cast<size_t>(model.toolSphere())
                                              : static_cast<size_t>(crossing.sphere);
    const gtsam::Point3 target =
        model.spherePosition(plan.at<Vector>(QKey(crossing.state)), sphere) + crossing.offset;
    MovingObstacle obstacle;
    obstacle.velocity = crossing.speed * crossingDirection(crossing.direction, target);
    obstacle.start = target - crossing.state * spec.dt * obstacle.velocity;
    obstacle.radius = crossing.radius;
    obstacles.push_back(obstacle);
  }
  return obstacles;
}

// ============================================================
// 2. Base disturbance of an executed run
// ============================================================

/**
 * The motion-induced wrench on the base over an executed run: what the
 * attitude control would have to cancel. Norms of the 3D torque (N m) and
 * force (N); the impulses are the time integrals of those norms (N m s, N s),
 * a proxy for attitude-control effort.
 */
struct BaseWrenchStats {
  double rms_torque = 0.0, peak_torque = 0.0, torque_impulse = 0.0;
  double rms_force = 0.0, peak_force = 0.0, force_impulse = 0.0;
};

/// Base wrench statistics of the executed motion, sampled `samples` times per interval.
inline BaseWrenchStats baseWrenchStats(const BaseWrenchModel& model,
                                       const ClosedLoopResult& result, double dt,
                                       int samples = 20) {
  BaseWrenchStats stats;
  double torque_squares = 0.0, force_squares = 0.0, duration = 0.0;
  for (size_t i = 0; i < result.u.size(); ++i) {
    const Vector& q = result.q[i];
    const Vector& v = result.v[i];
    const Vector& u = result.u[i];
    for (int s = 0; s < samples; ++s) {
      const double tau = (s + 0.5) / samples * dt;  // midpoint rule
      const gtsam::Vector6 wrench =
          model.wrench(q + tau * v + 0.5 * tau * tau * u, v + tau * u, u);
      const double torque = wrench.head<3>().norm(), force = wrench.tail<3>().norm();
      const double step = dt / samples;
      torque_squares += torque * torque * step;
      force_squares += force * force * step;
      stats.torque_impulse += torque * step;
      stats.force_impulse += force * step;
      stats.peak_torque = std::max(stats.peak_torque, torque);
      stats.peak_force = std::max(stats.peak_force, force);
      duration += step;
    }
  }
  if (duration > 0.0) {
    stats.rms_torque = std::sqrt(torque_squares / duration);
    stats.rms_force = std::sqrt(force_squares / duration);
  }
  return stats;
}


/// Append a run's executed base wrench, 10 samples per interval, to a CSV stream.
inline void writeBaseWrenchSeries(std::ofstream* out, const BaseWrenchModel& model,
                                  const ClosedLoopResult& result, double dt, double sigma) {
  const int samples = 10;
  for (size_t i = 0; i < result.u.size(); ++i) {
    for (int s = 0; s < samples; ++s) {
      const double tau = s * dt / samples;
      const Vector& q = result.q[i];
      const Vector& v = result.v[i];
      const Vector& u = result.u[i];
      const gtsam::Vector6 wrench =
          model.wrench(q + tau * v + 0.5 * tau * tau * u, v + tau * u, u);
      *out << i * dt + tau << "," << sigma;
      for (int c = 0; c < 6; ++c) *out << "," << wrench(c);
      *out << "\n";
    }
  }
}

// ============================================================
// 3. Reporting
// ============================================================

/// Per-step table: obstacles seen, next-interval clearance, solve time.
inline void printReactiveStepTable(const ClosedLoopResult& result) {
  std::cout << "\n--- Reactive loop ---\n";
  std::cout << " k  seen  clearance next interval (m)  ms\n";
  for (size_t k = 0; k < result.steps.size(); ++k) {
    const StepInfo& step = result.steps[k];
    const auto seen = std::count_if(step.obstacle_estimates.begin(), step.obstacle_estimates.end(),
                                    [](const auto& estimate) { return estimate.has_value(); });
    std::cout << std::setw(2) << k << "  " << seen << "/" << step.obstacle_estimates.size()
              << "   ";
    if (k < result.obstacle_clearance.size()) {
      std::cout << std::fixed << std::setprecision(3) << std::setw(8)
                << result.obstacle_clearance[k];
    } else {
      std::cout << "        ";
    }
    std::cout << "                      " << std::setprecision(1) << step.solve_ms << "\n";
    std::cout.unsetf(std::ios::floatfield);
    std::cout << std::setprecision(6);
  }
}

/// Summary of the three runs.
inline void printReactiveEvaluation(const ClosedLoopResult& reactive,
                                    const ClosedLoopResult& frozen,
                                    const ClosedLoopResult& blind, double standoff) {
  std::cout << "\n--- Closed-loop evaluation ---\n";
  const auto line = [&](const char* name, const ClosedLoopResult& result) {
    std::cout << name << ": min obstacle clearance " << result.min_obstacle_clearance
              << " m (per obstacle";
    for (double gap : result.min_clearance_per_obstacle) std::cout << " " << gap;
    std::cout << "), min self clearance " << result.min_self_clearance << " m, goal error "
              << result.goal_error << " rad" << (result.collided() ? "  COLLISION" : "") << "\n";
  };
  line("[diag] reactive", reactive);
  line("[diag] frozen  ", frozen);
  line("[diag] blind   ", blind);
  std::cout << "[diag] standoff " << standoff << " m, reactive slowest step "
            << reactive.max_solve_ms << " ms\n";
  std::cout << (reactive.collided() ? "RESULT: COLLISION" : "RESULT: no collision")
            << (frozen.collided() && blind.collided()
                    ? " (frozen and blind robots collide, as expected)\n"
                    : " (WARNING: a negative control does not collide)\n");
}

/**
 * Reactive, frozen and blind runs into a replay file (its static "obstacle"
 * field holds obstacle 0), with the base wrench in every frame.
 */
inline void writeReactiveReplay(const std::string& path, const ReactiveScenario& scenario,
                                const std::string& robot_file, unsigned seed,
                                const GraphSpec& spec, const SphereCollisionModel& model,
                                const std::shared_ptr<const BaseWrenchModel>& base_wrench,
                                const std::vector<MovingObstacle>& obstacles,
                                const ClosedLoopResult& reactive, const ClosedLoopResult& frozen,
                                const ClosedLoopResult& blind) {
  ReplayScenario replay;
  replay.robot_name = scenario.robot_name;
  replay.source = scenario.program + " (GTDynamics " + scenario.robot_name + "), robot " +
                  robot_file + ", seed " + std::to_string(seed);
  replay.obstacle_center = obstacles.front().start;
  replay.obstacle_radius = obstacles.front().radius;
  replay.moving_obstacles = obstacles;
  replay.obstacle_standoff = model.params().obstacle_epsilon;
  replay.self_standoff = model.params().self_epsilon;
  replay.base_wrench = [base_wrench](const Vector& q, const Vector& v, const Vector& a) {
    return base_wrench->wrench(q, v, a);
  };
  OnlineConfig planner_radius;  // only its obstacle_radius is written
  planner_radius.obstacle_radius = obstacles.front().radius;
  writeReplayJson(path, replay, spec, planner_radius, model,
                  {{"online", "Reactive SCATE", &reactive, 0.0},
                   {"frozen", "Frozen (obstacles seen at the start only)", &frozen, 0.0},
                   {"blind", "Blind (no obstacle sensing)", &blind, 0.0}});
}

// ============================================================
// 4. Setup and run
// ============================================================

/**
 * The reactive example: offline plan, moving obstacles crossing it, and the
 * reactive, frozen and blind runs. Writes reactive_trajectory.csv,
 * reactive_frozen.csv, reactive_blind.csv and reactive_replay.json. Returns the
 * program's exit code (1 if the reactive run collides).
 * @param robot_file       URDF or SDF of the arm
 * @param seed             noise seed of the simulation
 * @param scenario         goal, obstacles and planner settings
 * @param sphere_model     the arm's collision spheres and motion-capture markers
 * @param collision_params collision factor settings and SDF grid
 */
inline int runReactiveExample(const std::string& robot_file, unsigned seed,
                              const ReactiveScenario& scenario, const SphereModel& sphere_model,
                              SphereCollisionParams collision_params) {
  if (scenario.crossings.empty()) throw std::invalid_argument("runReactiveExample: no obstacles");
  if (sphere_model.markers.empty()) {
    throw std::invalid_argument("runReactiveExample: the sphere model has no markers");
  }
  std::cout << "Robot file: " << robot_file << ", seed " << seed << "\n";
  const gtdynamics::Robot robot = loadRobot(robot_file);
  const gtdynamics::Kinematics kinematics = makeKinematics();
  const InitialProblem problem =
      initializeProblem(kinematics, robot, startJoints(), scenario.goal_joints);
  const GraphSpec spec = makeGraphSpec(robot, problem);
  collision_params.dt = spec.dt;
  const auto model = makeCollisionModel(robot, spec, sphere_model, collision_params);
  std::cout << "Markers: " << model->numMarkers() << "\n";
  ReactiveConfig config = reactiveConfig(scenario);
  const auto base_wrench = std::make_shared<const BaseWrenchModel>(robot);
  if (scenario.minimize_base_wrench) {
    config.base_wrench = base_wrench;
    std::cout << "Base-disturbance cost on: sigma " << config.sigma_base_torque << " N m, "
              << config.sigma_base_force << " N\n";
  }

  // Offline plan with no obstacle known; the obstacles cross it. The plan is
  // made without the base-wrench cost, so the scenario is the same either way.
  ReactivePlanner probe(spec, reactiveConfig(scenario), model, problem.values);
  const StepInfo offline = probe.planOffline();
  std::cout << "\n--- Offline plan (no obstacle known) ---\n";
  std::cout << "[diag] offline: " << offline.num_factors << " factors, graph error "
            << offline.graph_error << ", " << offline.solve_ms << " ms\n";
  evaluateSolution(probe.values(), spec);
  const std::vector<MovingObstacle> obstacles =
      placeCrossingObstacles(scenario, *model, probe.values(), spec);
  for (size_t j = 0; j < obstacles.size(); ++j) {
    std::cout << "Obstacle " << j << ": start " << obstacles[j].start.transpose()
              << ", velocity " << obstacles[j].velocity.transpose() << " m/s, radius "
              << obstacles[j].radius << " m\n";
  }

  ViconSimulationConfig simulation;
  simulation.obstacles = obstacles;
  simulation.use_encoders = scenario.use_encoders;
  simulation.seed = seed;
  const auto run = [&](ObstacleSensing sensing) {
    ReactivePlanner planner(spec, config, model, problem.values);
    ViconSimulationConfig sensed = simulation;
    sensed.sensing = sensing;
    return runReactiveClosedLoop(planner, *model, sensed);
  };
  const ClosedLoopResult blind = run(ObstacleSensing::kNone);
  const ClosedLoopResult frozen = run(ObstacleSensing::kFirstStepOnly);
  const ClosedLoopResult reactive = run(ObstacleSensing::kEveryStep);
  writeClosedLoopCsv("reactive_trajectory.csv", reactive, *model);
  writeClosedLoopCsv("reactive_frozen.csv", frozen, *model);
  writeClosedLoopCsv("reactive_blind.csv", blind, *model);

  printReactiveStepTable(reactive);
  printReactiveEvaluation(reactive, frozen, blind, collision_params.obstacle_epsilon);
  const BaseWrenchStats base = baseWrenchStats(*base_wrench, reactive, spec.dt);
  std::cout << "[diag] base disturbance (reactive run): torque rms " << base.rms_torque
            << " N m, peak " << base.peak_torque << " N m, integral " << base.torque_impulse
            << " N m s; force rms " << base.rms_force << " N, peak " << base.peak_force << " N\n";
  writeReactiveReplay("reactive_replay.json", scenario, robot_file, seed, spec, *model,
                      base_wrench, obstacles, reactive, frozen, blind);
  std::cout << "Wrote reactive_trajectory.csv, reactive_frozen.csv, reactive_blind.csv and "
               "reactive_replay.json\n";
  return reactive.collided() ? 1 : 0;
}

// ============================================================
// 5. Base-disturbance study
// ============================================================

/// One row of the base-disturbance study: a cost weight and the runs' averages.
struct BaseWrenchStudyRow {
  double sigma = 0.0;  ///< N m and N; 0 = cost off
  BaseWrenchStats mean;               ///< averaged over seeds
  double worst_clearance = 1e9;       ///< m, over seeds
  double worst_goal_error = 0.0;      ///< rad, over seeds
  double slowest_step_ms = 0.0;       ///< over seeds
  int collisions = 0;
};

/**
 * Seed 0 of several base-wrench sigmas as one replay: each run is named by
 * baseWrenchRunName, frames carry the base wrench, and the primary
 * ("online", the default sigma) run comes first. program names the writer
 * in the replay's source line.
 */
inline void writeBaseWrenchReplay(const std::string& path, const ReactiveScenario& scenario,
                                  const std::string& robot_file, const GraphSpec& spec,
                                  const SphereCollisionModel& model,
                                  const std::shared_ptr<const BaseWrenchModel>& base_wrench,
                                  const std::vector<MovingObstacle>& obstacles,
                                  std::vector<std::pair<double, ClosedLoopResult>> runs,
                                  const std::string& program = "scate_ur10e_base_wrench");

/// Replay run name of a base-wrench sigma: the default is the primary ("online") run.
inline std::string baseWrenchRunName(double sigma) {
  if (sigma <= 0.0) return "cost_off";
  if (sigma == ReactiveConfig().sigma_base_torque) return "online";
  std::ostringstream name;
  name << "sigma_" << sigma;
  return name.str();
}

/// Replay label of a base-wrench sigma.
inline std::string baseWrenchRunLabel(double sigma) {
  if (sigma <= 0.0) return "Base cost off";
  std::ostringstream label;
  label << "\xCF\x83 = " << sigma;  // sigma
  if (sigma == ReactiveConfig().sigma_base_torque) label << " (default)";
  return label.str();
}

/**
 * Reactive SCATE with the base-wrench cost at each sigma (the same value for
 * torque in N m and force in N; 0 = off), over seeds 0..num_seeds-1, on the
 * same noise. Obstacles cross the plan as in the scenario (none if it has
 * no crossings). Returns one row per sigma; seed 0's executed base wrench of
 * every sigma is written to series_path (time, sigma, torque xyz, force xyz).
 * If replay_path is not empty, seed 0 of each of replay_sigmas is written
 * there as one replay, one run per sigma (the default sigma is the primary
 * "online" run); every replay sigma must also be in sigmas.
 */
inline std::vector<BaseWrenchStudyRow> runBaseWrenchStudy(
    const std::string& robot_file, int num_seeds, const ReactiveScenario& scenario,
    const SphereModel& sphere_model, SphereCollisionParams collision_params,
    const std::vector<double>& sigmas, const std::string& series_path,
    const std::string& replay_path = "", const std::vector<double>& replay_sigmas = {}) {
  const gtdynamics::Robot robot = loadRobot(robot_file);
  const gtdynamics::Kinematics kinematics = makeKinematics();
  const InitialProblem problem =
      initializeProblem(kinematics, robot, startJoints(), scenario.goal_joints);
  const GraphSpec spec = makeGraphSpec(robot, problem);
  collision_params.dt = spec.dt;
  const auto model = std::make_shared<SphereCollisionModel>(robot, sphere_model, collision_params);
  const auto base_wrench = std::make_shared<const BaseWrenchModel>(robot);

  // Obstacles cross the plan made without the base-wrench cost.
  const ReactiveConfig plain = reactiveConfig(scenario);
  ReactivePlanner probe(spec, plain, model, problem.values);
  probe.planOffline();
  const std::vector<MovingObstacle> obstacles =
      placeCrossingObstacles(scenario, *model, probe.values(), spec);

  std::ofstream series(series_path);
  series << "t,sigma,torque_x,torque_y,torque_z,force_x,force_y,force_z\n";
  std::vector<BaseWrenchStudyRow> rows;
  std::vector<std::pair<double, ClosedLoopResult>> replay_runs;  // seed 0
  for (double sigma : sigmas) {
    BaseWrenchStudyRow row;
    row.sigma = sigma;
    for (int seed = 0; seed < num_seeds; ++seed) {
      ReactiveConfig config = plain;
      if (sigma > 0.0) {
        config.base_wrench = base_wrench;
        config.sigma_base_torque = sigma;
        config.sigma_base_force = sigma;
      }
      ReactivePlanner planner(spec, config, model, problem.values);
      ViconSimulationConfig simulation;
      simulation.obstacles = obstacles;
      simulation.use_encoders = scenario.use_encoders;
      simulation.seed = static_cast<unsigned>(seed);
      const ClosedLoopResult result = runReactiveClosedLoop(planner, *model, simulation);
      const BaseWrenchStats stats = baseWrenchStats(*base_wrench, result, spec.dt);
      row.mean.rms_torque += stats.rms_torque / num_seeds;
      row.mean.peak_torque += stats.peak_torque / num_seeds;
      row.mean.torque_impulse += stats.torque_impulse / num_seeds;
      row.mean.rms_force += stats.rms_force / num_seeds;
      row.mean.peak_force += stats.peak_force / num_seeds;
      row.mean.force_impulse += stats.force_impulse / num_seeds;
      if (!obstacles.empty()) {
        row.worst_clearance = std::min(row.worst_clearance, result.min_obstacle_clearance);
      }
      row.worst_goal_error = std::max(row.worst_goal_error, result.goal_error);
      row.slowest_step_ms = std::max(row.slowest_step_ms, result.max_solve_ms);
      row.collisions += result.collided() ? 1 : 0;
      if (seed == 0) {
        writeBaseWrenchSeries(&series, *base_wrench, result, spec.dt, sigma);
        if (std::find(replay_sigmas.begin(), replay_sigmas.end(), sigma) != replay_sigmas.end()) {
          replay_runs.emplace_back(sigma, result);
        }
      }
    }
    rows.push_back(row);
  }
  if (!replay_path.empty()) {
    writeBaseWrenchReplay(replay_path, scenario, robot_file, spec, *model, base_wrench, obstacles,
                          replay_runs);
  }
  return rows;
}

inline void writeBaseWrenchReplay(const std::string& path, const ReactiveScenario& scenario,
                                  const std::string& robot_file, const GraphSpec& spec,
                                  const SphereCollisionModel& model,
                                  const std::shared_ptr<const BaseWrenchModel>& base_wrench,
                                  const std::vector<MovingObstacle>& obstacles,
                                  std::vector<std::pair<double, ClosedLoopResult>> runs,
                                  const std::string& program) {
  if (runs.empty()) throw std::invalid_argument("writeBaseWrenchReplay: no runs");
  // The primary run first, then the others in the order given.
  std::stable_partition(runs.begin(), runs.end(), [](const auto& run) {
    return baseWrenchRunName(run.first) == "online";
  });
  ReplayScenario replay;
  replay.source = program + " (GTDynamics " + scenario.robot_name +
                  "), base-disturbance cost comparison, robot " + robot_file + ", seed 0";
  replay.robot_name = scenario.robot_name;
  if (!obstacles.empty()) {
    replay.obstacle_center = obstacles.front().start;
    replay.obstacle_radius = obstacles.front().radius;
    replay.moving_obstacles = obstacles;
  }
  replay.obstacle_standoff = model.params().obstacle_epsilon;
  replay.self_standoff = model.params().self_epsilon;
  replay.base_wrench = [base_wrench](const Vector& q, const Vector& v, const Vector& a) {
    return base_wrench->wrench(q, v, a);
  };
  OnlineConfig planner_radius;  // only its obstacle_radius is written
  if (!obstacles.empty()) planner_radius.obstacle_radius = obstacles.front().radius;
  std::vector<ReplayRun> replay_runs;
  for (const auto& run : runs) {
    replay_runs.push_back({baseWrenchRunName(run.first), baseWrenchRunLabel(run.first),
                           &run.second, 0.0});
  }
  writeReplayJson(path, replay, spec, planner_radius, model, replay_runs);
}

}  // namespace scate
