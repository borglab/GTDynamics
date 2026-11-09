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
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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
// 2. Reporting
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

/// Reactive, frozen and blind runs into a replay file (its static "obstacle" field holds obstacle 0).
inline void writeReactiveReplay(const std::string& path, const ReactiveScenario& scenario,
                                const std::string& robot_file, unsigned seed,
                                const GraphSpec& spec, const SphereCollisionModel& model,
                                const std::vector<MovingObstacle>& obstacles,
                                const ClosedLoopResult& reactive, const ClosedLoopResult& frozen,
                                const ClosedLoopResult& blind) {
  ReplayScenario replay;
  replay.source = scenario.program + " (GTDynamics " + scenario.robot_name + "), robot " +
                  robot_file + ", seed " + std::to_string(seed);
  replay.obstacle_center = obstacles.front().start;
  replay.obstacle_radius = obstacles.front().radius;
  replay.moving_obstacles = obstacles;
  replay.obstacle_standoff = model.params().obstacle_epsilon;
  replay.self_standoff = model.params().self_epsilon;
  OnlineConfig planner_radius;  // only its obstacle_radius is written
  planner_radius.obstacle_radius = obstacles.front().radius;
  writeReplayJson(path, replay, spec, planner_radius, model,
                  {{"online", "Reactive SCATE", &reactive, 0.0},
                   {"frozen", "Frozen (obstacles seen at the start only)", &frozen, 0.0},
                   {"blind", "Blind (no obstacle sensing)", &blind, 0.0}});
}

// ============================================================
// 3. Setup and run
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
  const ReactiveConfig config = reactiveConfig(scenario);

  // Offline plan with no obstacle known; the obstacles cross it.
  ReactivePlanner probe(spec, config, model, problem.values);
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
  writeReactiveReplay("reactive_replay.json", scenario, robot_file, seed, spec, *model, obstacles,
                      reactive, frozen, blind);
  std::cout << "Wrote reactive_trajectory.csv, reactive_frozen.csv, reactive_blind.csv and "
               "reactive_replay.json\n";
  return reactive.collided() ? 1 : 0;
}

}  // namespace scate
