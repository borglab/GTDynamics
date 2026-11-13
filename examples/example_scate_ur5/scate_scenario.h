/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_scenario.h
 * @author Harrison Jenkins
 *
 * The two SCATE example programs as functions, so every arm runs the same
 * scenario and only its robot file and sphere model differ:
 *
 * runOfflineExample (scate_init):
 *   1. initialize: start pose, IK for the goal, initial trajectory  (scate_init.h)
 *   2. build the offline graph, solve, evaluate, write CSVs
 *
 * runOnlineExample (scate_online, paper Algorithm 1):
 *   1. initialize, as above, with a long shoulder-pan sweep as the goal
 *   2. offline plan with self-collision factors; the obstacle is unknown
 *   3. place a spherical obstacle on that plan's tool path, out of sensor
 *      range at the start
 *   4. run the closed loop: execute one control with process noise, measure
 *      joints (and the obstacle once in range), update the graph, re-solve
 *   5. repeat with a blind sensor as a negative control, and write CSVs and
 *      replay.json
 */

#pragma once

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "build_scate_graph.h"
#include "scate_init.h"
#include "scate_online.h"
#include "scate_replay.h"
#include "sphere_collision.h"

namespace scate {

// ============================================================
// 1. Offline example (scate_init)
// ============================================================

/// Build the offline graph, solve it from the initial guess and return the solution.
inline gtsam::Values solveOffline(const GraphSpec& spec, const gtsam::Values& initial) {
  const gtsam::NonlinearFactorGraph graph = buildScateGraph(spec);
  std::cout << "\nGraph: " << graph.size() << " factors (expected "
            << expectedFactorCount(spec.num_poses) << "), " << initial.size()
            << " variables\n";

  const double initial_error = graph.error(initial);
  const gtsam::Values result = optimizeGraph(graph, initial, /*verbose=*/true);
  std::cout << "[diag] graph error: initial " << initial_error << ", final "
            << graph.error(result) << "\n";
  return result;
}

/**
 * The offline example: load the robot, plan from startJoints() to the pose
 * of hiddenGoalJoints() without obstacles, and write initial_trajectory.csv
 * and optimized_trajectory.csv. Returns the program's exit code (0 if the
 * solution passes evaluateSolution).
 */
inline int runOfflineExample(const std::string& robot_file) {
  std::cout << "Robot file: " << robot_file << "\n";
  const gtdynamics::Robot robot = loadRobot(robot_file);
  const gtdynamics::Kinematics kinematics = makeKinematics();
  std::cout << "Loaded robot with " << robot.links().size() << " links and "
            << robot.joints().size() << " joints\n";

  // Start, goal (IK), and initial guess.
  const InitialProblem problem = initializeProblem(kinematics, robot);
  report(kinematics, robot, problem, "initial_trajectory.csv");

  // Factor graph and solve.
  const GraphSpec spec = makeGraphSpec(robot, problem);
  const gtsam::Values result = solveOffline(spec, problem.values);
  const SolutionMetrics metrics = evaluateSolution(result, spec);

  // report() reads the trajectory, values and the goal pose, so reuse it on
  // a copy of the problem holding the optimized trajectory.
  InitialProblem solved = problem;
  solved.trajectory = extractTrajectory(result, spec.num_poses);
  solved.values = result;
  std::cout << "\n=== Optimized trajectory (report labels it as a guess) ===";
  report(kinematics, robot, solved, "optimized_trajectory.csv");
  return metrics.passed ? 0 : 1;
}

// ============================================================
// 2. Online scenario settings
// ============================================================

/**
 * A 1.6 rad shoulder-pan sweep, so the tool travels far enough for the
 * obstacle to be discovered partway through (about 0.9 m on the UR5). The
 * goal is still given as a task-space pose (FK of this configuration) and
 * recovered by IK.
 */
inline Vector sweepGoalJoints() {
  Vector q(6);
  q << 1.6, -1.2, 1.3, -1.9, -1.3, 0.4;
  return q;
}

/**
 * Effort-prior sigmas per joint (rad/s^2): accelerating the elbow and wrists
 * costs 4x more than the shoulder, so avoidance prefers moving the whole arm
 * over swinging the wrist.
 */
inline Vector shoulderFirstEffortSigmas() {
  Vector sigmas(6);
  sigmas << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5;
  return sigmas;
}

/// Everything about the online example except the robot and its spheres.
struct OnlineScenario {
  std::string program;       ///< program name, for the replay's source line
  std::string robot_name;    ///< e.g. "UR5", for the replay's source line
  Vector goal_joints = sweepGoalJoints();  ///< FK of this is the goal pose
  int obstacle_state = 12;   ///< put the obstacle where the plan has the tool here
  double obstacle_lift = 0.03;    ///< m above that tool position (breaks symmetry)
  double obstacle_radius = 0.08;  ///< m
  double sensor_range = 0.4;      ///< m, sensor origin to obstacle center
  int checks_per_interval = 6;    ///< obstacle and self-collision checks per interval
  Vector effort_sigmas = shoulderFirstEffortSigmas();  ///< per joint, rad/s^2
};

// ============================================================
// 3. Online example: reporting
// ============================================================

/// One CSV row per state: q, v, tool position, clearances and planner statistics.
inline void writeClosedLoopCsv(const std::string& path, const ClosedLoopResult& result,
                               const SphereCollisionModel& model) {
  std::ofstream csv(path);
  const size_t n = result.q.front().size();
  csv << "k";
  for (size_t j = 0; j < n; ++j) csv << ",q" << j;
  for (size_t j = 0; j < n; ++j) csv << ",v" << j;
  csv << ",tool_x,tool_y,tool_z,clearance_next_interval,self_clearance,detected,"
         "l_x,l_y,l_z,solves,solve_ms,graph_error,factors\n";
  for (size_t k = 0; k < result.q.size(); ++k) {
    const StepInfo& step = result.steps[k];
    const gtsam::Point3 tool = model.spherePosition(result.q[k], model.toolSphere());
    csv << k;
    for (size_t j = 0; j < n; ++j) csv << "," << result.q[k](j);
    for (size_t j = 0; j < n; ++j) csv << "," << result.v[k](j);
    csv << "," << tool.x() << "," << tool.y() << "," << tool.z() << ","
        << (k < result.obstacle_clearance.size()
                ? std::to_string(result.obstacle_clearance[k])
                : "")
        << "," << model.selfClearance(result.q[k]) << "," << step.detected << ",";
    if (step.have_obstacle) {
      csv << step.l_estimate.x() << "," << step.l_estimate.y() << "," << step.l_estimate.z();
    } else {
      csv << ",,";
    }
    csv << "," << step.solves << "," << step.solve_ms << "," << step.graph_error << ","
        << step.num_factors << "\n";
  }
}

/// Per-step table: detection, estimate of l, next-interval clearance, solves, time.
inline void printStepTable(const ClosedLoopResult& result) {
  std::cout << "\n--- Online loop ---\n";
  std::cout << " k  seen  l_est (m)                     clearance  solves  ms\n";
  for (size_t k = 0; k < result.steps.size(); ++k) {
    const StepInfo& step = result.steps[k];
    std::cout << std::setw(2) << k << "  " << (step.detected ? "yes " : "    ") << "  ";
    if (step.have_obstacle) {
      std::cout << std::fixed << std::setprecision(3) << std::setw(7) << step.l_estimate.x()
                << " " << std::setw(7) << step.l_estimate.y() << " " << std::setw(7)
                << step.l_estimate.z() << "  ";
    } else {
      std::cout << "  (unknown)                 ";
    }
    std::cout << "      ";
    if (k < result.obstacle_clearance.size()) {
      std::cout << std::setw(8) << std::setprecision(3) << result.obstacle_clearance[k];
    } else {
      std::cout << "        ";
    }
    std::cout << "  " << step.solves << "  " << std::setprecision(1) << step.solve_ms << "\n";
    std::cout.unsetf(std::ios::floatfield);
    std::cout << std::setprecision(6);
  }
}

/// Closed-loop summary, compared with the blind negative control.
inline void printEvaluation(const ClosedLoopResult& result, const ClosedLoopResult& blind,
                            double l_error, const SphereCollisionParams& collision_params) {
  std::cout << "\n--- Closed-loop evaluation ---\n";
  std::cout << "first detection at state " << result.first_detection << "\n";
  for (const StepInfo& step : result.steps) {
    if (step.goal_clearance && *step.goal_clearance < 0.0) {
      std::cout << "WARNING: at step " << step.k << " the goal is inside the estimated obstacle ("
                << *step.goal_clearance << " m); no plan can reach it without collision\n";
      break;
    }
  }
  std::cout << "obstacle estimate error " << l_error << " m\n";
  std::cout << "[diag] min obstacle clearance " << result.min_obstacle_clearance
            << " m (standoff " << collision_params.obstacle_epsilon << "), blind robot "
            << blind.min_obstacle_clearance << " m\n";
  std::cout << "[diag] min self clearance " << result.min_self_clearance << " m\n";
  std::cout << "[diag] goal error " << result.goal_error << " rad, final speed "
            << result.final_speed << " rad/s, slowest step " << result.max_solve_ms << " ms\n";
  std::cout << (result.collided() ? "RESULT: COLLISION" : "RESULT: no collision")
            << (blind.collided() ? " (blind robot collides, as expected)\n"
                                 : " (WARNING: blind robot does not collide; the "
                                   "obstacle is not in the way)\n");
}

// ============================================================
// 4. Online example: setup and run
// ============================================================

/// Build the collision model and report its size and start/goal self clearance.
inline std::shared_ptr<SphereCollisionModel> makeCollisionModel(
    const gtdynamics::Robot& robot, const GraphSpec& spec, const SphereModel& sphere_model,
    const SphereCollisionParams& collision_params) {
  auto model = std::make_shared<SphereCollisionModel>(robot, sphere_model, collision_params);
  std::cout << "Collision model: " << model->numSpheres() << " spheres, "
            << model->numPairs() << " self-collision pairs\n";
  std::cout << "Self clearance at start " << model->selfClearance(spec.q_start)
            << " m, at goal " << model->selfClearance(spec.q_goal) << " m\n";
  if (model->selfClearance(spec.q_start) < collision_params.self_epsilon) {
    std::cout << "  WARNING: the start already violates the self-collision standoff\n";
  }
  if (model->selfClearance(spec.q_goal) < collision_params.self_epsilon) {
    std::cout << "  WARNING: the goal violates the self-collision standoff; no plan can avoid it\n";
  }
  return model;
}

/// Planner settings of the online scenario: obstacle radius, checks, effort sigmas.
inline OnlineConfig onlineConfig(const OnlineScenario& scenario) {
  OnlineConfig config;
  config.obstacle_radius = scenario.obstacle_radius;
  config.compute_l_covariance = true;  // for the replay; not part of the timing
  config.tau_fractions = evenlySpacedFractions(scenario.checks_per_interval);
  config.effort_sigmas = scenario.effort_sigmas;
  return config;
}

/**
 * Plan offline (obstacle unknown), report the plan, and return an obstacle
 * center on its tool path at scenario.obstacle_state, lifted by
 * scenario.obstacle_lift.
 */
inline gtsam::Point3 placeObstacleOnOfflinePlan(
    const GraphSpec& spec, const OnlineConfig& config,
    const std::shared_ptr<SphereCollisionModel>& model, const gtsam::Values& initial,
    const OnlineScenario& scenario) {
  OnlinePlanner probe(spec, config, model, initial);
  const StepInfo info = probe.planOffline();
  std::cout << "\n--- Offline plan (obstacle unknown) ---\n";
  std::cout << "[diag] offline: " << info.num_factors << " factors, graph error "
            << info.graph_error << ", " << info.solve_ms << " ms\n";
  evaluateSolution(probe.values(), spec);
  double self_min = 1e9, grid_min = 1e9;
  for (int i = 0; i < spec.num_poses; ++i) {
    const Vector q_i = probe.values().at<Vector>(QKey(i));
    self_min = std::min(self_min, model->selfClearance(q_i));
    grid_min = std::min(grid_min, model->gridMargin(q_i));
  }
  std::cout << "Plan self clearance >= " << self_min << " m, SDF grid margin >= "
            << grid_min << " m\n";
  const Vector q_middle = probe.values().at<Vector>(QKey(scenario.obstacle_state));
  return model->spherePosition(q_middle, model->toolSphere()) +
         gtsam::Point3(0, 0, scenario.obstacle_lift);
}

/**
 * The online example: offline plan, an obstacle on its path, the closed loop
 * and its blind negative control. Writes online_trajectory.csv,
 * online_blind.csv and replay.json. Returns the program's exit code (1 if
 * the online run collides).
 * @param robot_file       URDF or SDF of the arm
 * @param seed             noise seed of the simulation
 * @param scenario         goal, obstacle, sensor and planner settings
 * @param sphere_model     the arm's collision spheres
 * @param collision_params collision factor settings and SDF grid
 */
inline int runOnlineExample(const std::string& robot_file, unsigned seed,
                            const OnlineScenario& scenario, const SphereModel& sphere_model,
                            SphereCollisionParams collision_params) {
  std::cout << "Robot file: " << robot_file << ", seed " << seed << "\n";
  const gtdynamics::Robot robot = loadRobot(robot_file);
  const gtdynamics::Kinematics kinematics = makeKinematics();
  const InitialProblem problem =
      initializeProblem(kinematics, robot, startJoints(), scenario.goal_joints);
  const GraphSpec spec = makeGraphSpec(robot, problem);

  collision_params.dt = spec.dt;
  const auto model = makeCollisionModel(robot, spec, sphere_model, collision_params);
  const OnlineConfig config = onlineConfig(scenario);

  const gtsam::Point3 obstacle =
      placeObstacleOnOfflinePlan(spec, config, model, problem.values, scenario);
  std::cout << "Obstacle: center " << obstacle.transpose() << ", radius "
            << scenario.obstacle_radius << " m, sensor range " << scenario.sensor_range
            << " m\n";

  SimulationConfig simulation;
  simulation.obstacle_center = obstacle;
  simulation.obstacle_radius = scenario.obstacle_radius;
  simulation.sensor_range = scenario.sensor_range;
  simulation.seed = seed;

  // Negative control: same loop, sensor blind.
  OnlinePlanner blind_planner(spec, config, model, problem.values);
  SimulationConfig blind_simulation = simulation;
  blind_simulation.sensor_range = 0.0;
  const ClosedLoopResult blind = runClosedLoop(blind_planner, *model, blind_simulation);
  writeClosedLoopCsv("online_blind.csv", blind, *model);

  // The online loop.
  OnlinePlanner planner(spec, config, model, problem.values);
  const ClosedLoopResult result = runClosedLoop(planner, *model, simulation);
  writeClosedLoopCsv("online_trajectory.csv", result, *model);

  printStepTable(result);
  printEvaluation(result, blind, (planner.obstacleEstimate() - obstacle).norm(),
                  collision_params);

  ReplayScenario replay;
  replay.source = scenario.program + " (GTDynamics " + scenario.robot_name + "), robot " +
                  robot_file + ", seed " + std::to_string(seed);
  replay.obstacle_center = obstacle;
  replay.obstacle_radius = scenario.obstacle_radius;
  replay.obstacle_standoff = collision_params.obstacle_epsilon;
  replay.self_standoff = collision_params.self_epsilon;
  writeReplayJson("replay.json", replay, spec, config, *model,
                  {{"online", "SCATE online", &result, scenario.sensor_range},
                   {"blind", "Blind (no obstacle sensing)", &blind, 0.0}});
  std::cout << "Wrote online_trajectory.csv, online_blind.csv and replay.json\n";
  return result.collided() ? 1 : 0;
}

}  // namespace scate
