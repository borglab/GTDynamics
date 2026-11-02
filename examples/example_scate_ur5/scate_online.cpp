/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_online.cpp
 * @author Harrison Jenkins
 *
 * Manipulator SCATE, online (paper Algorithm 1) on the UR5:
 *   1. initialize: start pose, IK for the goal, initial trajectory  (scate_init.h)
 *   2. offline plan with self-collision factors; the obstacle is unknown
 *   3. place a spherical obstacle on that plan's tool path, out of sensor
 *      range at the start
 *   4. run the closed loop: execute one control with process noise, measure
 *      joints (and the obstacle once in range), update the graph, re-solve
 *   5. repeat with a blind sensor as a negative control, and write CSVs
 *
 * Usage:  ./scate_online [/path/to/ur5.urdf] [seed]
 * Without a path it loads the UR5 model that ships with GTDynamics.
 */

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "build_scate_graph.h"
#include "scate_init.h"
#include "scate_online.h"
#include "scate_replay.h"
#include "ur5_collision.h"

namespace {

/**
 * A 1.6 rad shoulder-pan sweep, so the tool travels about 0.9 m and the
 * obstacle can be discovered partway through. The goal is still given as a
 * task-space pose (FK of this configuration) and recovered by IK.
 */
scate::Vector sweepGoalJoints() {
  scate::Vector q(6);
  q << 1.6, -1.2, 1.3, -1.9, -1.3, 0.4;
  return q;
}

constexpr int kObstacleState = 12;        ///< put the obstacle where the plan has the tool here
constexpr double kObstacleLift = 0.03;    ///< m above that tool position (breaks symmetry)
constexpr double kObstacleRadius = 0.08;  ///< m
constexpr double kSensorRange = 0.4;      ///< m, wrist_3 to obstacle center

/// Robot file: the first argument, else the UR5 model that ships with GTDynamics.
std::string robotFilePath(int argc, char** argv) {
  return argc >= 2 ? std::string(argv[1]) : scate::ur5UrdfPath();
}

/// One CSV row per state: q, v, tool position, clearances and planner statistics.
void writeCsv(const std::string& path, const scate::ClosedLoopResult& result,
              const scate::Ur5CollisionModel& model) {
  std::ofstream csv(path);
  const size_t n = result.q.front().size();
  csv << "k";
  for (size_t j = 0; j < n; ++j) csv << ",q" << j;
  for (size_t j = 0; j < n; ++j) csv << ",v" << j;
  csv << ",tool_x,tool_y,tool_z,clearance_next_interval,self_clearance,detected,"
         "l_x,l_y,l_z,solves,solve_ms,graph_error,factors\n";
  for (size_t k = 0; k < result.q.size(); ++k) {
    const scate::StepInfo& step = result.steps[k];
    const gtsam::Point3 tool = model.spherePosition(result.q[k], scate::kUr5ToolSphere);
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

/**
 * Plan offline (obstacle unknown), report the plan, and return an obstacle
 * center on its tool path at kObstacleState, lifted by kObstacleLift.
 */
gtsam::Point3 placeObstacleOnOfflinePlan(
    const scate::GraphSpec& spec, const scate::OnlineConfig& config,
    const std::shared_ptr<scate::Ur5CollisionModel>& model, const gtsam::Values& initial) {
  scate::OnlinePlanner probe(spec, config, model, initial);
  const scate::StepInfo info = probe.planOffline();
  std::cout << "\n--- Offline plan (obstacle unknown) ---\n";
  std::cout << "[diag] offline: " << info.num_factors << " factors, graph error "
            << info.graph_error << ", " << info.solve_ms << " ms\n";
  scate::evaluateSolution(probe.values(), spec);
  double self_min = 1e9, grid_min = 1e9;
  for (int i = 0; i < spec.num_poses; ++i) {
    const scate::Vector q_i = probe.values().at<scate::Vector>(scate::QKey(i));
    self_min = std::min(self_min, model->selfClearance(q_i));
    grid_min = std::min(grid_min, model->gridMargin(q_i));
  }
  std::cout << "Plan self clearance >= " << self_min << " m, SDF grid margin >= "
            << grid_min << " m\n";
  const scate::Vector q_middle = probe.values().at<scate::Vector>(scate::QKey(kObstacleState));
  return model->spherePosition(q_middle, scate::kUr5ToolSphere) +
         gtsam::Point3(0, 0, kObstacleLift);
}

/// Per-step table: detection, estimate of l, next-interval clearance, solves, time.
void printStepTable(const scate::ClosedLoopResult& result) {
  std::cout << "\n--- Online loop ---\n";
  std::cout << " k  seen  l_est (m)                     clearance  solves  ms\n";
  for (size_t k = 0; k < result.steps.size(); ++k) {
    const scate::StepInfo& step = result.steps[k];
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
void printEvaluation(const scate::ClosedLoopResult& result,
                     const scate::ClosedLoopResult& blind, double l_error,
                     const scate::Ur5CollisionParams& collision_params) {
  std::cout << "\n--- Closed-loop evaluation ---\n";
  std::cout << "first detection at state " << result.first_detection << "\n";
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

/// Build the UR5 collision model and report its size and start/goal self clearance.
std::shared_ptr<scate::Ur5CollisionModel> makeCollisionModel(
    const gtdynamics::Robot& robot, const scate::GraphSpec& spec,
    const scate::Ur5CollisionParams& collision_params) {
  auto model = std::make_shared<scate::Ur5CollisionModel>(robot, collision_params);
  std::cout << "Collision model: " << model->numSpheres() << " spheres, "
            << model->numPairs() << " self-collision pairs\n";
  std::cout << "Self clearance at start " << model->selfClearance(spec.q_start)
            << " m, at goal " << model->selfClearance(spec.q_goal) << " m\n";
  if (model->selfClearance(spec.q_start) < collision_params.self_epsilon) {
    std::cout << "  WARNING: the start already violates the self-collision standoff\n";
  }
  return model;
}

/// Write the online and blind runs to replay.json for plot_scate.py and the viewer.
void writeReplay(const scate::ReplayScenario& scenario, const scate::GraphSpec& spec,
                 const scate::OnlineConfig& config, const scate::Ur5CollisionModel& model,
                 const scate::ClosedLoopResult& result, const scate::ClosedLoopResult& blind) {
  scate::writeReplayJson("replay.json", scenario, spec, config, model,
                         {{"online", "SCATE online", &result, kSensorRange},
                          {"blind", "Blind (no obstacle sensing)", &blind, 0.0}});
}

}  // namespace

int main(int argc, char** argv) {
  try {
    // ---- 1. Robot and problem --------------------------------------------
    const std::string path = robotFilePath(argc, argv);
    const unsigned seed = argc >= 3 ? static_cast<unsigned>(std::atoi(argv[2])) : 0u;
    std::cout << "Robot file: " << path << ", seed " << seed << "\n";

    const gtdynamics::Robot robot = scate::loadRobot(path);
    const gtdynamics::Kinematics kinematics = scate::makeKinematics();
    const scate::InitialProblem problem = scate::initializeProblem(
        kinematics, robot, scate::startJoints(), sweepGoalJoints());
    const scate::GraphSpec spec = scate::makeGraphSpec(robot, problem);

    scate::Ur5CollisionParams collision_params;
    collision_params.dt = spec.dt;
    const auto model = makeCollisionModel(robot, spec, collision_params);

    scate::OnlineConfig config;
    config.obstacle_radius = kObstacleRadius;
    config.compute_l_covariance = true;  // for the replay; not part of the timing

    // ---- 2-3. Offline plan, and an obstacle on its path -------------------
    const gtsam::Point3 obstacle =
        placeObstacleOnOfflinePlan(spec, config, model, problem.values);
    std::cout << "Obstacle: center " << obstacle.transpose() << ", radius " << kObstacleRadius
              << " m, sensor range " << kSensorRange << " m\n";

    scate::SimulationConfig simulation;
    simulation.obstacle_center = obstacle;
    simulation.obstacle_radius = kObstacleRadius;
    simulation.sensor_range = kSensorRange;
    simulation.seed = seed;

    // ---- 5a. Negative control: same loop, sensor blind --------------------
    scate::OnlinePlanner blind_planner(spec, config, model, problem.values);
    scate::SimulationConfig blind_simulation = simulation;
    blind_simulation.sensor_range = 0.0;
    const scate::ClosedLoopResult blind =
        scate::runClosedLoop(blind_planner, *model, blind_simulation);
    writeCsv("online_blind.csv", blind, *model);

    // ---- 4. The online loop ------------------------------------------------
    scate::OnlinePlanner planner(spec, config, model, problem.values);
    const scate::ClosedLoopResult result = scate::runClosedLoop(planner, *model, simulation);
    writeCsv("online_trajectory.csv", result, *model);

    printStepTable(result);
    printEvaluation(result, blind, (planner.obstacleEstimate() - obstacle).norm(),
                    collision_params);

    scate::ReplayScenario scenario;
    scenario.source = "scate_online (GTDynamics UR5), robot " + path + ", seed " +
                      std::to_string(seed);
    scenario.obstacle_center = obstacle;
    scenario.obstacle_radius = kObstacleRadius;
    scenario.obstacle_standoff = collision_params.obstacle_epsilon;
    scenario.self_standoff = collision_params.self_epsilon;
    writeReplay(scenario, spec, config, *model, result, blind);
    std::cout << "Wrote online_trajectory.csv, online_blind.csv and replay.json\n";
    return result.collided() ? 1 : 0;
  } catch (const std::exception& e) {
    std::cerr << "scate_online failed: " << e.what() << "\n";
    return 1;
  }
}
