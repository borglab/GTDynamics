/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file scate_init.cpp
 * @author Harrison Jenkins
 *
 * Manipulator SCATE, offline plan (paper Algorithm 1, "Generate Initial Plan"):
 *   1. load the robot and solve IK for the goal          (scate_init.h)
 *   2. build the initial trajectory guess                 (scate_init.h)
 *   3. build the factor graph from a GraphSpec            (build_scate_graph.h)
 *   4. solve with Levenberg-Marquardt                     (build_scate_graph.h)
 *   5. check the solution and write both trajectories to CSV
 *
 * Usage:  ./scate_init [/path/to/ur5.urdf]
 * Without an argument it loads the UR5 model that ships with GTDynamics.
 */

#include <iostream>
#include <stdexcept>
#include <string>

#include "build_scate_graph.h"
#include "scate_init.h"

namespace {

/// Robot file: the first argument, else the UR5 model that ships with GTDynamics.
std::string robotFilePath(int argc, char** argv) {
  return argc >= 2 ? std::string(argv[1]) : scate::ur5UrdfPath();
}

/// Build the graph, solve it from the initial guess and return the solution.
gtsam::Values solveOffline(const scate::GraphSpec& spec, const gtsam::Values& initial) {
  const gtsam::NonlinearFactorGraph graph = scate::buildScateGraph(spec);
  std::cout << "\nGraph: " << graph.size() << " factors (expected "
            << scate::expectedFactorCount(spec.num_poses) << "), "
            << initial.size() << " variables\n";

  const double initial_error = graph.error(initial);
  const gtsam::Values result = scate::optimizeGraph(graph, initial, /*verbose=*/true);
  std::cout << "[diag] graph error: initial " << initial_error << ", final "
            << graph.error(result) << "\n";
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    // ---- 1. Robot file -------------------------------------------------
    const std::string path = robotFilePath(argc, argv);
    std::cout << "Robot file: " << path << "\n";

    const gtdynamics::Robot robot = scate::loadRobot(path);
    const gtdynamics::Kinematics kinematics = scate::makeKinematics();
    std::cout << "Loaded robot with " << robot.links().size() << " links and "
              << robot.joints().size() << " joints\n";

    // ---- 2. Start, goal (IK), and initial guess -------------------------
    const scate::InitialProblem problem = scate::initializeProblem(kinematics, robot);
    scate::report(kinematics, robot, problem, "initial_trajectory.csv");

    // ---- 3-4. Factor graph and solve --------------------------------------
    const scate::GraphSpec spec = scate::makeGraphSpec(robot, problem);
    const gtsam::Values result = solveOffline(spec, problem.values);

    // ---- 5. Evaluate and write the solved trajectory ----------------------
    const scate::SolutionMetrics metrics = scate::evaluateSolution(result, spec);

    // report() reads the trajectory, values and the goal pose, so reuse it on
    // a copy of the problem holding the optimized trajectory.
    scate::InitialProblem solved = problem;
    solved.trajectory = scate::extractTrajectory(result, spec.num_poses);
    solved.values = result;
    std::cout << "\n=== Optimized trajectory (report labels it as a guess) ===";
    scate::report(kinematics, robot, solved, "optimized_trajectory.csv");
    return metrics.passed ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "scate_init failed: " << e.what() << "\n";
    return 1;
  }
}
