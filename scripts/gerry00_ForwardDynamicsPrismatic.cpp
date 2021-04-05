/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  gerry00_ForwardDynamicsPrismatic.cpp
 * @brief Test forward dynamics optimization for an rpr robot
 * @authors Frank Dellaert, Alejandro Escontrela, Gerry Chen
 */

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>

using namespace gtdynamics;

int main(int argc, char** argv) {
  // Load the robot and build a nonlinear factor graph of kinodynamics
  // constraints.
  auto simple_rpr = CreateRobotFromFile(
      SDF_PATH + "/test/simple_rpr.sdf", "simple_rpr_sdf");
  std::cout << "\033[1;31m"
            << "Robot Model:"
            << "\033[0m\n"
            << std::endl;
  simple_rpr.print();

  // Simulation parameters
  int T = 15;
  const double dt = 0.1;

  // Build graph
  auto graph_builder = DynamicsGraph();
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, 0).finished();
  auto kdfg = graph_builder.trajectoryFG(
      simple_rpr, T, dt, DynamicsGraph::CollocationScheme::Euler, gravity);

  // Specify the forward dynamics priors and add them to the factor graph.
  gtsam::Vector theta = (gtsam::Vector(3) << 0, 0, 0).finished();
  gtsam::Vector theta_dot = (gtsam::Vector(3) << 0.3, 0.1, 0).finished();
  std::vector<gtsam::Vector> taus;
  for (int t = 0; t <= T; t++) {
    taus.push_back((gtsam::Vector(3) << 0, 0, 0).finished());
  }
  auto fd_priors =
      graph_builder.trajectoryFDPriors(simple_rpr, T, theta, theta_dot, taus);
  kdfg.add(fd_priors);

  // Initialize solution.
  auto init_values = ZeroValuesTrajectory(simple_rpr, T, 0);

  // Compute the forward dynamics.
  gtsam::LevenbergMarquardtOptimizer optimizer(kdfg, init_values);
  gtsam::Values results = optimizer.optimize();
  // graph_builder.printValues(results);
  std::cout << "\033[1;31m"
            << "Joint Angles:"
            << "\033[0m" << std::endl;
  for (int t = 0; t <= T; t++) {
    std::cout << graph_builder.jointAngles(simple_rpr, results, t) << std::endl
              << std::endl;
  }
  std::cout << "\033[1;31m"
            << "Link Poses:"
            << "\033[0m\n"
            << std::endl;
  for (int t = 0; t <= T; t++) {
    std::cout
        << Pose(results, 0, t).translation().transpose()
        << std::endl;
    std::cout
        << Pose(results, 1, t).translation().transpose()
        << std::endl;
    std::cout
        << Pose(results, 2, t).translation().transpose()
        << std::endl;
    std::cout
        << Pose(results, 3, t).translation().transpose()
        << std::endl;
    std::cout << std::endl;
  }

  // export
  graph_builder.saveGraphMultiSteps("gerry00_result.json", kdfg, results,
                                    simple_rpr, T);

  return 0;
}
