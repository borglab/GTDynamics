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

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtdynamics;

int main(int argc, char** argv) {
  // Load the robot and build a nonlinear factor graph of kinodynamics
  // constraints.
  auto simple_rpr = CreateRobotFromFile(
      kSdfPath + std::string("test/simple_rpr.sdf"), "simple_rpr_sdf");
  std::cout << "\033[1;31m"
            << "Robot Model:"
            << "\033[0m\n"
            << std::endl;
  simple_rpr.print();

  // Simulation parameters
  int T = 15;
  const double dt = 0.1;
  gtsam::Vector3 gravity(0, 0, 0);

  // Build graph
  auto graph_builder = DynamicsGraph(gravity);
  auto kdfg =
      graph_builder.trajectoryFG(simple_rpr, T, dt, CollocationScheme::Euler);

  // Specify the forward dynamics priors and add them to the factor graph.
  gtsam::Values theta_and_theta_dot;

  int j1 = simple_rpr.joint("joint_1")->id(),
      j2 = simple_rpr.joint("joint_2")->id(),
      j3 = simple_rpr.joint("joint_3")->id();
  InsertJointAngle<double>(&theta_and_theta_dot, j1, 0, 0.0);
  InsertJointAngle<double>(&theta_and_theta_dot, j2, 0, 0.0);
  InsertJointAngle<double>(&theta_and_theta_dot, j3, 0, 0.0);
  InsertJointVel<double>(&theta_and_theta_dot, j1, 0, 0.3);
  InsertJointVel<double>(&theta_and_theta_dot, j2, 0, 0.1);
  InsertJointVel<double>(&theta_and_theta_dot, j3, 0, 0.0);

  std::vector<gtsam::Vector> taus;
  for (int t = 0; t <= T; t++) {
    taus.push_back(gtsam::Vector::Zero(3));
  }
  auto fd_priors =
      graph_builder.trajectoryFDPriors(simple_rpr, T, theta_and_theta_dot);
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
    std::cout << Pose(results, 0, t).translation().transpose() << std::endl;
    std::cout << Pose(results, 1, t).translation().transpose() << std::endl;
    std::cout << Pose(results, 2, t).translation().transpose() << std::endl;
    std::cout << Pose(results, 3, t).translation().transpose() << std::endl;
    std::cout << std::endl;
  }

  // export
  graph_builder.saveGraphMultiSteps("gerry00_result.json", kdfg, results,
                                    simple_rpr, T);

  return 0;
}
