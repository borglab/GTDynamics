/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  quadruped_pose_control.cpp
 * @brief Compute the inverse dynamics of a four-bar linkage.
 * @author Alejandro Escontrela and Yetong Zhang
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/factors/MinTorqueFactor.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

using namespace gtdynamics;

int main(int argc, char** argv) {
  using four_bar_linkage_pure::planar_axis;
  using four_bar_linkage_pure::robot;

  gtsam::Values joint_angles_vels_accels;

  gtsam::Vector3 gravity(0, -10, 0);
  robot.fixLink("l1");

  std::cout << "\033[1;32;7;4mParsed Robot:\033[0m" << std::endl;
  robot.print();
  std::cout << "-------------" << std::endl;

  // Build the dynamics graph.
  auto graph_builder = DynamicsGraph(gravity, planar_axis);
  gtsam::NonlinearFactorGraph graph =
      graph_builder.dynamicsFactorGraph(robot, 0);

  // Inverse dynamics priors. We care about the torques.
  gtsam::Vector joint_accels = gtsam::Vector::Zero(robot.numJoints());
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(robot, 0, joint_angles_vels_accels);

  // Pose and twist priors. Assume robot initially stationary.
  for (auto link : robot.links()) {
    int i = link->id();
    prior_factors.addPrior(internal::PoseKey(i, 0), link->wTcom(),
                           gtsam::noiseModel::Constrained::All(6));
    prior_factors.addPrior<gtsam::Vector6>(
        internal::TwistKey(i, 0), gtsam::Z_6x1,
        gtsam::noiseModel::Constrained::All(6));
  }
  graph.add(prior_factors);

  // Add min torque factor to each joint. This factor minimizes torque squared.
  for (auto joint : robot.joints())
    graph.add(
        MinTorqueFactor(internal::TorqueKey(joint->id(), 0),
                        gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));

  // Initialize solution.
  gtsam::Values init_values = ZeroValues(robot, 0);

  std::cout << "\033[1;32;7mFactor Graph Optimization:\033[0m" << std::endl;
  graph_builder.printGraph(graph);

  // Optimize! and calculate error.
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gtsam::Values results = optimizer.optimize();

  std::cout << "\033[1;31mError: " << graph.error(results) << "\033[0m"
            << std::endl
            << "-------------" << std::endl;

  // Save fg visualization.
  // graph_builder.saveGraph("../../visualization/factor_graph.json", graph,
  //                         results, robot, 0, true);

  std::cout << "\033[1;32;7;4mJoint Torques: \033[0m" << std::endl;
  for (auto joint : robot.joints())
    std::cout << "\t'" << joint->name()
              << "': " << Torque(results, joint->id(), 0) << "," << std::endl;

  return 0;
}
