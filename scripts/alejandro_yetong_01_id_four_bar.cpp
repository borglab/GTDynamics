/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  quadruped_pose_control.cpp
 * @brief Compute the inverse dynamics of a four-bar linkage.
 * @Author: Alejandro Escontrela and Yetong Zhang
 */

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>


#include <iostream>
#include <fstream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/RobotModels.h"
#include "gtdynamics/factors/MinTorqueFactor.h"
#include "gtdynamics/utils/initialize_solution_utils.h"

int main(int argc, char** argv) {
  using four_bar_linkage::my_robot, four_bar_linkage::planar_axis,
    four_bar_linkage::joint_angles, four_bar_linkage::joint_vels;

  gtsam::Vector3 gravity(0, -10, 0);
  my_robot.getLinkByName("l1")->fix();

  std::cout << "\033[1;32;7;4mParsed Robot:\033[0m" << std::endl;
  my_robot.printRobot();
  std::cout << "-------------" << std::endl;

  // Build the dynamics graph.
  auto graph_builder = gtdynamics::DynamicsGraph();
  gtsam::NonlinearFactorGraph graph = graph_builder.dynamicsFactorGraph(
      my_robot, 0, gravity, planar_axis);

  // Inverse dynamics priors. We care about the torques.
  gtsam::Vector joint_accels = gtsam::Vector::Zero(my_robot.numJoints());
  gtsam::NonlinearFactorGraph prior_factors =
      graph_builder.inverseDynamicsPriors(my_robot, 0, joint_angles, joint_vels,
                                          joint_accels);

  // Pose and twist priors. Assume robot initially stationary.
  for (auto link : my_robot.links()) {
    int i = link->getID();
    prior_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtdynamics::PoseKey(i, 0), link->wTcom(),
        gtsam::noiseModel::Constrained::All(6)));
    prior_factors.add(gtsam::PriorFactor<gtsam::Vector6>(
        gtdynamics::TwistKey(i, 0), gtsam::Vector6::Zero(),
        gtsam::noiseModel::Constrained::All(6)));
  }
  graph.add(prior_factors);

  // Add min torque factor to each joint. This factor minimizes torque squared.
  for (auto joint : my_robot.joints())
    graph.add(
        gtdynamics::MinTorqueFactor(gtdynamics::TorqueKey(joint->getID(), 0),
            gtsam::noiseModel::Gaussian::Covariance(gtsam::I_1x1)));

  // Initialize solution.
  gtsam::Values init_values = gtdynamics::ZeroValues(my_robot, 0);

  std::cout << "\033[1;32;7mFactor Graph Optimization:\033[0m" << std::endl;
  graph_builder.printGraph(graph);

  // Optimize! and calculate error.
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  params.setAbsoluteErrorTol(1e-12);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gtsam::Values results = optimizer.optimize();

  std::cout << "\033[1;31mError: " << graph.error(results) << "\033[0m"
            << std::endl << "-------------" << std::endl;

  // Save fg visualization.
  // graph_builder.saveGraph("../../visualization/factor_graph.json", graph,
  //                         results, my_robot, 0, true);

  std::cout << "\033[1;32;7;4mJoint Torques: \033[0m" << std::endl;
  for (auto joint : my_robot.joints())
      std::cout << "\t'" << joint->name() << "': " << results.atDouble(
          gtdynamics::TorqueKey(joint->getID(), 0)) << "," << std::endl;

  return 0;
}
