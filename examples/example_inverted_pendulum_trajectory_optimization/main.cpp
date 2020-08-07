/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for an inverted pendulum.
 * @Author: Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

using namespace gtdynamics; 
using gtsam::noiseModel::Isotropic;

int main(int argc, char** argv) {
  // Load the inverted pendulum.
  auto ip = CreateRobotFromFile("../inverted_pendulum.urdf");
  auto j1_id = ip.getJointByName("j1")->getID();
  ip.getLinkByName("l1")->fix();
  ip.printRobot();le
  gtsam::Vector3 gravity(0, 0, -9.8), planar_axis(1, 0, 0);

  double T = 3, dt = 1. / 100;  // Time horizon (s) and timestep duration (s).
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  // Noise models:
  auto dynamics_model = Isotropic::Sigma(1, 1e-5);  // Dynamics constraints.
  auto objectives_model = Isotropic::Sigma(1, 1e-2);  // Objectives.
  auto control_model = Isotropic::Sigma(1, 1e-1);  // Controls.

  // Specify initial conditions and goal state.
  double theta_i = 0, dtheta_i = 0, ddtheta_i = 0;
  double theta_T = M_PI, dtheta_T = 0, ddtheta_T = 0;

  // Create trajectory factor graph.
  auto graph_builder = DynamicsGraph();
  auto graph = graph_builder.trajectoryFG(
    ip, t_steps, dt, DynamicsGraph::CollocationScheme::Trapezoidal,
    gravity, planar_axis);

  // Add initial conditions to trajectory factor graph.
  graph.addPrior(JointAngleKey(j1_id, 0), theta_i, dynamics_model);
  graph.addPrior(JointVelKey(j1_id, 0), dtheta_i, dynamics_model);

  // Add state and min torque objectives to trajectory factor graph.
  graph.addPrior(JointVelKey(j1_id, t_steps), dtheta_T, objectives_model);
  graph.addPrior(JointAccelKey(j1_id, t_steps), dtheta_T, objectives_model);
  bool apply_theta_objective_all_dt = false;
  graph.addPrior(JointAngleKey(j1_id, t_steps), theta_T, objectives_model);
  if (apply_theta_objective_all_dt) {
    for (int t = 0; t < t_steps; t++)
      graph.addPrior(JointAngleKey(j1_id, t), theta_T, objectives_model);
  }
  for (int t = 0; t <= t_steps; t++)
    graph.emplace_shared<MinTorqueFactor>(TorqueKey(j1_id, t), control_model);

  // Initialize solution.
  auto init_vals = ZeroValuesTrajectory(ip, t_steps, 0, 0.0);
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::ofstream traj_file;
  traj_file.open("../traj.csv");
  traj_file << "t,theta,dtheta,ddtheta,tau" << "\n";
  double t_elapsed = 0;
  for (int t = 0; t <= t_steps; t++, t_elapsed += dt) {
    std::vector<gtsam::Key> keys = {JointAngleKey(j1_id, t),
        JointVelKey(j1_id, t), JointAccelKey(j1_id, t), TorqueKey(j1_id, t)};
    std::vector<std::string> vals = {std::to_string(t_elapsed)};
    for (auto&& k : keys) vals.push_back(std::to_string(results.atDouble(k)));
    traj_file << boost::algorithm::join(vals, ",") << "\n";
  }
  traj_file.close();

  return 0;
}
