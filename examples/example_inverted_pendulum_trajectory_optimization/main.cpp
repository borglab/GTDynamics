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
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

using gtdynamics::Robot, gtdynamics::JointAngleKey, gtdynamics::JointVelKey,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtsam::PriorFactor,
    gtdynamics::ZeroValuesTrajectory, gtsam::noiseModel::Isotropic;

int main(int argc, char** argv) {
  // Load the inverted pendulum.
  auto ip = Robot("../inverted_pendulum.urdf");
  auto j1 = ip.getJointByName("j1");
  ip.getLinkByName("l1")->fix();
  ip.printRobot();
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished(),
                 planar_axis = (gtsam::Vector(3) << 1, 0, 0).finished();

  // Specify optimal control problem parameters.
  double T = 3.0,                  // Time horizon (s.)
         dt = 1. / 100,            // Time step (s.)
         sigma_dynamics = 1e-5,    // Variance of dynamics constraints.
         sigma_objectives = 1e-2,  // Variance of additional objectives.
         sigma_control = 1e-1;     // Variance of control.
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  // Specify initial conditions and goal state.
  double theta_i = 0, dtheta_i = 0, ddtheta_i = 0;
  double theta_T = M_PI, dtheta_T = 0, ddtheta_T = 0;

  // Create trajectory factor graph.
  auto graph_builder = gtdynamics::DynamicsGraph();
  auto graph = graph_builder.trajectoryFG(ip, t_steps, dt,
    gtdynamics::DynamicsGraph::CollocationScheme::Trapezoidal,
    gravity, planar_axis);

  // Add initial conditions to trajectory factor graph.
  graph.add(PriorFactor<double>(JointAngleKey(j1->getID(), 0), theta_i,
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointVelKey(j1->getID(), 0), dtheta_i,
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointAccelKey(j1->getID(), 0), ddtheta_i,
    Isotropic::Sigma(1, sigma_dynamics)));

  // Add state and min torque objectives to trajectory factor graph.
  graph.add(PriorFactor<double>(JointVelKey(j1->getID(), t_steps), dtheta_T,
    Isotropic::Sigma(1, sigma_objectives)));
  graph.add(PriorFactor<double>(JointAccelKey(j1->getID(), t_steps), dtheta_T,
    Isotropic::Sigma(1, sigma_objectives)));
  bool apply_theta_objective_all_dt = false;
  graph.add(PriorFactor<double>(JointAngleKey(j1->getID(), t_steps), theta_T,
    Isotropic::Sigma(1, sigma_objectives)));
  if (apply_theta_objective_all_dt) {
    for (int t = 0; t < t_steps; t++)
      graph.add(PriorFactor<double>(JointAngleKey(j1->getID(), t), theta_T,
        Isotropic::Sigma(1, sigma_objectives)));
  }
  for (int t = 0; t <= t_steps; t++)
    graph.add(gtdynamics::MinTorqueFactor(
      TorqueKey(j1->getID(), t), Isotropic::Sigma(1, sigma_control)));

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
    std::vector<std::string> vals = {
      std::to_string(t_elapsed),
      std::to_string(results.atDouble(JointAngleKey(j1->getID(), t))),
      std::to_string(results.atDouble(JointVelKey(j1->getID(), t))),
      std::to_string(results.atDouble(JointAccelKey(j1->getID(), t))),
      std::to_string(results.atDouble(TorqueKey(j1->getID(), t)))};
    traj_file << boost::algorithm::join(vals, ",") << "\n";
  }
  traj_file.close();

  return 0;
}
