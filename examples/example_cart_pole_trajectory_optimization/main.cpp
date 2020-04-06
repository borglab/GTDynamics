/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a cart pole.
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
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>

using gtdynamics::Robot, gtdynamics::JointAngleKey, gtdynamics::JointVelKey,
    gtdynamics::JointAccelKey, gtdynamics::TorqueKey, gtsam::PriorFactor,
    gtdynamics::ZeroValuesTrajectory, gtsam::noiseModel::Isotropic,
    gtsam::noiseModel::Constrained;

int main(int argc, char** argv) {
  // Load the inverted pendulum.
  auto cp = Robot("../cart_pole.urdf");
  int j0_id = cp.getJointByName("j0")->getID(),
      j1_id = cp.getJointByName("j1")->getID();
  cp.getLinkByName("l0")->fix();
  cp.printRobot();
  gtsam::Vector3 gravity(0, 0, -9.8);

  // Specify optimal control problem parameters.
  double T = 2,                     // Time horizon (s.)
      dt = 1. / 100,                // Time step (s.)
      sigma_dynamics = 1e-7,        // Variance of dynamics constraints.
      sigma_objectives = 5e-3,      // Variance of additional objectives.
      sigma_pos_objectives = 1e-5,  // Variance of positional objectives.
      sigma_control = 20;           // Variance of control.
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  auto dynamics_nm = Isotropic::Sigma(1, sigma_dynamics);
  auto objectives_nm = Isotropic::Sigma(1, sigma_objectives);
  auto pos_objectives_nm = Isotropic::Sigma(1, sigma_pos_objectives);

  // Specify initial conditions and goal state.
  // State: x, dx/dt, d^2x/dt^2, theta, dtheta/dt, d^2theta/dt
  gtsam::Vector6 X_i = gtsam::Vector6::Constant(6, 0),
                 X_T = (gtsam::Vector(6) << 1, 0, 0, M_PI, 0, 0).finished();
  std::cout << "\nInitial State: " << X_i.transpose() << std::endl;
  std::cout << "Goal State: " << X_T.transpose() << std::endl << std::endl;

  // Create trajectory factor graph.
  auto graph_builder = gtdynamics::DynamicsGraph();
  auto graph = graph_builder.trajectoryFG(
      cp, t_steps, dt,
      gtdynamics::DynamicsGraph::CollocationScheme::Trapezoidal, gravity);

  // Set the pendulum joint to be unactuated.
  for (int t = 0; t <= t_steps; t++)
    graph.emplace_shared<PriorFactor<double>>(TorqueKey(j1_id, t), 0.0,
                                              Constrained::All(1));

  // Add initial conditions to trajectory factor graph.
  graph.emplace_shared<PriorFactor<double>>(JointAngleKey(j0_id, 0), X_i[0],
                                            dynamics_nm);
  graph.emplace_shared<PriorFactor<double>>(JointVelKey(j0_id, 0), X_i[1],
                                            dynamics_nm);
  graph.emplace_shared<PriorFactor<double>>(JointAccelKey(j0_id, 0), X_i[2],
                                            dynamics_nm);
  graph.emplace_shared<PriorFactor<double>>(JointAngleKey(j1_id, 0), X_i[3],
                                            dynamics_nm);
  graph.emplace_shared<PriorFactor<double>>(JointVelKey(j1_id, 0), X_i[4],
                                            dynamics_nm);
  graph.emplace_shared<PriorFactor<double>>(JointAccelKey(j1_id, 0), X_i[5],
                                            dynamics_nm);

  // Add state and min torque objectives to trajectory factor graph.
  graph.emplace_shared<PriorFactor<double>>(JointVelKey(j0_id, t_steps), X_T[1],
                                            objectives_nm);
  graph.emplace_shared<PriorFactor<double>>(JointAccelKey(j0_id, t_steps),
                                            X_T[2], objectives_nm);
  graph.emplace_shared<PriorFactor<double>>(JointVelKey(j1_id, t_steps), X_T[4],
                                            objectives_nm);
  graph.emplace_shared<PriorFactor<double>>(JointAccelKey(j1_id, t_steps),
                                            X_T[5], objectives_nm);

  // Insert position objective (x, theta) factor at every timestep or only at
  // the terminal state. Adding the position objective will force the system to
  // converge to the desired state quicker at the cost of more impulsive control
  // actions.
  bool apply_pos_objective_all_dt = false;
  graph.emplace_shared<PriorFactor<double>>(JointAngleKey(j0_id, t_steps),
                                            X_T[0], pos_objectives_nm);
  graph.emplace_shared<PriorFactor<double>>(JointAngleKey(j1_id, t_steps),
                                            X_T[3], pos_objectives_nm);
  if (apply_pos_objective_all_dt) {
    for (int t = 0; t < t_steps; t++) {
      graph.emplace_shared<PriorFactor<double>>(JointAngleKey(j0_id, t), X_T[0],
                                                pos_objectives_nm);
      graph.emplace_shared<PriorFactor<double>>(JointAngleKey(j1_id, t), X_T[3],
                                                pos_objectives_nm);
    }
  }
  for (int t = 0; t <= t_steps; t++)
    graph.emplace_shared<gtdynamics::MinTorqueFactor>(
        TorqueKey(j0_id, t), Isotropic::Sigma(1, sigma_control));

  // Initialize solution.
  auto init_vals = ZeroValuesTrajectory(cp, t_steps, 0, 0.0);
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(40);
  params.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_vals, params);
  auto results = optimizer.optimize();

  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::ofstream traj_file;
  traj_file.open("../traj.csv");
  traj_file << "t,x,xdot,xddot,xtau,theta,thetadot,thetaddot,thetatau"
            << "\n";
  double t_elapsed = 0;
  for (int t = 0; t <= t_steps; t++, t_elapsed += dt) {
    std::vector<std::string> vals = {
        std::to_string(t_elapsed),
        std::to_string(results.atDouble(JointAngleKey(j0_id, t))),
        std::to_string(results.atDouble(JointVelKey(j0_id, t))),
        std::to_string(results.atDouble(JointAccelKey(j0_id, t))),
        std::to_string(results.atDouble(TorqueKey(j0_id, t))),
        std::to_string(results.atDouble(JointAngleKey(j1_id, t))),
        std::to_string(results.atDouble(JointVelKey(j1_id, t))),
        std::to_string(results.atDouble(JointAccelKey(j1_id, t))),
        std::to_string(results.atDouble(TorqueKey(j1_id, t)))};
    traj_file << boost::algorithm::join(vals, ",") << "\n";
  }
  traj_file.close();

  return 0;
}
