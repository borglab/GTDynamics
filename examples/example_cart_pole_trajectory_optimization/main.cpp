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
    gtdynamics::ZeroValuesTrajectory, gtsam::noiseModel::Isotropic,
    gtsam::noiseModel::Constrained;

int main(int argc, char** argv) {
  // Load the inverted pendulum.
  auto cp = Robot("../cart_pole.urdf");
  auto j0 = cp.getJointByName("j0"), j1 = cp.getJointByName("j1");
  cp.getLinkByName("l0")->fix();
  cp.printRobot();
  gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8).finished();

  // Specify optimal control problem parameters.
  double T = 2,                        // Time horizon (s.)
         dt = 1. / 100,                // Time step (s.)
         sigma_dynamics = 1e-7,        // Variance of dynamics constraints.
         sigma_objectives = 5e-3,      // Variance of additional objectives.
         sigma_pos_objectives = 1e-5,  // Variance of positional objectives.
         sigma_control = 20;           // Variance of control.
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  // Specify initial conditions and goal state.
  // State: x, dx/dt, d^2x/dt^2, theta, dtheta/dt, d^2theta/dt
  gtsam::Vector6 X_i = gtsam::Vector6::Constant(6, 0),
                 X_T = (gtsam::Vector(6) << 1, 0, 0, M_PI, 0, 0).finished();
  std::cout << "\nInitial State: " << X_i.transpose() << std::endl;
  std::cout << "Goal State: " << X_T.transpose() << std::endl << std::endl;

  // Create trajectory factor graph.
  auto graph_builder = gtdynamics::DynamicsGraph();
  auto graph = graph_builder.trajectoryFG(cp, t_steps, dt,
    gtdynamics::DynamicsGraph::CollocationScheme::Trapezoidal, gravity);

  // Set the pendulum joint to be unactuated.
  for (int t = 0; t <= t_steps; t++)
    graph.add(PriorFactor<double>(TorqueKey(j1->getID(), t), 0.0,
      Constrained::All(1)));

  // Add initial conditions to trajectory factor graph.
  graph.add(PriorFactor<double>(JointAngleKey(j0->getID(), 0), X_i[0],
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointVelKey(j0->getID(), 0), X_i[1],
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointAccelKey(j0->getID(), 0), X_i[2],
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointAngleKey(j1->getID(), 0), X_i[3],
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointVelKey(j1->getID(), 0), X_i[4],
    Isotropic::Sigma(1, sigma_dynamics)));
  graph.add(PriorFactor<double>(JointAccelKey(j1->getID(), 0), X_i[5],
    Isotropic::Sigma(1, sigma_dynamics)));

  // Add state and min torque objectives to trajectory factor graph.
  graph.add(PriorFactor<double>(JointVelKey(j0->getID(), t_steps), X_T[1],
    Isotropic::Sigma(1, sigma_objectives)));
  graph.add(PriorFactor<double>(JointAccelKey(j0->getID(), t_steps), X_T[2],
    Isotropic::Sigma(1, sigma_objectives)));
  graph.add(PriorFactor<double>(JointVelKey(j1->getID(), t_steps), X_T[4],
    Isotropic::Sigma(1, sigma_objectives)));
  graph.add(PriorFactor<double>(JointAccelKey(j1->getID(), t_steps), X_T[5],
    Isotropic::Sigma(1, sigma_objectives)));
  bool apply_pos_objective_all_dt = false;
  graph.add(PriorFactor<double>(JointAngleKey(j0->getID(), t_steps), X_T[0],
    Isotropic::Sigma(1, sigma_pos_objectives)));
  graph.add(PriorFactor<double>(JointAngleKey(j1->getID(), t_steps), X_T[3],
    Isotropic::Sigma(1, sigma_pos_objectives)));
  if (apply_pos_objective_all_dt) {
    for (int t = 0; t < t_steps; t++) {
      graph.add(PriorFactor<double>(JointAngleKey(j0->getID(), t), X_T[0],
        Isotropic::Sigma(1, sigma_pos_objectives)));
      graph.add(PriorFactor<double>(JointAngleKey(j1->getID(), t), X_T[3],
        Isotropic::Sigma(1, sigma_pos_objectives)));
    }
  }
  for (int t = 0; t <= t_steps; t++)
    graph.add(gtdynamics::MinTorqueFactor(
      TorqueKey(j0->getID(), t), Isotropic::Sigma(1, sigma_control)));

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
  traj_file << "t,x,xdot,xddot,xtau,theta,thetadot,thetaddot,thetatau" << "\n";
  double t_elapsed = 0;
  for (int t = 0; t <= t_steps; t++, t_elapsed += dt) {
    std::vector<std::string> vals = {
      std::to_string(t_elapsed),
      std::to_string(results.atDouble(JointAngleKey(j0->getID(), t))),
      std::to_string(results.atDouble(JointVelKey(j0->getID(), t))),
      std::to_string(results.atDouble(JointAccelKey(j0->getID(), t))),
      std::to_string(results.atDouble(TorqueKey(j0->getID(), t))),
      std::to_string(results.atDouble(JointAngleKey(j1->getID(), t))),
      std::to_string(results.atDouble(JointVelKey(j1->getID(), t))),
      std::to_string(results.atDouble(JointAccelKey(j1->getID(), t))),
      std::to_string(results.atDouble(TorqueKey(j1->getID(), t)))
      };
    traj_file << boost::algorithm::join(vals, ",") << "\n";
  }
  traj_file.close();

  return 0;
}
