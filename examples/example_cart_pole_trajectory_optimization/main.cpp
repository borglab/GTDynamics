/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a cart pole.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/initialize_solution_utils.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/optional.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

using namespace gtdynamics;
using gtsam::noiseModel::Isotropic, gtsam::noiseModel::Constrained;

int main(int argc, char** argv) {
  // Load the inverted pendulum.
  auto cp = Robot("../cart_pole.urdf");
  int j0_id = cp.joint("j0")->id(), j1_id = cp.joint("j1")->id();
  cp.fixLink("l0");
  cp.print();
  gtsam::Vector3 gravity(0, 0, -9.8);

  double T = 2, dt = 1. / 100;  // Time horizon (s) and timestep duration (s).
  int t_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  // Noise models:
  auto dynamics_model = Isotropic::Sigma(1, 1e-7);  // Dynamics constraints.
  auto pos_objectives_model = Isotropic::Sigma(1, 1e-5);  // Pos objectives.
  auto objectives_model = Isotropic::Sigma(1, 5e-3);  // Additional objectives.
  auto control_model = Isotropic::Sigma(1, 20);       // Controls.

  // Specify initial conditions and goal state.
  // State: x, dx/dt, d^2x/dt^2, theta, dtheta/dt, d^2theta/dt
  gtsam::Vector6 X_i = gtsam::Vector6::Constant(6, 0),
                 X_T = (gtsam::Vector(6) << 1, 0, 0, M_PI, 0, 0).finished();
  std::cout << "\nInitial State: " << X_i.transpose() << std::endl;
  std::cout << "Goal State: " << X_T.transpose() << std::endl << std::endl;

  // Create trajectory factor graph.
  auto graph_builder = DynamicsGraph();
  auto graph = graph_builder.trajectoryFG(
      cp, t_steps, dt, DynamicsGraph::CollocationScheme::Trapezoidal, gravity);

  // Set the pendulum joint to be unactuated.
  for (int t = 0; t <= t_steps; t++)
    graph.addPrior(TorqueKey(j1_id, t), 0.0, Constrained::All(1));

  // Add initial conditions to trajectory factor graph.
  graph.addPrior(JointAngleKey(j0_id, 0), X_i[0], dynamics_model);
  graph.addPrior(JointVelKey(j0_id, 0), X_i[1], dynamics_model);
  graph.addPrior(JointAngleKey(j1_id, 0), X_i[3], dynamics_model);
  graph.addPrior(JointVelKey(j1_id, 0), X_i[4], dynamics_model);

  // Add terminal conditions to the factor graph.
  graph.addPrior(JointVelKey(j0_id, t_steps), X_T[1], objectives_model);
  graph.addPrior(JointAccelKey(j0_id, t_steps), X_T[2], objectives_model);
  graph.addPrior(JointVelKey(j1_id, t_steps), X_T[4], objectives_model);
  graph.addPrior(JointAccelKey(j1_id, t_steps), X_T[5], objectives_model);

  // Insert position objective (x, theta) factor at every timestep or only at
  // the terminal state. Adding the position objective at every timestep will
  // force the system to converge to the desired state quicker at the cost of
  // more impulsive control actions.
  bool apply_pos_objective_all_dt = false;
  graph.addPrior(JointAngleKey(j0_id, t_steps), X_T[0], pos_objectives_model);
  graph.addPrior(JointAngleKey(j1_id, t_steps), X_T[3], pos_objectives_model);
  if (apply_pos_objective_all_dt) {
    for (int t = 0; t < t_steps; t++) {
      graph.addPrior(JointAngleKey(j0_id, t), X_T[0], pos_objectives_model);
      graph.addPrior(JointAngleKey(j1_id, t), X_T[3], pos_objectives_model);
    }
  }
  for (int t = 0; t <= t_steps; t++)
    graph.emplace_shared<MinTorqueFactor>(TorqueKey(j0_id, t), control_model);

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
  traj_file << "t,x,xdot,xddot,xtau,theta,thetadot,thetaddot,thetatau\n";
  double t_elapsed = 0;
  for (int t = 0; t <= t_steps; t++, t_elapsed += dt) {
    std::vector<gtsam::Key> keys = {
        JointAngleKey(j0_id, t), JointVelKey(j0_id, t),
        JointAccelKey(j0_id, t), TorqueKey(j0_id, t),
        JointAngleKey(j1_id, t), JointVelKey(j1_id, t),
        JointAccelKey(j1_id, t), TorqueKey(j1_id, t)};
    std::vector<std::string> vals = {std::to_string(t_elapsed)};
    for (auto&& k : keys) vals.push_back(std::to_string(results.atDouble(k)));
    traj_file << boost::algorithm::join(vals, ",") << "\n";
  }
  traj_file.close();

  return 0;
}
