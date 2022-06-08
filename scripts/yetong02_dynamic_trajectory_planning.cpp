/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  dynamic_trajectory_planning.cpp
 * @brief Dynamic trajectory planning problem of rotating up a cart-pole.
 * @author Yetong Zhang
 */

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/factors/JointLimitFactor.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/optimizer/ManifoldOptimizerType1.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/slam/BetweenFactor.h>

#include "manifold_opt_benchmark.h"

using namespace gtsam;
using namespace gtdynamics;
using gtsam::noiseModel::Isotropic;

// Cart-pole dynamic planning scenario setting.
auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                 .fixLink("l0");
int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
auto prior_model = Isotropic::Sigma(1, 1e-7);           // prior constraints.
auto pos_objectives_model = Isotropic::Sigma(1, 1e-5);  // Pos objectives.
auto objectives_model = Isotropic::Sigma(1, 5e-3);  // Additional objectives.
auto control_model = Isotropic::Sigma(1, 20);       // Controls.
gtsam::Vector6 X_i = gtsam::Vector6::Constant(6, 0),
               X_T = (gtsam::Vector(6) << 1, 0, 0, M_PI, 0, 0).finished();
const gtsam::Vector3 gravity(0, 0, -9.8);
auto graph_builder = DynamicsGraph(gravity);
double T = 2, dt = 1. / 100;  // Time horizon (s) and timestep duration (s).
int num_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.
auto collocation_scheme = CollocationScheme::Trapezoidal;

/** Function to manually define the basis keys for each constraint manifold. */
KeyVector FindBasisKeys(const ConnectedComponent::shared_ptr& cc) {
  KeyVector basis_keys;
  for (const Key& key : cc->keys) {
    auto symb = DynamicsSymbol(key);
    if (symb.label() == "q") {
      basis_keys.push_back(key);
    }
    if (symb.label() == "v") {
      basis_keys.push_back(key);
    }
    if (symb.label() == "T" and symb.jointIdx() == j0_id) {
      basis_keys.push_back(key);
    }
  }
  return basis_keys;
}

/** Factor graph of all constraints, includes kinodynamic constraints at each
 * step, pendulum joint unactuated constraint, initial condition for first step.
 */
NonlinearFactorGraph get_constraints_graph() {
  NonlinearFactorGraph graph;
  // kinodynamic constraints at each step
  for (size_t k = 0; k <= num_steps; k++) {
    graph.add(graph_builder.dynamicsFactorGraph(robot, k));
  }

  // Set the pendulum joint to be unactuated.
  for (int k = 0; k <= num_steps; k++)
    graph.addPrior(TorqueKey(j1_id, k), 0.0,
                   graph_builder.opt().prior_t_cost_model);

  // Initial conditions
  graph.addPrior(JointAngleKey(j0_id, 0), X_i[0], prior_model);
  graph.addPrior(JointVelKey(j0_id, 0), X_i[1], prior_model);
  graph.addPrior(JointAccelKey(j0_id, 0), X_i[2], prior_model);
  graph.addPrior(JointAngleKey(j1_id, 0), X_i[3], prior_model);
  graph.addPrior(JointVelKey(j1_id, 0), X_i[4], prior_model);
  return graph;
}

/** Cost for planning, includes terminal state objectives, control costs.*/
NonlinearFactorGraph get_costs() {
  NonlinearFactorGraph graph;

  // terminal pose objective
  bool apply_pos_objective_all_dt = false;
  if (apply_pos_objective_all_dt) {
    for (int k = 0; k < num_steps; k++) {
      graph.addPrior(JointAngleKey(j0_id, k), X_T[0], pos_objectives_model);
      graph.addPrior(JointAngleKey(j1_id, k), X_T[3], pos_objectives_model);
    }
  }
  graph.addPrior(JointAngleKey(j0_id, num_steps), X_T[0], pos_objectives_model);
  graph.addPrior(JointAngleKey(j1_id, num_steps), X_T[3], pos_objectives_model);
  graph.addPrior(JointVelKey(j0_id, num_steps), X_T[1], objectives_model);
  graph.addPrior(JointAccelKey(j0_id, num_steps), X_T[2], objectives_model);
  graph.addPrior(JointVelKey(j1_id, num_steps), X_T[4], objectives_model);
  graph.addPrior(JointAccelKey(j1_id, num_steps), X_T[5], objectives_model);

  // control cost
  for (int k = 0; k <= num_steps; k++)
    graph.emplace_shared<MinTorqueFactor>(TorqueKey(j0_id, k), control_model);
  return graph;
}

/** Collocation between each step, benchmark will be made by either treating
 * collocation as costs or constraints. */
NonlinearFactorGraph get_collocation() {
  NonlinearFactorGraph graph;
  for (int k = 0; k < num_steps; k++) {
    graph.add(
        graph_builder.collocationFactors(robot, k, dt, collocation_scheme));
  }
  return graph;
}

/** Initial values seet as rest pose for all steps. */
Values get_init_values() {
  Initializer initializer;
  Values values0 = initializer.ZeroValues(robot, 0, 0.0);
  Values known_values;
  for (const auto& joint : robot.joints()) {
    InsertJointAngle(&known_values, joint->id(),
                     JointAngle(values0, joint->id()));
    InsertJointVel(&known_values, joint->id(), JointVel(values0, joint->id()));
    InsertTorque(&known_values, joint->id(), 0.0);
  }
  for (const auto& link : robot.links()) {
    InsertPose(&known_values, link->id(), Pose(values0, link->id()));
    InsertTwist(&known_values, link->id(), Twist(values0, link->id()));
  }
  values0 = graph_builder.linearSolveFD(robot, 0, known_values);

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    for (const auto& joint : robot.joints()) {
      InsertJointAngle(&values, joint->id(), k,
                       JointAngle(values0, joint->id()));
      InsertJointVel(&values, joint->id(), k, JointVel(values0, joint->id()));
      InsertJointAccel(&values, joint->id(), k,
                       JointAccel(values0, joint->id()));
      InsertTorque(&values, joint->id(), k, Torque(values0, joint->id()));
      for (const auto& link : joint->links()) {
        InsertWrench(&values, link->id(), joint->id(), k,
                     Wrench(values0, link->id(), joint->id()));
      }
    }
    for (const auto& link : robot.links()) {
      InsertPose(&values, link->id(), k, Pose(values0, link->id()));
      InsertTwist(&values, link->id(), k, Twist(values0, link->id()));
      InsertTwistAccel(&values, link->id(), k, TwistAccel(values0, link->id()));
    }
  }
  return values;
}

/** Print joint angles for all steps. */
void print_joint_angles(const Values& values) {
  for (size_t k = 0; k <= num_steps; k++) {
    std::cout << "step " << k << ":";
    for (const auto& joint : robot.joints()) {
      double angle = JointAngle(values, joint->id(), k);
      std::cout << "\t" << angle;
    }
    std::cout << "\n";
  }
}

/** Cart-pole dynamic planning example benchmarking (1) dynamic factor graph (2)
 * constraint manifold. */
void dynamic_planning() {
  auto constraints_graph = get_constraints_graph();
  auto costs = get_costs();
  auto collocation_graph = get_collocation();
  auto init_values = get_init_values();

  Initializer initializer;
  auto init_values_nonfeasible =
      initializer.ZeroValuesTrajectory(robot, num_steps, 0, 0.0);

  costs.add(collocation_graph);

  // optimize dynamics graph
  std::cout << "dynamics graph with feasible initial values:\n";
  auto dfg_result_feasible =
      optimize_dynamics_graph(constraints_graph, costs, init_values);
  std::cout << "final cost: " << costs.error(dfg_result_feasible) << "\n";
  std::cout << "constraint violation: "
            << constraints_graph.error(dfg_result_feasible) << "\n";

  std::cout << "dynamics graph with infeasible initial values:\n";
  auto constraints = ConstraintsFromGraph(constraints_graph);
  auto dfg_result_infeasible = optimize_dynamics_graph(
      constraints_graph, costs, init_values_nonfeasible);
  std::cout << "final cost: " << costs.error(dfg_result_infeasible) << "\n";
  std::cout << "constraint violation: "
            << constraints_graph.error(dfg_result_infeasible) << "\n";
  print_joint_angles(dfg_result_infeasible);

  // optimize constraint manifold
  std::cout << "constraint manifold:\n";
  auto cm_result =
      optimize_constraint_manifold_specify_variables(constraints, costs, init_values, &FindBasisKeys);
  std::cout << "final cost: " << costs.error(cm_result) << "\n";
  std::cout << "constraint violation: " << constraints_graph.error(cm_result)
            << "\n";
  print_joint_angles(cm_result);

  tictoc_finishedIteration_();
  tictoc_print_();
}

int main(int argc, char** argv) {
  dynamic_planning();
  return 0;
}
