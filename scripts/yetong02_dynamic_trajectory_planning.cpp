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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/timing.h>

using namespace gtsam;
using namespace gtdynamics;

auto robot = CreateRobotFromFile(kUrdfPath + std::string("cart_pole.urdf"))
                 .fixLink("l0");
int j0_id = robot.joint("j0")->id(), j1_id = robot.joint("j1")->id();
// cp.print();
const gtsam::Vector3 gravity(0, 0, -9.8);
// Noise models:
auto dynamics_model =
    noiseModel::Isotropic::Sigma(1, 1e-5);  // Dynamics constraints.
auto pos_objectives_model =
    noiseModel::Isotropic::Sigma(1, 1e-5);  // Pos objectives.
auto objectives_model =
    noiseModel::Isotropic::Sigma(1, 5e-3);  // Additional objectives.
auto control_model = noiseModel::Isotropic::Sigma(1, 20);  // Controls.
gtsam::Vector6 X_i = gtsam::Vector6::Constant(6, 0),
               X_T = (gtsam::Vector(6) << 1, 0, 0, M_PI, 0, 0).finished();
auto graph_builder = DynamicsGraph(gravity);

double T = 2, dt = 1. / 100;  // Time horizon (s) and timestep duration (s).
int num_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.
auto collocation_scheme = CollocationScheme::Trapezoidal;

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

/** Kinematic trajectory optimization using dynamics factor graph. */
Values optimize_dynamics_graph(const NonlinearFactorGraph& constraints_graph,
                               const NonlinearFactorGraph& costs,
                               const Values& init_values) {
  NonlinearFactorGraph graph;
  graph.add(constraints_graph);
  graph.add(costs);

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, init_values, params);
  gttic_(dynamic_factor_graph);
  auto result = optimizer.optimize();
  gttoc_(dynamic_factor_graph);

  size_t graph_dim = 0;
  for (const auto& factor : graph) {
    graph_dim += factor->dim();
  }
  std::cout << "dimension: " << graph_dim << " x " << init_values.dim() << "\n";
  return result;
}

/** Kinematic trajectory optimization using constraint manifold. */
Values optimize_constraint_manifold(
    const NonlinearFactorGraph& constraints_graph,
    const NonlinearFactorGraph& costs, const Values& init_values) {
  auto constraints = ConstraintsFromGraph(constraints_graph);

  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  // params.setlambdaInitial(1e2);
  // params.setlambdaUpperBound(1e10);
  auto manopt_params = boost::make_shared<ManifoldOptimizer::Params>();
  manopt_params->cc_params->retract_type =
      ConstraintManifold::Params::RetractType::PARTIAL_PROJ;
  manopt_params->cc_params->basis_type =
      ConstraintManifold::Params::BasisType::SPECIFY_VARIABLES;
  // manopt_params->cc_params->lm_params.setVerbosityLM("SUMMARY");
  // params.minModelFidelity = 0.1;
  std::cout << "building optimizer\n";
  ManifoldOptimizerType1 optimizer(costs, constraints, init_values, params,
                                   manopt_params, &FindBasisKeys);
  // optimizer.print("", GTDKeyFormatter);
  std::cout << "optimize\n";
  gttic_(constraint_manifold);
  auto result = optimizer.optimize();
  gttoc_(constraint_manifold);

  auto problem_dim = optimizer.problem_dimension();
  std::cout << "dimension: " << problem_dim.first << " x " << problem_dim.second
            << "\n";
  return result;
}

/** Factor graph of all kinematic constraints. */
NonlinearFactorGraph get_constraints_graph() {
  NonlinearFactorGraph graph;

  OptimizerSetting opt;
  // opt.p_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.001);
  // opt.v_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
  // opt.a_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  // opt.f_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  // opt.fa_cost_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  // opt.t_cost_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  auto new_graph_builder = DynamicsGraph(opt, gravity);

  // kinodynamic constraints at each step
  for (size_t k = 0; k <= num_steps; k++) {
    graph.add(new_graph_builder.dynamicsFactorGraph(robot, k));
  }
  // Set the pendulum joint to be unactuated.
  for (int k = 0; k <= num_steps; k++)
    graph.addPrior(TorqueKey(j1_id, k), 0.0,
                   new_graph_builder.opt().prior_t_cost_model);
  graph.addPrior(JointAngleKey(j0_id, 0), X_i[0], dynamics_model);
  graph.addPrior(JointVelKey(j0_id, 0), X_i[1], dynamics_model);
  graph.addPrior(JointAccelKey(j0_id, 0), X_i[2], dynamics_model);
  graph.addPrior(JointAngleKey(j1_id, 0), X_i[3], dynamics_model);
  graph.addPrior(JointVelKey(j1_id, 0), X_i[4], dynamics_model);
  return graph;
}

NonlinearFactorGraph get_costs() {
  NonlinearFactorGraph graph;
  bool apply_pos_objective_all_dt = false;
  graph.addPrior(JointAngleKey(j0_id, num_steps), X_T[0], pos_objectives_model);
  graph.addPrior(JointAngleKey(j1_id, num_steps), X_T[3], pos_objectives_model);
  if (apply_pos_objective_all_dt) {
    for (int k = 0; k < num_steps; k++) {
      graph.addPrior(JointAngleKey(j0_id, k), X_T[0], pos_objectives_model);
      graph.addPrior(JointAngleKey(j1_id, k), X_T[3], pos_objectives_model);
    }
  }

  graph.addPrior(JointVelKey(j0_id, num_steps), X_T[1], objectives_model);
  graph.addPrior(JointAccelKey(j0_id, num_steps), X_T[2], objectives_model);
  graph.addPrior(JointVelKey(j1_id, num_steps), X_T[4], objectives_model);
  graph.addPrior(JointAccelKey(j1_id, num_steps), X_T[5], objectives_model);
  for (int k = 0; k <= num_steps; k++)
    graph.emplace_shared<MinTorqueFactor>(TorqueKey(j0_id, k), control_model);
  return graph;
}

NonlinearFactorGraph get_collocation() {
  NonlinearFactorGraph graph;
  for (int k = 0; k < num_steps; k++) {
    graph.add(
        graph_builder.collocationFactors(robot, k, dt, collocation_scheme));
  }
  return graph;
}

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
  // OptimizerSetting opt;
  // auto graph_builder = DynamicsGraph(opt, gravity);
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

void dynamic_planning() {
  auto constraints_graph = get_constraints_graph();
  auto costs = get_costs();
  auto collocation_graph = get_collocation();
  auto init_values = get_init_values();

  costs.add(collocation_graph);

  // optimize dynamics graph
  std::cout << "dynamics graph:\n";
  auto dfg_result =
      optimize_dynamics_graph(constraints_graph, costs, init_values);
  std::cout << "final cost: " << costs.error(dfg_result) << "\n";
  std::cout << "constraint violation: " << constraints_graph.error(dfg_result)
            << "\n";
  print_joint_angles(dfg_result);

  // optimize constraint manifold
  std::cout << "constraint manifold:\n";
  auto cm_result =
      optimize_constraint_manifold(constraints_graph, costs, init_values);
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
