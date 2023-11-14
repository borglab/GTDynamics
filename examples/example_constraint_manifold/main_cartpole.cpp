/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  cartpole_planning.cpp
 * @brief Dynamic trajectory planning problem of rotating up a cart-pole.
 * @author Yetong Zhang
 */

#include "CartPoleUtils.h"

#include "gtdynamics/optimizer/AugmentedLagrangianOptimizer.h"
#include "gtdynamics/optimizer/PenaltyMethodOptimizer.h"
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/OptimizationBenchmark.h>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/ConjugateGradientSolver.h>

using namespace gtsam;
using namespace gtdynamics;
using gtsam::noiseModel::Isotropic;

/** Cart-pole dynamic planning example benchmarking (1) dynamic factor graph (2)
 * constraint manifold. */
void dynamic_planning() {
  double T = 2, dt = 1. / 100;  // Time horizon (s) and timestep duration (s).
  int num_steps = static_cast<int>(std::ceil(T / dt));  // Timesteps.

  CartPole cartpole;

  NonlinearFactorGraph dynamic_constraints_graph = cartpole.getDynamicsGraph(num_steps);
  NonlinearFactorGraph unactuated_graph = cartpole.getUnactuatedGraph(num_steps);
  NonlinearFactorGraph init_state_graph = cartpole.initStateGraph();

  NonlinearFactorGraph final_state_graph = cartpole.finalStateGraph(num_steps);
  NonlinearFactorGraph collocation_costs = cartpole.getCollocation(num_steps, dt);
  NonlinearFactorGraph min_torque_costs = cartpole.minTorqueCosts(num_steps);

  NonlinearFactorGraph constraints_graph;
  constraints_graph.add(dynamic_constraints_graph);
  constraints_graph.add(unactuated_graph);
  constraints_graph.add(init_state_graph);

  NonlinearFactorGraph costs;
  costs.add(final_state_graph);
  costs.add(collocation_costs);
  costs.add(min_torque_costs);
  auto EvaluateCosts = [=] (const Values& values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values) << "\n";
    std::cout << "final state costs:\t" << final_state_graph.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values) << "\n";
  };

  auto init_values = cartpole.getInitValues(num_steps, "zero");
  // cartpole.printJointAngles(init_values, num_steps);


  auto constraints = ConstraintsFromGraph(constraints_graph);
  auto problem = EqConsOptProblem(costs, constraints, init_values);
  std::ostringstream latex_os;

  LevenbergMarquardtParams lm_params;
  // lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e10);
  cartpole.exprotTrajectory(init_values, num_steps, dt, "/Users/yetongzhang/packages/GTDynamics/data/cartpole_traj_init.csv");

  double constraint_unit_scale =1e-3;
  // optimize soft constraints
  // std::cout << "soft constraints:\n";
  // auto soft_result =
  //     OptimizeSoftConstraints(problem, latex_os, lm_params, 1.0, constraint_unit_scale);
  // EvaluateCosts(soft_result);
  // cartpole.exprotTrajectory(soft_result, num_steps, dt, "/Users/yetongzhang/packages/GTDynamics/data/cartpole_traj_soft_infeas.csv");

  // // optimize penalty method
  lm_params.setMaxIterations(30);
  // std::cout << "penalty method:\n";
  // PenaltyMethodParameters penalty_params;
  // penalty_params.lm_parameters = lm_params;
  // auto penalty_result =
  //     OptimizePenaltyMethod(problem, latex_os, penalty_params, constraint_unit_scale);

  // // optimize augmented lagrangian
  // std::cout << "augmented lagrangian:\n";
  // AugmentedLagrangianParameters al_params;
  // al_params.lm_parameters = lm_params;
  // auto augl_result =
  //     OptimizeAugmentedLagrangian(problem, latex_os, al_params, constraint_unit_scale);

  // optimize constraint manifold specify variables (feasbile)
  // std::cout << "constraint manifold basis variables (feasible):\n";
  auto mopt_params = DefaultMoptParamsSV(cartpole.getBasisKeyFunc(true));
  mopt_params.cc_params->retractor_creator->params()->check_feasible=true;
  mopt_params.cc_params->retractor_creator->params()->lm_params.linearSolverType = gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  // auto cm_basis_result =
  //     OptimizeConstraintManifold(problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)", constraint_unit_scale);
  // EvaluateCosts(cm_basis_result);
  // cartpole.exprotTrajectory(cm_basis_result, num_steps, dt, "/Users/yetongzhang/packages/GTDynamics/data/cartpole_traj.csv");

  // // optimize constraint manifold specify variables (infeasbile)
  std::cout << "constraint manifold basis variables (infeasible):\n";
  mopt_params.cc_params->retractor_creator->params()->lm_params.setMaxIterations(1);
  auto cm_basis_infeasible_result =
      OptimizeConstraintManifold(problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)", constraint_unit_scale);

  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  dynamic_planning();
  return 0;
}
