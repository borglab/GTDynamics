/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  gerry02_cablerobot_kinodynamics.cpp
 * @brief Benchmark cable robot kinodynamics planning with Yetong's manifold opt
 * @author Gerry Chen
 * @author Yetong Zhang
 */

#include "gtdynamics/optimizer/OptimizationBenchmark.h"

#include "gtdynamics/utils/CableRobotUtils.h"

#include <fstream>
#include <iostream>
#include <string>

using namespace gtdynamics;
using namespace gtsam;

void TrajectoryOptimization() {
  CableRobotParams params;
  params.dt = 1e-2;
  params.R = 1;
  params.Q = 1e2;
  params.init_pose = Pose3(Rot3(), Point3(1.5, 1.0, 0));
  CableRobot cdpr(params);

  /// Scenario
  size_t num_steps = 1000;
  // std::map<int, Pose3> des_poses{{10, Pose3(Rot3(), Point3(1.7, 1.2, 0))},
  //                                {20, Pose3(Rot3(), Point3(1.3, 1.2, 0))},
  //                                {29, Pose3(Rot3(), Point3(1.5, 1.0, 0))}};
  // std::map<int, Pose3> des_poses{{29, Pose3(Rot3(), Point3(1.55, 1.2, 0))}};
  std::map<int, Pose3> des_poses;
  for (int k = 0; k < num_steps; ++k) {
    des_poses.emplace(
        k, Pose3(Rot3(), Point3(1.5 + 0.2 * k / (num_steps - 1),
                                1.0 + 0.1 * k / (num_steps - 1), 0)));
  }
  // for (int i = 0; i < 10; ++i) {  // extra cost on terminal objective
  //   des_poses.emplace(29, Pose3(Rot3(), Point3(1.5, 1.0, 0)));
  // }

  /// Get constraints, costs, and initial values.
  auto constraints = ConstraintsFromGraph(cdpr.constraintFactors(num_steps));
  auto costs_all = cdpr.costFactors(num_steps, des_poses);
  auto costs = NonlinearFactorGraph();
  for (const auto& [_, cost] : costs_all) {
    costs.add(cost);
  }
  auto EvaluateCosts = [costs_all](const Values& values) {
    for (const auto& [str, cost] : costs_all) {
      std::cout << str << ":\t" << cost.error(values) << "\n";
    }
  };
  auto init_values = cdpr.initialValues(num_steps);

  /// Export initial trajectory
  EvaluateCosts(init_values);
  cdpr.exportTrajectory(init_values, num_steps,
                        "/Users/gerry/Downloads/init_traj.csv");

  /// Construct problem
  auto problem = EqConsOptProblem(costs, constraints, init_values);
  std::ostringstream latex_os;
  LevenbergMarquardtParams lm_params;
  lm_params.minModelFidelity = 0.3;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setlambdaUpperBound(1e10);
  // lm_params.setMaxIterations(10);

  // optimize soft constraints
  std::cout << "soft constraints:\n";
  auto soft_result = OptimizeSoftConstraints(problem, latex_os, lm_params, 1e2);
  EvaluateCosts(soft_result);
  cdpr.exportTrajectory(soft_result, num_steps,
                        "/Users/gerry/Downloads/soft_traj.csv");

  // optimize penalty method
  std::cout << "penalty method:\n";
  auto penalty_result = OptimizePenaltyMethod(problem, latex_os);
  EvaluateCosts(penalty_result);

  // optimize augmented lagrangian
  std::cout << "augmented lagrangian:\n";
  auto augl_result = OptimizeAugmentedLagrangian(problem, latex_os);
  EvaluateCosts(augl_result);

  // optimize constraint manifold don't specify variables (feasbile)
  std::cout << "constraint manifold basis variables (feasible):\n";
  auto mopt_params = DefaultMoptParams();
  auto cm_basis_result = OptimizeConstraintManifold(
      problem, latex_os, mopt_params, lm_params, "Constraint Manifold (F)");
  EvaluateCosts(cm_basis_result);
  cdpr.exportTrajectory(cm_basis_result, num_steps,
                        "/Users/gerry/Downloads/manifold_feasible_traj.csv");

  // optimize constraint manifold don't specify variables (infeasbile)
  std::cout << "constraint manifold basis variables (infeasible):\n";
  mopt_params.cc_params->retract_params->lm_params.setMaxIterations(1);
  auto cm_basis_infeasible_result = OptimizeConstraintManifold(
      problem, latex_os, mopt_params, lm_params, "Constraint Manifold (I)");
  EvaluateCosts(cm_basis_infeasible_result);

  /*
  // optimize constraint manifold basis variables feasible
  std::cout << "constraint manifold basis variables feasible:\n";
  auto mopt_params2 = DefaultMoptParamsSV();
  // mopt_params2.cc_params->retract_params->setDynamics(true);
  // mopt_params2.cc_params->retract_params->setProjection(true, 1.0, true);
  mopt_params2.cc_params->retract_params->setUopt();
  mopt_params2.cc_params->basis_key_func = cdpr.getBasisKeyFunc();
  mopt_params2.cc_params->retract_params->check_feasible = true;
  auto cm_result = OptimizeConstraintManifold(
      problem, latex_os, mopt_params2, lm_params, "Constraint Manifold (F)");
  EvaluateCosts(cm_result);
  cdpr.exportTrajectory(cm_result, num_steps,
                        "/Users/gerry/Downloads/cm_traj.csv");

  // optimize constraint manifold basis variables infeasible
  std::cout << "constraint manifold basis variables infeasible:\n";
  mopt_params2.cc_params->retract_params->lm_params.setMaxIterations(10);
  auto cm_infeas_result = OptimizeConstraintManifold(
      problem, latex_os, mopt_params2, lm_params, "Constraint Manifold (I)");
  EvaluateCosts(cm_infeas_result);
  cdpr.exportTrajectory(cm_infeas_result, num_steps,
                        "/Users/gerry/Downloads/cm_infeas_traj.csv");
  */

  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  TrajectoryOptimization();
  return 0;
}
