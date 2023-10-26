/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main.cpp
 * @brief Trajectory optimization for a legged robot with contacts.
 * @author Alejandro Escontrela
 */

#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>

#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/dynamics/OptimizerSetting.h>
#include <gtdynamics/factors/MinTorqueFactor.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

#include <gtdynamics/factors/ContactDynamicsMomentFactor.h>
#include <gtdynamics/factors/ContactDynamicsFrictionConeFactor.h>
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/factors/TorqueFactor.h>

#include "gtdynamics/factors/ContactPointFactor.h"
#include "gtdynamics/manifold/ConnectedComponent.h"
#include "gtdynamics/optimizer/ConstrainedOptimizer.h"
#include "gtdynamics/manifold/ConstraintManifold.h"
#include "gtdynamics/manifold/TspaceBasis.h"
#include "gtdynamics/optimizer/EqualityConstraint.h"
#include "gtdynamics/optimizer/InequalityConstraint.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/values.h"
#include "gtdynamics/optimizer/OptimizationBenchmark.h"

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

using namespace gtdynamics;
using namespace gtsam;

void TrajectoryOptimization() {
  /// Initialize vision60 robot
  IEVision60Robot::Params vision60_params;
  vision60_params.express_redundancy = true;
  vision60_params.basis_using_torques = true;
  std::map<std::string, double> torque_lower_limits;
  std::map<std::string, double> torque_upper_limits;
  double lower_torque_lower_limit = -20.0;
  double lower_torque_upper_limit = 20.0;
  for (const auto& hip_joint : IEVision60Robot::lower_joint_names) {
    torque_lower_limits.insert({hip_joint, lower_torque_lower_limit});
    torque_upper_limits.insert({hip_joint, lower_torque_upper_limit});
  }
  vision60_params.torque_upper_limits = torque_upper_limits;
  vision60_params.torque_lower_limits = torque_lower_limits;
  vision60_params.include_torque_limits = true;

  IEVision60Robot vision60(vision60_params);

  /// Scenario
  size_t num_steps = 10;
  double dt = 0.1;
  Pose3 base_pose_init(Rot3::Identity(), Point3(0, 0, vision60.nominal_height));
  Vector6 base_twist_init = Vector6::Zero();

  Pose3 des_pose(Rot3::Ry(-0.2), Point3(0.2, 0, vision60.nominal_height + 0.2));
  Vector6 des_twist = Vector6::Zero();

  /// Constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k=0; k<=num_steps; k++) {
    e_constraints.add(vision60.eConstraints(k));
    i_constraints.add(vision60.iConstraints(k));
  }
  e_constraints.add(vision60.initStateConstraints(base_pose_init, base_twist_init));
  auto EvaluateConstraints = [=] (const Values& values) {
    std::cout << "e constraints violation:\t" << e_constraints.evaluateViolationL2Norm(values) << "\n";
    std::cout << "i constraints violation:\t" << i_constraints.evaluateViolationL2Norm(values) << "\n";
  };

  /// Costs
  NonlinearFactorGraph collocation_costs = vision60.collocationCosts(num_steps, dt);
  NonlinearFactorGraph boundary_costs = vision60.finalStateCosts(des_pose, des_twist, num_steps);
  NonlinearFactorGraph min_torque_costs = vision60.minTorqueCosts(num_steps);
  NonlinearFactorGraph costs;
  costs.add(collocation_costs);
  costs.add(boundary_costs);
  costs.add(min_torque_costs);
  auto EvaluateCosts = [=] (const Values& values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values) << "\n";
    std::cout << "boundary costs:\t" << boundary_costs.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values) << "\n";
  };

  /// Initial Values
  std::vector<Pose3> des_poses{des_pose};
  std::vector<double> des_poses_t{num_steps * dt};
  auto init_values = vision60.getInitValuesTrajectory(num_steps, dt, base_pose_init, des_poses, des_poses_t);
  EvaluateCosts(init_values);

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);

  // Parameters
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->ecm_params->basis_key_func = vision60.getBasisKeyFunc();

  Vision60Retractor::Params vision60_retractor_params;
  vision60_retractor_params.lm_params = LevenbergMarquardtParams();
  // vision60_retractor_params.lm_params.setVerbosityLM("SUMMARY");
  // vision60_retractor_params.lm_params.minModelFidelity = 0.5;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;
  vision60_retractor_params.use_basis_keys = true;
  vision60_retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60RetractorCreator>(vision60, vision60_retractor_params);


  LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(10);
  IELMParams ie_params;

  // optimize IELM
  auto lm_result = OptimizeIELM(problem, lm_params, ie_params, iecm_params);
  Values result_values = IEOptimizer::CollectManifoldValues(
      lm_result.second.back().state.manifolds);
  for (const auto &iter_details : lm_result.second) {
    IEOptimizer::PrintIterDetails(
        iter_details, num_steps, false, IEVision60Robot::PrintValues,
        IEVision60Robot::PrintDelta, gtdynamics::GTDKeyFormatter);
  }
  IEVision60Robot::PrintValues(result_values, num_steps);
  EvaluateCosts(result_values);
  IEVision60Robot::ExportValues(result_values, num_steps, "/Users/yetongzhang/packages/noboost/GTD_ineq/GTDynamics/data/ineq_quadruped_traj.csv");
  
  // // Optimize Barrier
  // BarrierParameters barrier_params;
  // barrier_params.initial_mu = 1e0;
  // barrier_params.num_iterations = 5;
  // auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);
  // EvaluateCosts(barrier_result.second.rbegin()->values);
  // barrier_result.first.exportFileWithMu("/Users/yetongzhang/packages/noboost/GTD_ineq/GTDynamics/data/ineq_quadruped_barrier.txt");
  // barrier_result.first.printLatex(std::cout);
  // lm_result.first.printLatex(std::cout);
}


int main(int argc, char **argv) {
  TrajectoryOptimization();
  return 0;
}

