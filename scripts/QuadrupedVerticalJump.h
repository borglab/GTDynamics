/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedVerticalJump.h
 * @brief Utilities for quadruped vertical jump experiments
 * @author Yetong Zhang
 */

#pragma once
#include "gtdynamics/utils/GraphUtils.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtsam;
using namespace gtdynamics;

namespace quadruped_vertical_jump {

inline IEVision60Robot::Params GetVision60Params() {
  IEVision60Robot::Params vision60_params;
  vision60_params.express_redundancy = true;
  vision60_params.express_contact_force = true;
  vision60_params.ad_basis_using_torques = true;
  vision60_params.collocation = CollocationScheme::Trapezoidal;
  std::map<std::string, double> torque_lower_limits;
  std::map<std::string, double> torque_upper_limits;
  double hip_torque_lower_limit = -20.0;
  double hip_torque_upper_limit = 20.0;
  double upper_torque_lower_limit = -20.0;
  double upper_torque_upper_limit = 20.0;
  double lower_torque_lower_limit = -20.0;
  double lower_torque_upper_limit = 20.0;
  for (const auto &leg : IEVision60Robot::legs) {
    torque_lower_limits.insert({leg.hip_joint->name(), hip_torque_lower_limit});
    torque_upper_limits.insert({leg.hip_joint->name(), hip_torque_upper_limit});
    torque_lower_limits.insert(
        {leg.upper_joint->name(), upper_torque_lower_limit});
    torque_upper_limits.insert(
        {leg.upper_joint->name(), upper_torque_upper_limit});
    torque_lower_limits.insert(
        {leg.lower_joint->name(), lower_torque_lower_limit});
    torque_upper_limits.insert(
        {leg.lower_joint->name(), lower_torque_upper_limit});
  }
  vision60_params.torque_upper_limits = torque_upper_limits;
  vision60_params.torque_lower_limits = torque_lower_limits;

  vision60_params.include_torque_limits = true;

  vision60_params.sigma_des_pose = 1e-2;
  vision60_params.sigma_des_twist = 1e-2;
  vision60_params.sigma_actuation = 1e1;
  vision60_params.sigma_q_col = 5e-3;
  vision60_params.sigma_v_col = 5e-3;
  vision60_params.sigma_twist_col = 5e-3;
  vision60_params.sigma_pose_col = 3e-3;

  return vision60_params;
}

struct VerticalJumpParams {
  IEVision60Robot::Params vision60_params = GetVision60Params();
  std::vector<size_t> phase_num_steps{10, 10};
  bool include_inequalities = false;
  bool add_phase_prior = false;
  bool init_values_with_trapezoidal = false;
  bool add_phase_duration_constraints = false;
  std::vector<double> phases_min_dt;
  std::vector<double> phase_prior_dt;
  Pose3 des_pose = Pose3(Rot3::Ry(0), Point3(0, 0, 0.8));
};

inline IEVision60RobotMultiPhase
GetVision60MultiPhase(const VerticalJumpParams &params) {
  IEVision60Robot::Params vision60_params = params.vision60_params;
  vision60_params.set4C();
  IEVision60Robot vision60_4c(vision60_params);
  vision60_params.setInAir();
  IEVision60Robot vision60_air(vision60_params);
  vision60_params.setBoundaryLeave(vision60_4c.params, vision60_air.params);
  IEVision60Robot vision60_boundary(vision60_params);

  std::vector<IEVision60Robot> phase_robots{vision60_4c, vision60_air};
  std::vector<IEVision60Robot> boundary_robots{vision60_boundary};
  IEVision60RobotMultiPhase vision60_multi_phase(phase_robots, boundary_robots,
                                                 params.phase_num_steps);
  return vision60_multi_phase;
}

inline IEConsOptProblem CreateProblem(const VerticalJumpParams &params) {
  auto vision60_multi_phase = GetVision60MultiPhase(params);
  const IEVision60Robot &vision60_4c = vision60_multi_phase.phase_robots_[0];
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps = vision60_multi_phase.phase_num_steps_[0] +
                     vision60_multi_phase.phase_num_steps_[1];

  Pose3 base_pose_init(Rot3::Identity(),
                       Point3(0, 0, vision60_4c.nominal_height));
  Vector6 base_twist_init = Vector6::Zero();

  Values des_values;
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointAngle(&des_values, joint->id(), num_steps, 0.0);
  }
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointVel(&des_values, joint->id(), num_steps, 0.0);
  }
  InsertTwist(&des_values, vision60_air.base_id, num_steps, Vector6::Zero());
  InsertPose(&des_values, vision60_air.base_id, num_steps, params.des_pose);

  /// Constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    const IEVision60Robot &vision60 = vision60_multi_phase.robotAtStep(k);
    e_constraints.add(vision60.eConstraints(k));
    if (params.include_inequalities) {
      i_constraints.add(vision60.iConstraints(k));
    }
  }
  e_constraints.add(
      vision60_4c.initStateConstraints(base_pose_init, base_twist_init));
  if (params.add_phase_duration_constraints) {
    i_constraints.add(
        vision60_multi_phase.phaseMinDurationConstraints(params.phases_min_dt));
  }

  /// Costs
  NonlinearFactorGraph collocation_costs;
  std::vector<NonlinearFactorGraph> step_collocation_costs;
  size_t start_k = 0;
  for (size_t phase_idx = 0;
       phase_idx < vision60_multi_phase.phase_num_steps_.size(); phase_idx++) {
    size_t end_k = start_k + vision60_multi_phase.phase_num_steps_[phase_idx];
    for (size_t k = start_k; k < end_k; k++) {
      NonlinearFactorGraph collo_graph_step =
          vision60_multi_phase.phase_robots_.at(phase_idx)
              .multiPhaseCollocationCostsStep(k, phase_idx);
      collocation_costs.add(collo_graph_step);
      step_collocation_costs.push_back(collo_graph_step);
    }
    start_k = end_k;
  }

  // NonlinearFactorGraph collocation_costs =
  //     vision60_multi_phase.collocationCosts();
  NonlinearFactorGraph boundary_costs = vision60_air.stateCosts(des_values);
  NonlinearFactorGraph min_torque_costs = vision60_4c.minTorqueCosts(num_steps);
  NonlinearFactorGraph costs;
  costs.add(collocation_costs);
  costs.add(boundary_costs);
  costs.add(min_torque_costs);
  if (params.add_phase_prior) {
    costs.addPrior(PhaseKey(0), params.phase_prior_dt.at(0), noiseModel::Isotropic::Sigma(1, 1e-4));
    costs.addPrior(PhaseKey(1), params.phase_prior_dt.at(1), noiseModel::Isotropic::Sigma(1, 1e-4));
  }

  auto Evaluate = [=](const Values &values) {
    std::cout << "phase_dt: " << values.atDouble(PhaseKey(0)) << "\t"
              << values.atDouble(PhaseKey(1)) << "\n";
    std::cout << "collocation costs:\t" << collocation_costs.error(values)
              << "\n";
    std::cout << "boundary costs:\t" << boundary_costs.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values)
              << "\n";
    for (size_t k = 0; k < num_steps; k++) {
      std::cout << "\t" << k << "\t"
                << step_collocation_costs.at(k).error(values) << "\n";
    }
    std::cout << "e constraints violation:\t"
              << e_constraints.evaluateViolationL2Norm(values) << "\n";
    std::cout << "i constraints violation:\t"
              << i_constraints.evaluateViolationL2Norm(values) << "\n";
  };

  /// Initial Values
  std::vector<double> phases_dt{0.02, 0.02};
  auto init_values =
      TrajectoryValuesVerticalJump(vision60_multi_phase, phases_dt, 15, 5,
                                   params.init_values_with_trapezoidal);

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = Evaluate;
  return problem;
}

} // namespace quadruped_vertical_jump
