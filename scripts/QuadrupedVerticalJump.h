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

inline IEVision60Robot::Params::shared_ptr GetVision60Params() {
  auto vision60_params = std::make_shared<IEVision60Robot::Params>();
  IEVision60Robot vision60_tmp(vision60_params,
                               IEVision60Robot::PhaseInfo::Ground());

  vision60_params->express_redundancy = true;
  vision60_params->express_contact_force = true;
  vision60_params->ad_basis_using_torques = true;
  vision60_params->collocation = CollocationScheme::Trapezoidal;

  // e-constraints
  vision60_params->include_state_constraints = true;
  Values state_values;
  InsertPose(
      &state_values, IEVision60Robot::base_id, 0,
      Pose3(Rot3::Identity(), Point3(0, 0, vision60_tmp.nominal_height)));
  InsertTwist(&state_values, IEVision60Robot::base_id, 0, Vector6::Zero());
  vision60_params->state_constrianed_values = state_values;

  // i-constraints
  vision60_params->include_torque_limits = false;
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
  vision60_params->torque_upper_limits = torque_upper_limits;
  vision60_params->torque_lower_limits = torque_lower_limits;

  // costs
  vision60_params->sigma_des_pose = 1e-2;
  vision60_params->sigma_des_twist = 1e-2;
  vision60_params->sigma_actuation = 1e1;
  vision60_params->sigma_jerk = 1e1;
  vision60_params->sigma_q_col = 5e-3;
  vision60_params->sigma_v_col = 5e-3;
  vision60_params->sigma_twist_col = 5e-3;
  vision60_params->sigma_pose_col = 5e-3;
  vision60_params->include_collocation_costs = true;
  vision60_params->include_actuation_costs = true;
  vision60_params->include_state_costs = true;
  // vision60_params->include_jerk_costs = true;

  // vision60_params->include_friction_cone = true;

  return vision60_params;
}

struct VerticalJumpParams {
  IEVision60Robot::Params::shared_ptr vision60_params = GetVision60Params();
  std::vector<size_t> phase_num_steps{10, 10};
  std::vector<double> phases_dt{0.02, 0.02};
  bool include_inequalities = false;
  bool init_values_with_trapezoidal = false;
  Pose3 des_pose = Pose3(Rot3::Ry(0), Point3(0, 0, 0.8));
};

inline IEVision60RobotMultiPhase
GetVision60MultiPhase(const VerticalJumpParams &params) {
  return GetVision60MultiPhase(params.vision60_params, params.phase_num_steps);
}

inline IEConsOptProblem CreateProblem(const VerticalJumpParams &params) {
  auto vision60_multi_phase = GetVision60MultiPhase(params);
  // const IEVision60Robot &vision60_4c = vision60_multi_phase.phase_robots_[0];
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps = vision60_multi_phase.numSteps();

  Values des_values;
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointAngle(&des_values, joint->id(), num_steps, 0.0);
  }
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointVel(&des_values, joint->id(), num_steps, 0.0);
  }
  InsertTwist(&des_values, vision60_air.base_id, num_steps, Vector6::Zero());
  InsertPose(&des_values, vision60_air.base_id, num_steps, params.des_pose);
  vision60_multi_phase.params()->state_cost_values = des_values;

  /// Constraints
  EqualityConstraints e_constraints = vision60_multi_phase.eConstraints();
  InequalityConstraints i_constraints = vision60_multi_phase.iConstraints();
  NonlinearFactorGraph costs = vision60_multi_phase.costs();

  /// Initial Values
  auto init_values =
      InitValuesTrajectory(vision60_multi_phase, params.phases_dt, 15, 5,
                           params.init_values_with_trapezoidal);
  // auto init_values =
  //   InitValuesTrajectoryInfeasible(vision60_multi_phase, params.phases_dt);

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = vision60_multi_phase.costsEvalFunc();
  return problem;
}

} // namespace quadruped_vertical_jump
