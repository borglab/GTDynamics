/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedVerticalJumpLand.h
 * @brief Utilities for quadruped vertical jump experiments
 * @author Yetong Zhang
 */

#pragma once
#include "gtdynamics/utils/GraphUtils.h"
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

using namespace gtsam;
using namespace gtdynamics;

namespace quadruped_forward_jump_land {

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
  // joint limits
  std::map<std::string, double> joint_lower_limits;
  std::map<std::string, double> joint_upper_limits;
  double hip_joint_lower_limit = -0.1;
  double hip_joint_upper_limit = M_PI_2;
  double upper_joint_lower_limit = -1;
  double upper_joint_upper_limit = 4;
  double lower_joint_lower_limit = -0.76;
  double lower_joint_upper_limit = M_PI;
  for (const auto &leg : IEVision60Robot::legs) {
    std::string hip_name = leg.hip_joint->name();
    if (IEVision60Robot::isLeft(hip_name)) {
      joint_lower_limits.insert({hip_name, hip_joint_lower_limit});
      joint_upper_limits.insert({hip_name, hip_joint_upper_limit});
    } else {
      joint_lower_limits.insert({hip_name, -hip_joint_upper_limit});
      joint_upper_limits.insert({hip_name, -hip_joint_lower_limit});
    }
    joint_lower_limits.insert(
        {leg.upper_joint->name(), upper_joint_lower_limit});
    joint_upper_limits.insert(
        {leg.upper_joint->name(), upper_joint_upper_limit});
    joint_lower_limits.insert(
        {leg.lower_joint->name(), lower_joint_lower_limit});
    joint_upper_limits.insert(
        {leg.lower_joint->name(), lower_joint_upper_limit});
  }
  vision60_params->joint_lower_limits = joint_lower_limits;
  vision60_params->joint_upper_limits = joint_upper_limits;

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

  // collision checking points
  std::vector<std::pair<std::string, Point3>> collision_points;
  collision_points.emplace_back("body", Point3(0.44, 0, -0.095));
  collision_points.emplace_back("body", Point3(-0.44, 0, -0.095));
  collision_points.emplace_back("fl_lower", Point3(0.14, 0, 0));
  collision_points.emplace_back("fr_lower", Point3(0.14, 0, 0));
  collision_points.emplace_back("rl_lower", Point3(0.14, 0, 0));
  collision_points.emplace_back("rr_lower", Point3(0.14, 0, 0));
  collision_points.emplace_back("fl_upper", Point3(-0.125, 0, 0));
  collision_points.emplace_back("fr_upper", Point3(-0.125, 0, 0));
  collision_points.emplace_back("rl_upper", Point3(-0.125, 0, 0));
  collision_points.emplace_back("rr_upper", Point3(-0.125, 0, 0));
  vision60_params->collision_checking_points_z = collision_points;

  // costs
  vision60_params->sigma_des_pose = 1e-3;
  vision60_params->sigma_des_twist = 1e-2;
  vision60_params->sigma_des_point = 5e-3;
  vision60_params->sigma_des_point_v = 1e-2;
  vision60_params->sigma_actuation = 1e1;
  vision60_params->sigma_jerk = 1e1;
  vision60_params->sigma_q_col = 5e-3;
  vision60_params->sigma_v_col = 5e-3;
  vision60_params->sigma_twist_col = 5e-3;
  vision60_params->sigma_pose_col = 5e-3;
  vision60_params->sigma_a_penalty = 1e1;

  // friction cone
  vision60_params->mu = 1.0;

  return vision60_params;
}

struct ForwardJumpLandParams {
  IEVision60Robot::Params::shared_ptr vision60_params = GetVision60Params();
  std::vector<size_t> phase_num_steps{20, 10, 20, 20};
  std::vector<double> phases_dt{0.01, 0.02, 0.02, 0.02};
  bool init_values_include_i_constraints = false;
  bool init_values_ensure_feasible = false;
  double forward_distance = 1.5;
};

inline std::tuple<IEConsOptProblem, IEVision60RobotMultiPhase::shared_ptr,
                  ForwardJumpLandParams>
CreateProblem(const ForwardJumpLandParams &params) {
  auto vision60_multi_phase = GetVision60MultiPhase(
      params.vision60_params, params.phase_num_steps, params.forward_distance);

  const IEVision60Robot &vision60_air = vision60_multi_phase->phase_robots_[1];
  size_t num_steps = vision60_multi_phase->numSteps();

  Values des_values;
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointAngle(&des_values, joint->id(), num_steps, 0.0);
  }
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointVel(&des_values, joint->id(), num_steps, 0.0);
  }
  InsertTwist(&des_values, vision60_air.base_id, num_steps, Vector6::Zero());
  Pose3 des_pose(Rot3::Identity(), Point3(params.forward_distance, 0.0,
                                          vision60_air.nominal_height));
  InsertPose(&des_values, vision60_air.base_id, num_steps, des_pose);
  vision60_multi_phase->params()->state_cost_values = des_values;

  /// Constraints
  EqualityConstraints e_constraints = vision60_multi_phase->eConstraints();
  InequalityConstraints i_constraints = vision60_multi_phase->iConstraints();
  NonlinearFactorGraph costs = vision60_multi_phase->costs();

  /// Initial Values
  auto init_values =
      InitValuesTrajectory(*vision60_multi_phase, params.phases_dt,
                           params.init_values_include_i_constraints,
                           params.init_values_ensure_feasible);

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = vision60_multi_phase->costsEvalFunc();
  return {problem, vision60_multi_phase, params};
}

} // namespace quadruped_forward_jump_land
