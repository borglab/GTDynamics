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

#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>
#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "gtdynamics/utils/DebugUtils.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include "gtdynamics/utils/values.h"

using namespace gtdynamics;
using namespace gtsam;

double SolveConstA(const size_t num_steps, const double dt,
                   const double target_x) {
  double x = 0, v = 0;
  for (size_t k = 0; k < num_steps; k++) {
    x += v * dt;
    v += 1 * dt;
  }
  double a = target_x / x;
  return a;
}

Values getInitValuesTrajectorySmoooth(
    const IEVision60RobotMultiPhase &vision60_multi_phase,
    const std::vector<double> &phases_dt) {
  Values values;
  for (size_t phase_idx = 0; phase_idx < phases_dt.size(); phase_idx += 1) {
    values.insert(PhaseKey(phase_idx), phases_dt.at(phase_idx));
  }

  //// Phase on ground: part1: const accel of torso
  std::cout << "phase on ground part1\n";
  const IEVision60Robot &vision60_ground =
      vision60_multi_phase.phase_robots_[0];
  size_t num_steps_ground = vision60_multi_phase.phase_num_steps_[0];
  double dt_ground = phases_dt[0];
  // double leave_height = 0.07;
  size_t num_steps_ground1 = num_steps_ground * 0.5;
  // double accel = SolveConstA(num_steps_ground1, dt_ground, leave_height);
  double accel = 15;

  double base_z = vision60_ground.nominal_height;
  double base_v = 0;
  Values prev_values;
  for (size_t k = 0; k <= num_steps_ground1; k++) {
    // set a,v,q level for each step of base_link
    Vector6 base_accel = (Vector(6) << 0, 0, 0, 0, 0, accel).finished();
    Vector6 base_twist = (Vector(6) << 0, 0, 0, 0, 0, base_v).finished();
    Pose3 base_pose(Rot3::Identity(), Point3(0, 0, base_z));
    base_z += base_v * dt_ground;
    base_v += accel * dt_ground;

    /// solve kinodynamics
    Values init_values_k = k == 0 ? vision60_ground.nominal_values
                                  : DynamicsValuesFromPrev(prev_values);
    init_values_k.update(PoseKey(vision60_ground.base_id, k), base_pose);
    init_values_k.update(TwistKey(vision60_ground.base_id, k), base_twist);
    init_values_k.update(TwistAccelKey(vision60_ground.base_id, k), base_accel);
    KeyVector known_keys{PoseKey(vision60_ground.base_id, k),
                         TwistKey(vision60_ground.base_id, k),
                         TwistAccelKey(vision60_ground.base_id, k),
                         ContactRedundancyKey(k)};
    Values values_k = vision60_ground.stepValues(k, init_values_k, known_keys);
    values.insert(values_k);
    prev_values = values_k;
  }

  //// Phase on ground: part2: reduce contact force to 0
  std::cout << "phase on ground part2\n";
  std::map<uint8_t, Vector3> contact_forces;
  for (const auto &link_id : vision60_ground.contact_link_ids) {
    contact_forces.insert({link_id, prev_values.at<Vector3>(ContactForceKey(
                                        link_id, 0, num_steps_ground1))});
  }

  size_t num_steps_ground2 = num_steps_ground - num_steps_ground1;
  for (size_t k = num_steps_ground1 + 1; k <= num_steps_ground; k++) {
    // integrate prev values
    Values init_values_k =
        vision60_ground.stepValuesByIntegration(k, dt_ground, prev_values);

    // set contact force
    for (const auto &link_id : vision60_ground.contact_link_ids) {
      Vector3 boundary_contact_force = contact_forces.at(link_id);
      Vector3 contact_force =
          boundary_contact_force *
          (double(num_steps_ground - k) / num_steps_ground2);
      init_values_k.update(ContactForceKey(link_id, 0, k), contact_force);
    }

    // solve for ad level
    KeyVector basis_keys;
    basis_keys.push_back(PoseKey(vision60_ground.base_id, k));
    basis_keys.push_back(TwistKey(vision60_ground.base_id, k));
    for (const auto &link_id : vision60_ground.contact_link_ids) {
      basis_keys.push_back(ContactForceKey(link_id, 0, k));
    }
    init_values_k = vision60_ground.stepValues(k, init_values_k, basis_keys);
    values.insert(init_values_k);
    prev_values = init_values_k;
  }

  //// boundary step same as last step in ground phase

  //// Phase in air
  std::cout << "phase in air\n";
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps_air = vision60_multi_phase.phase_num_steps_[1];
  double dt_air = phases_dt[1];

  std::map<uint8_t, double> prev_torques;
  for (const auto &joint : vision60_air.robot.joints()) {
    uint8_t j = joint->id();
    prev_torques.insert({j, Torque(prev_values, j, num_steps_ground)});
  }

  size_t num_steps = num_steps_ground + num_steps_air;
  for (size_t k = num_steps_ground + 1; k <= num_steps; k++) {
    // integrate prev values
    Values init_values_k =
        vision60_air.stepValuesByIntegration(k, dt_air, prev_values);

    // set joint torques
    for (const auto &joint : vision60_air.robot.joints()) {
      uint8_t j = joint->id();
      double prev_torque = prev_torques.at(j);
      double torque = prev_torque * (double(num_steps - k) / num_steps_air);
      init_values_k.update(TorqueKey(j, k), torque);
    }

    // solve for ad-level
    KeyVector basis_keys;
    basis_keys.push_back(PoseKey(vision60_air.base_id, k));
    basis_keys.push_back(TwistKey(vision60_air.base_id, k));
    for (const auto &joint : vision60_air.robot.orderedJoints()) {
      basis_keys.push_back(JointAngleKey(joint->id(), k));
      basis_keys.push_back(JointVelKey(joint->id(), k));
      basis_keys.push_back(TorqueKey(joint->id(), k));
    }
    init_values_k = vision60_air.stepValues(k, init_values_k, basis_keys);
    values.insert(init_values_k);
    prev_values = init_values_k;
  }

  return values;
}

Values
getInitValuesTrajectory(const IEVision60RobotMultiPhase &vision60_multi_phase,
                        const std::vector<double> &phases_dt) {
  Values values;
  for (size_t phase_idx = 0; phase_idx < phases_dt.size(); phase_idx += 1) {
    values.insert(PhaseKey(phase_idx), phases_dt.at(phase_idx));
  }

  //// Phase on ground
  std::cout << "phase on ground\n";
  const IEVision60Robot &vision60_ground =
      vision60_multi_phase.phase_robots_[0];
  size_t num_steps_ground = vision60_multi_phase.phase_num_steps_[0];
  double dt_ground = phases_dt[0];
  double leave_height = 0.2;
  double accel = SolveConstA(num_steps_ground, dt_ground, leave_height);

  double base_z = vision60_ground.nominal_height;
  double base_v = 0;
  Values prev_values;
  for (size_t k = 0; k < num_steps_ground; k++) {
    // set a,v,q level for each step of base_link
    Vector6 base_accel = (Vector(6) << 0, 0, 0, 0, 0, accel).finished();
    Vector6 base_twist = (Vector(6) << 0, 0, 0, 0, 0, base_v).finished();
    Pose3 base_pose(Rot3::Identity(), Point3(0, 0, base_z));
    base_z += base_v * dt_ground;
    base_v += accel * dt_ground;

    /// solve kinodynamics
    Values init_values_k = k == 0 ? vision60_ground.nominal_values
                                  : DynamicsValuesFromPrev(prev_values);
    init_values_k.update(PoseKey(vision60_ground.base_id, k), base_pose);
    init_values_k.update(TwistKey(vision60_ground.base_id, k), base_twist);
    init_values_k.update(TwistAccelKey(vision60_ground.base_id, k), base_accel);
    KeyVector known_keys{PoseKey(vision60_ground.base_id, k),
                         TwistKey(vision60_ground.base_id, k),
                         TwistAccelKey(vision60_ground.base_id, k),
                         ContactRedundancyKey(k)};
    Values values_k = vision60_ground.stepValues(k, init_values_k, known_keys);
    values.insert(values_k);
    prev_values = values_k;
  }

  //// boundary step
  std::cout << "boundary step\n";
  size_t boundary_k = vision60_multi_phase.boundary_ks_[0];
  const IEVision60Robot &vision60_boundary =
      vision60_multi_phase.boundary_robots_[0];
  Values values_boundary;
  {
    values_boundary = vision60_ground.stepValuesByIntegration(
        boundary_k, dt_ground, prev_values);
    values_boundary = vision60_boundary.stepValues(boundary_k, values_boundary);
    values.insert(values_boundary);
  }

  //// Phase in air
  std::cout << "phase in air\n";
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps_air = vision60_multi_phase.phase_num_steps_[1];
  double dt_air = phases_dt[1];

  // set joint constant a for air phase
  Values values_first_air = vision60_air.stepValuesByIntegration(
      boundary_k + 1, dt_air, values_boundary);
  Values joint_a_air_values;
  for (const auto &joint : vision60_air.robot.joints()) {
    uint8_t j = joint->id();
    double v = JointVel(values_first_air, j, boundary_k + 1);
    double a = -v / ((num_steps_air - 1) * dt_air);
    InsertJointAccel(&joint_a_air_values, j, boundary_k, a);
  }

  prev_values = values_boundary;
  for (size_t k = boundary_k + 1; k <= boundary_k + num_steps_air; k++) {
    // integrate prev values
    Values init_values_k =
        vision60_air.stepValuesByIntegration(k, dt_air, prev_values);

    // set joint accel
    joint_a_air_values = DynamicsValuesFromPrev(joint_a_air_values);
    init_values_k.update(joint_a_air_values);

    // solve for target a
    KeyVector basis_keys;
    basis_keys.push_back(PoseKey(vision60_air.base_id, k));
    basis_keys.push_back(TwistKey(vision60_air.base_id, k));
    for (const auto &joint : vision60_air.robot.orderedJoints()) {
      basis_keys.push_back(JointAngleKey(joint->id(), k));
      basis_keys.push_back(JointVelKey(joint->id(), k));
      basis_keys.push_back(JointAccelKey(joint->id(), k));
    }
    init_values_k = vision60_air.stepValues(k, init_values_k, basis_keys);
    values.insert(init_values_k);
    prev_values = init_values_k;
  }

  return values;
}

IEVision60Robot::Params GetVision60Params() {
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
    torque_lower_limits.insert(
        {leg.hip_joint->name(), hip_torque_lower_limit});
    torque_upper_limits.insert(
        {leg.hip_joint->name(), hip_torque_upper_limit});
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
  vision60_params.sigma_q_col = 1e-2;
  vision60_params.sigma_v_col = 1e-2;

  return vision60_params;
}

IEVision60RobotMultiPhase
GetVision60MultiPhase(IEVision60Robot::Params &_vision60_params) {
  IEVision60Robot::Params vision60_params = _vision60_params;
  vision60_params.set4C();
  IEVision60Robot vision60_4c(vision60_params);
  vision60_params.setInAir();
  IEVision60Robot vision60_air(vision60_params);
  vision60_params.setBoundaryLeave(vision60_4c.params, vision60_air.params);
  IEVision60Robot vision60_boundary(vision60_params);

  /// Scenario
  size_t num_steps_ground = 10;
  size_t num_steps_air = 10;

  std::vector<IEVision60Robot> phase_robots{vision60_4c, vision60_air};
  std::vector<IEVision60Robot> boundary_robots{vision60_boundary};
  std::vector<size_t> phase_num_steps{num_steps_ground, num_steps_air};
  IEVision60RobotMultiPhase vision60_multi_phase(phase_robots, boundary_robots,
                                                 phase_num_steps);
  return vision60_multi_phase;
}

IEConsOptProblem
CreateProblem(const IEVision60RobotMultiPhase &vision60_multi_phase) {
  const IEVision60Robot &vision60_4c = vision60_multi_phase.phase_robots_[0];
  const IEVision60Robot &vision60_air = vision60_multi_phase.phase_robots_[1];
  size_t num_steps = vision60_multi_phase.phase_num_steps_[0] +
                     vision60_multi_phase.phase_num_steps_[1];

  Pose3 base_pose_init(Rot3::Identity(),
                       Point3(0, 0, vision60_4c.nominal_height));
  Vector6 base_twist_init = Vector6::Zero();

  Values des_values;
  for (const auto &joint : vision60_air.robot.joints()) {
    InsertJointVel(&des_values, joint->id(), num_steps, 0.0);
  }
  InsertTwist(&des_values, vision60_air.base_id, num_steps, Vector6::Zero());
  Pose3 des_pose(Rot3::Ry(0), Point3(0, 0, 0.8));
  InsertPose(&des_values, vision60_air.base_id, num_steps, des_pose);

  /// Constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    const IEVision60Robot &vision60 = vision60_multi_phase.robotAtStep(k);
    e_constraints.add(vision60.eConstraints(k));
    i_constraints.add(vision60.iConstraints(k));
  }
  e_constraints.add(
      vision60_4c.initStateConstraints(base_pose_init, base_twist_init));

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
  auto Evaluate = [=](const Values &values) {
    std::cout << "phase_dt: " << values.atDouble(PhaseKey(0)) << "\t"
              << values.atDouble(PhaseKey(1)) << "\n";
    std::cout << "collocation costs:\t" << collocation_costs.error(values)
              << "\n";
    for (size_t k = 0; k < num_steps; k++) {
      std::cout << "\t" << k << "\t"
                << step_collocation_costs.at(k).error(values) << "\n";
    }
    std::cout << "boundary costs:\t" << boundary_costs.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values)
              << "\n";
    // PrintGraphWithError(collocation_costs, values);
    // PrintGraphWithError(boundary_costs, values);
    std::cout << "e constraints violation:\t"
              << e_constraints.evaluateViolationL2Norm(values) << "\n";
    std::cout << "i constraints violation:\t"
              << i_constraints.evaluateViolationL2Norm(values) << "\n";
  };

  /// Initial Values
  std::vector<double> phases_dt{0.02, 0.02};
  auto init_values = getInitValuesTrajectorySmoooth(vision60_multi_phase, phases_dt);
  // Evaluate(init_values);

  // IEVision60Robot::ExportValues(init_values, num_steps,
  //                               "/Users/yetongzhang/packages/noboost/GTD_ineq/"
  //                               "GTDynamics/data/quadruped_traj_init.csv");

  /// Problem
  IEConsOptProblem problem(costs, e_constraints, i_constraints, init_values);
  problem.eval_func = Evaluate;
  return problem;
}

void RunRetractorBenchMark(
    const IEConsOptProblem &problem,
    const std::map<std::string, IERetractorCreator::shared_ptr>
        &retractor_creators,
    const LevenbergMarquardtParams &lm_params, const IELMParams &ie_params,
    const IEConstraintManifold::Params::shared_ptr &_ecm_params) {
  auto iecm_params =
      std::make_shared<IEConstraintManifold::Params>(*_ecm_params);

  for (const auto &it : retractor_creators) {
    const auto &retractor_name = it.first;
    const auto &retractor_creator = it.second;
    iecm_params->retractor_creator = retractor_creator;
    std::cout << retractor_name << ":\n";
    auto lm_result = OptimizeIELM(problem, lm_params, ie_params, iecm_params);
    // Values result_values = lm_result.second.back().state.baseValues();
    const auto &iters_details = lm_result.second;
    std::string state_file_path =
        "../../data/quadruped_ground_air_" + retractor_name + "_states.csv";
    std::string trial_file_path =
        "../../data/quadruped_ground_air_" + retractor_name + "_trials.csv";
    iters_details.exportFile(state_file_path, trial_file_path);
  }
}

void RetractorBenchMark() {
  /// Initialize vision60 robot
  auto vision60_params = GetVision60Params();
  vision60_params.ad_basis_using_torques = false;
  auto vision60_multi_phase_a = GetVision60MultiPhase(vision60_params);
  vision60_params.ad_basis_using_torques = true;
  auto vision60_multi_phase_T = GetVision60MultiPhase(vision60_params);

  /// Create problem
  auto problem = CreateProblem(vision60_multi_phase_a);

  BarrierRetractor::Params barrier_params;
  barrier_params.prior_sigma = 0.1;

  KinodynamicHierarchicalRetractor::Params vision60_retractor_params;
  vision60_retractor_params.prior_sigma = 0.1;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;

  auto barrier_retractor_creator_no_basis_keys =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_a, barrier_params, false);

  auto barrier_retractor_creator_a_basis_keys =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_a, barrier_params, true);

  auto barrier_retractor_creator_T_basis_keys =
      std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
          vision60_multi_phase_T, barrier_params, true);

  auto hierarchical_retractor_creator_no_basis_keys =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase_a, vision60_retractor_params, false);

  auto hierarchical_retractor_creator_a_basis_keys =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase_a, vision60_retractor_params, true);

  auto hierarchical_retractor_creator_T_basis_keys =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase_T, vision60_retractor_params, true);

  std::map<std::string, IERetractorCreator::shared_ptr> retractor_creators{
      // {"barrier_none", barrier_retractor_creator_no_basis_keys},
      // {"barrier_a", barrier_retractor_creator_a_basis_keys},
      // {"barrier_T", barrier_retractor_creator_T_basis_keys},
      {"hierarchical_none", hierarchical_retractor_creator_no_basis_keys},
      {"hierarchical_a", hierarchical_retractor_creator_a_basis_keys},
      {"hierarchical_T", hierarchical_retractor_creator_T_basis_keys}};

  LevenbergMarquardtParams lm_params;
  lm_params.setLinearSolverType("SEQUENTIAL_QR");
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setMaxIterations(100);
  lm_params.setlambdaInitial(1e0);
  lm_params.setlambdaUpperBound(1e10);

  IELMParams ie_params;

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_with_new_constraints = true;
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->e_basis_creator =
      std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
          vision60_multi_phase_T, iecm_params->ecm_params->basis_params);

  RunRetractorBenchMark(problem, retractor_creators, lm_params, ie_params,
                        iecm_params);
}

void TrajectoryOptimization() {
  /// Initialize vision60 robot
  auto vision60_params = GetVision60Params();
  auto vision60_multi_phase = GetVision60MultiPhase(vision60_params);
  size_t num_steps = vision60_multi_phase.phase_num_steps_[0] +
                     vision60_multi_phase.phase_num_steps_[1];
  // std::vector<double> phases_dt{0.01, 0.01};

  // Values init_values =
  //     getInitValuesTrajectorySmoooth(vision60_multi_phase, phases_dt);

  /// Create problem
  auto problem = CreateProblem(vision60_multi_phase);
  IEVision60Robot::PrintValues(problem.initValues(), num_steps);
  problem.eval_func(problem.initValues());

  IEVision60Robot::ExportValues(problem.initValues(), num_steps,
                                "/Users/yetongzhang/packages/noboost/GTD_ineq/"
                                "GTDynamics/data/quadruped_traj_init.csv");
  // optimize IELM
  // Parameters
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->e_basis_with_new_constraints = true;
  iecm_params->ecm_params->basis_params->setFixVars();
  // iecm_params->ecm_params->basis_key_func = vision60.getBasisKeyFunc();

  KinodynamicHierarchicalRetractor::Params vision60_retractor_params;
  vision60_retractor_params.lm_params = LevenbergMarquardtParams();
  // vision60_retractor_params.lm_params.setVerbosityLM("SUMMARY");
  // vision60_retractor_params.lm_params.minModelFidelity = 0.5;
  vision60_retractor_params.check_feasible = true;
  vision60_retractor_params.feasible_threshold = 1e-3;
  vision60_retractor_params.prior_sigma = 0.1;
  iecm_params->retractor_creator =
      std::make_shared<Vision60MultiPhaseHierarchicalRetractorCreator>(
          vision60_multi_phase, vision60_retractor_params, false);
  // BarrierRetractor::Params barrier_params;
  // barrier_params.prior_sigma = 0.1;
  // iecm_params->retractor_creator =
  //     std::make_shared<Vision60MultiPhaseBarrierRetractorCreator>(
  //         vision60_multi_phase, barrier_params, true);
  iecm_params->e_basis_creator =
      std::make_shared<Vision60MultiPhaseTspaceBasisCreator>(
          vision60_multi_phase, iecm_params->ecm_params->basis_params);

  LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");
  lm_params.setMaxIterations(10);
  lm_params.setLinearSolverType("SEQUENTIAL_QR");
  lm_params.setlambdaInitial(1e-1);
  // lm_params.setlambdaUpperBound(1e10);
  // lm_params.minModelFidelity = 0.3;
  IELMParams ie_params;

  auto lm_result = OptimizeIELM(problem, lm_params, ie_params, iecm_params);
  Values result_values = lm_result.second.back().state.baseValues();
  for (const auto &iter_details : lm_result.second) {
    IEOptimizer::PrintIterDetails(
        iter_details, num_steps, false, IEVision60Robot::PrintValues,
        IEVision60Robot::PrintDelta, gtdynamics::GTDKeyFormatter);
  }
  IEVision60Robot::PrintValues(result_values, num_steps);
  problem.eval_func(result_values);
  IEVision60Robot::ExportValues(result_values, num_steps,
  "/Users/yetongzhang/packages/noboost/GTD_ineq/"
  "GTDynamics/data/ineq_quadruped_traj.csv");

  // // Optimize Barrier
  // BarrierParameters barrier_params;
  // barrier_params.verbose = true;
  // barrier_params.initial_mu = 1e0;
  // barrier_params.num_iterations = 10;
  // auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);
  // problem.eval_func(barrier_result.second.rbegin()->values);
  // // IEVision60Robot::PrintValues(barrier_result.second.rbegin()->values,
  // //                              num_steps);
  // barrier_result.first.exportFileWithMu(
  //     "/Users/yetongzhang/packages/noboost/GTD_ineq/GTDynamics/data/"
  //     "ineq_quadruped_barrier.csv");

  // barrier_result.first.printLatex(std::cout);
  // lm_result.first.printLatex(std::cout);
}

int main(int argc, char **argv) {
  TrajectoryOptimization();
  // RetractorBenchMark();
  return 0;
}
