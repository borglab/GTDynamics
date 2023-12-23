/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/scenarios/IEQuadrupedUtils.h>

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
IEVision60RobotMultiPhase::IEVision60RobotMultiPhase(
    const std::vector<IEVision60Robot> &phase_robots,
    const std::vector<IEVision60Robot> &boundary_robots,
    const std::vector<size_t> &phase_num_steps)
    : phase_robots_(phase_robots), boundary_robots_(boundary_robots),
      phase_num_steps_(phase_num_steps) {
  size_t k = 0;
  for (size_t i = 0; i < phase_num_steps.size() - 1; i++) {
    k += phase_num_steps[i];
    boundary_ks_.push_back(k);
  }
}

/* ************************************************************************* */
const IEVision60Robot &
IEVision60RobotMultiPhase::robotAtStep(const size_t k) const {
  for (size_t i = 0; i < boundary_ks_.size(); i++) {
    size_t boundary_k = boundary_ks_.at(i);
    if (k == boundary_k) {
      return boundary_robots_.at(i);
    } else if (k < boundary_k) {
      return phase_robots_.at(i);
    }
  }
  return phase_robots_.back();
}

/* <=======================================================================> */
/* <================================ costs ================================> */
/* <=======================================================================> */

/* ************************************************************************* */
std::vector<NonlinearFactorGraph>
IEVision60RobotMultiPhase::collocationFactorsByStep() const {
  std::vector<NonlinearFactorGraph> collocation_factors(numSteps());

  size_t k = 0;
  for (size_t phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    const auto &robot = phase_robots_.at(phase_idx);
    for (size_t step_idx = 0; step_idx < phase_num_steps_.at(phase_idx);
         step_idx++) {
      collocation_factors[k] =
          robot.stepMultiPhaseCollocationCosts(k, phase_idx);
      k++;
    }
  }
  return collocation_factors;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::collocationCosts() const {
  NonlinearFactorGraph graph;
  for (const auto &graph_k : collocationFactorsByStep()) {
    graph.add(graph_k);
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::actuationCosts() const {
  NonlinearFactorGraph graph;
  const auto &params = phase_robots_[0].params;
  size_t num_steps = numSteps();
  if (params->actuation_cost_option == ACTUATION_RMSE_TORQUE) {
    for (size_t k = 0; k <= num_steps; k++) {
      graph.add(robotAtStep(k).stepActuationRmseTorqueCosts(k));
    }
  } else if (params->actuation_cost_option == ACTUATION_IMPULSE_SQR ||
             params->actuation_cost_option == ACTUATION_IMPULSE) {
    bool apply_sqrt = params->actuation_cost_option == ACTUATION_IMPULSE;
    size_t k = 0;
    for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
      for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
           phase_step++) {
        graph.add(
            robotAtStep(k).stepActuationImpulseCosts(k, phase_idx, apply_sqrt));
        k++;
      }
    }
  } else if (params->actuation_cost_option == ACTUATION_WORK_SQR ||
             params->actuation_cost_option == ACTUATION_WORK) {
    bool apply_sqrt = params->actuation_cost_option == ACTUATION_WORK;
    size_t k = 0;
    for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
      for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
           phase_step++) {
        graph.add(robotAtStep(k).stepActuationWorkCosts(k, apply_sqrt));
        k++;
      }
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::jerkCosts() const {
  NonlinearFactorGraph graph;
  size_t k = 0;
  for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    const auto &robot = phase_robots_.at(phase_idx);
    Key phase_key = PhaseKey(phase_idx);
    for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
         phase_step++) {
      graph.add(robot.stepJerkCosts(k, phase_key));
      k++;
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::contactForceJerkCosts() const {
  NonlinearFactorGraph graph;
  size_t k = 0;
  for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    const auto &robot = phase_robots_.at(phase_idx);
    Key phase_key = PhaseKey(phase_idx);
    for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
         phase_step++) {
      graph.add(robot.stepContactForceJerkCosts(k, phase_key));
      k++;
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::symmetryCosts() const {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k <= numSteps(); k++) {
    graph.add(robotAtStep(k).stepSymmetryCosts(k));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::stateCosts() const {
  return phase_robots_.at(0).stateCosts();
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60RobotMultiPhase::phaseDurationPriorCosts() const {
  NonlinearFactorGraph graph;
  for (size_t phase_idx = 0; phase_idx < phase_robots_.size(); phase_idx++) {
    graph.addPrior(PhaseKey(phase_idx), params()->phase_prior_dt.at(phase_idx),
                   noiseModel::Isotropic::Sigma(1, params()->sigma_phase_dt));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::accelPenaltyCosts() const {
  return phase_robots_.at(0).accelPenaltyCosts(numSteps());
}

/* ************************************************************************* */
std::vector<std::pair<std::string, NonlinearFactorGraph>>
IEVision60RobotMultiPhase::classifiedCosts() const {
  std::vector<std::pair<std::string, NonlinearFactorGraph>> classified_costs;
  if (params()->include_collocation_costs) {
    classified_costs.emplace_back("collocation", collocationCosts());
  }
  if (params()->include_actuation_costs) {
    classified_costs.emplace_back("actuation", actuationCosts());
  }
  if (params()->include_jerk_costs) {
    classified_costs.emplace_back("jerk", jerkCosts());
  }
  if (params()->include_state_costs) {
    classified_costs.emplace_back("state", stateCosts());
  }
  if (params()->include_phase_duration_prior_costs) {
    classified_costs.emplace_back("phase_dt", phaseDurationPriorCosts());
  }
  if (params()->include_accel_penalty) {
    classified_costs.emplace_back("accel_penalty", accelPenaltyCosts());
  }
  if (params()->include_cf_jerk_costs) {
    classified_costs.emplace_back("cf_jerk", contactForceJerkCosts());
  }
  if (params()->collision_as_cost) {
    NonlinearFactorGraph graph;
    graph.add(groundCollisionFreeConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width));
    graph.add(obstacleCollisionFreeConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width));
    graph.add(hurdleCollisionFreeConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width));
    classified_costs.emplace_back("collision", graph);
  }
  if (params()->joint_limits_as_cost) {
    auto graph = jointLimitConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width);
    classified_costs.emplace_back("joint_limit", graph);
  }
  if (params()->torque_limits_as_cost) {
    auto graph = torqueLimitConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width);
    classified_costs.emplace_back("torque_limit", graph);
  }
  if (params()->friction_cone_as_cost) {
    auto graph = frictionConeConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width);
    classified_costs.emplace_back("friction_cone", graph);
  }
  if (params()->phase_duration_limit_as_cost) {
    auto graph = phaseMinDurationConstraints().meritGraph(
        1.0, params()->use_smooth_barrier_for_cost,
        params()->smooth_barrier_buffer_width);
    classified_costs.emplace_back("phase_min_dt", graph);
  }
  if (params()->include_collision_free_z_inter_cost) {
    NonlinearFactorGraph graph;
    graph.add(
        groundCollisionFreeInterStepConstraints().meritGraph(1.0, true, 5.0));
    classified_costs.emplace_back("collision_inter", graph);
  }
  if (params()->include_symmetry_costs) {
    classified_costs.emplace_back("symmetry", symmetryCosts());
  }
  return classified_costs;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::costs() const {
  NonlinearFactorGraph graph;
  for (const auto &[name, costs] : classifiedCosts()) {
    graph.add(costs);
  }
  return graph;
}

/* <=======================================================================> */
/* <============================= constraints =============================> */
/* <=======================================================================> */

/* ************************************************************************* */
InequalityConstraints IEVision60RobotMultiPhase::jointLimitConstraints() const {
  InequalityConstraints constraints;
  for (size_t k = 0; k <= numSteps(); k++) {
    constraints.add(robotAtStep(k).stepJointLimitConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::groundCollisionFreeConstraints() const {
  InequalityConstraints constraints;
  for (size_t k = 0; k <= numSteps(); k++) {
    constraints.add(robotAtStep(k).stepGroundCollisionFreeConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::groundCollisionFreeInterStepConstraints() const {
  InequalityConstraints constraints;
  size_t k = 0;
  for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    const auto &robot = phase_robots_.at(phase_idx);
    for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
         phase_step++) {
      constraints.add(robot.interStepGroundCollisionFreeConstraints(k));
      k++;
    }
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::obstacleCollisionFreeConstraints() const {
  InequalityConstraints constraints;
  for (size_t k = 0; k <= numSteps(); k++) {
    constraints.add(robotAtStep(k).stepObstacleCollisionFreeConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::hurdleCollisionFreeConstraints() const {
  InequalityConstraints constraints;
  for (size_t k = 0; k <= numSteps(); k++) {
    constraints.add(robotAtStep(k).stepHurdleCollisionFreeConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::torqueLimitConstraints() const {
  InequalityConstraints constraints;
  for (size_t k = 0; k <= numSteps(); k++) {
    constraints.add(robotAtStep(k).stepTorqueLimitConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::frictionConeConstraints() const {
  InequalityConstraints constraints;
  for (size_t k = 0; k <= numSteps(); k++) {
    constraints.add(robotAtStep(k).stepFrictionConeConstraints(k));
  }
  return constraints;
}

/* ************************************************************************* */
InequalityConstraints
IEVision60RobotMultiPhase::phaseMinDurationConstraints() const {
  InequalityConstraints constraints;
  for (size_t phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    Key phase_key = PhaseKey(phase_idx);
    double phase_min_dt = params()->phases_min_dt.at(phase_idx);
    Double_ phase_expr(phase_key);
    Double_ phase_min_expr = phase_expr - Double_(phase_min_dt);
    constraints.emplace_shared<DoubleExpressionInequality>(
        phase_min_expr, params()->tol_phase_dt);
  }
  return constraints;
}

/* ************************************************************************* */
std::vector<std::pair<std::string, InequalityConstraints>>
IEVision60RobotMultiPhase::classifiedIConstraints() const {
  std::vector<std::pair<std::string, InequalityConstraints>>
      classified_constraints;
  if (params()->include_joint_limits) {
    classified_constraints.emplace_back("joint_limit", jointLimitConstraints());
  }
  if (params()->include_collision_free_s) {
    classified_constraints.emplace_back("collision_free_obstacle",
                                        obstacleCollisionFreeConstraints());
  }
  if (params()->include_collision_free_h) {
    classified_constraints.emplace_back("collision_free_hurdle",
                                        hurdleCollisionFreeConstraints());
  }
  if (params()->include_collision_free_z) {
    classified_constraints.emplace_back("collision_free_ground",
                                        groundCollisionFreeConstraints());
  }
  if (params()->include_torque_limits) {
    classified_constraints.emplace_back("torque_limit",
                                        torqueLimitConstraints());
  }
  if (params()->include_friction_cone) {
    classified_constraints.emplace_back("friction_cone",
                                        frictionConeConstraints());
  }
  if (params()->include_phase_duration_limits) {
    classified_constraints.emplace_back("phase_min_dt",
                                        phaseMinDurationConstraints());
  }
  return classified_constraints;
}

/* ************************************************************************* */
InequalityConstraints IEVision60RobotMultiPhase::iConstraints() const {
  InequalityConstraints constraints_all;
  for (const auto &[name, constraints] : classifiedIConstraints()) {
    constraints_all.add(constraints);
  }
  return constraints_all;
}

/* ************************************************************************* */
EqualityConstraints IEVision60RobotMultiPhase::eConstraints() const {
  size_t num_steps = numSteps();
  EqualityConstraints e_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    const IEVision60Robot &vision60 = robotAtStep(k);
    e_constraints.add(vision60.eConstraints(k));
  }
  if (params()->include_state_constraints) {
    e_constraints.add(phase_robots_.at(0).stateConstraints());
  }
  return e_constraints;
}

/* <=======================================================================> */
/* <========================== evaluate function ==========================> */
/* <=======================================================================> */

/* ************************************************************************* */
void PrintGraphError(const std::string &graph_name, const double &graph_error,
                     const double &sigma) {
  std::cout << std::setw(26) << graph_name + ":";
  std::cout << std::setw(14) << graph_error;
  std::cout << "->";
  std::cout << std::setw(14) << ComputeErrorNorm(graph_error, sigma);
  std::cout << "\n";
}

/* ************************************************************************* */
void PrintConstraintViolation(const std::string &name, const double &violation,
                              const double &sigma) {
  std::cout << std::setw(26) << name + ":";
  std::cout << std::setw(14) << violation;
  std::cout << "->";
  std::cout << std::setw(14) << violation * sigma;
  std::cout << "\n";
}

/* ************************************************************************* */
EqConsOptProblem::EvalFunc IEVision60RobotMultiPhase::costsEvalFunc() const {
  auto params_ = params();
  size_t num_steps = numSteps();
  auto graph = costs();
  auto e_constraints = eConstraints();
  auto i_constraints = iConstraints();

  auto classified_costs = classifiedCosts();
  auto classified_i_constraints = classifiedIConstraints();
  auto collocation_factors = collocationFactorsByStep();

  std::map<std::string, double> sigma_map{
      {"collocation", params_->sigma_q_col},
      {"actuation", params_->sigma_actuation},
      {"jerk", params_->sigma_jerk},
      {"state", params_->sigma_des_pose},
      {"phase_dt", params_->sigma_phase_dt},
      {"accel_penalty", params_->sigma_a_penalty},
      {"collision", params_->tol_cf},
      {"collision_inter", params_->tol_cf},
      {"cf_jerk", params_->sigma_cf_jerk},
      {"symmetry", params_->sigma_symmetry},
      {"joint_limit", params_->tol_jl},
      {"collision_free_obstacle", params_->tol_cf},
      {"collision_free_hurdle", params_->tol_cf},
      {"collision_free_ground", params_->tol_cf},
      {"torque_limit", params_->tol_tl},
      {"friction_cone", params_->tol_fc},
      {"phase_min_dt", params_->tol_phase_dt}};

  auto Evaluate = [=](const Values &values) {
    std::cout << std::setw(30) << "\033[1mcost:" << std::setw(14)
              << graph.error(values) << "\033[0m\n";
    if (params_->eval_details) {
      for (const auto &[name, graph] : classified_costs) {
        PrintGraphError(name, graph.error(values), sigma_map.at(name));
      }
    }
    if (params_->eval_collo_step) {
      std::cout << "\033[90m";
      for (size_t k = 0; k < num_steps; k++) {
        std::string name =
            "collo " + std::to_string(k) + "->" + std::to_string(k + 1);
        PrintGraphError(name, collocation_factors.at(k).error(values),
                        params_->sigma_pose_col);
      }
      std::cout << "\033[0m";
    }
    double e_violation = e_constraints.evaluateViolationL2Norm(values);
    double i_violation = i_constraints.evaluateViolationL2Norm(values);
    std::cout << std::setw(30) << "\033[1me_constraint vio:" << std::setw(14)
              << e_violation << "\033[0m\n";
    std::cout << std::setw(30) << "\033[1mi_constraint vio:" << std::setw(14)
              << i_violation << "\033[0m\n";
    if (params_->eval_details && i_violation > 1e-3) {
      for (const auto &[name, constraints] : classified_i_constraints) {
        PrintConstraintViolation(name,
                                 constraints.evaluateViolationL2Norm(values),
                                 sigma_map.at(name));
      }
    }
  };
  return Evaluate;
}

/* ************************************************************************* */
void IEVision60RobotMultiPhase::evaluateCollocation(
    const Values &values) const {
  size_t base_id = IEVision60Robot::base_id;
  size_t k = 0;
  double accum_base_error_t = 0, accum_base_error_r = 0;
  double accum_base_error_v = 0, accum_base_error_w = 0;

  for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    const auto &robot = phase_robots_.at(phase_idx);
    Key phase_key = PhaseKey(phase_idx);
    for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
         phase_step++) {

      auto torso_pose_col_factor =
          robot.multiPhaseLinkPoseCollocationFactor(base_id, k, phase_key);
      auto torso_twist_col_factor =
          robot.multiPhaseLinkTwistCollocationFactor(base_id, k, phase_key);

      auto error_pose_col = torso_pose_col_factor->unwhitenedError(values);
      auto error_twist_col = torso_twist_col_factor->unwhitenedError(values);
      auto error_r = error_pose_col.middleRows(0, 3);
      auto error_t = error_pose_col.middleRows(3, 3);
      auto error_v = error_twist_col.middleRows(3, 3);
      auto error_w = error_twist_col.middleRows(0, 3);

      accum_base_error_t += error_t.norm();
      accum_base_error_r += error_r.norm();
      accum_base_error_v += error_v.norm();
      accum_base_error_w += error_w.norm();

      std::cout << k << "\t" << error_t.norm() << "\t" << error_r.norm() << "\t"
                << error_v.norm() << "\t" << error_w.norm() << "\n";
      k++;
    }
  }
  std::cout << "accumulative translation error: " << accum_base_error_t << "\n";
  std::cout << "accumulative rotation error:    " << accum_base_error_r << "\n";
  std::cout << "accumulative velocity error:    " << accum_base_error_v << "\n";
  std::cout << "accumulative angular vel error: " << accum_base_error_w << "\n";
}

// /* *************************************************************************
// */ void IEVision60RobotMultiPhase::exportEvaluation(
//     const Values &values, const std::string &file_path) const {
//   auto graph = costs();
//   auto e_constraints = eConstraints();
//   auto i_constraints = iConstraints();

//   auto classified_costs = classifiedCosts();
//   auto classified_i_constraints = classifiedIConstraints();
//   auto collocation_factors = collocationFactorsByStep();

// }

} // namespace gtsam

/* <=======================================================================> */
/* <============================== scenarios ==============================> */
/* <=======================================================================> */

using namespace gtsam;
namespace quadruped_vertical_jump {
IEVision60RobotMultiPhase::shared_ptr
GetVision60MultiPhase(const IEVision60Robot::Params::shared_ptr &params,
                      const std::vector<size_t> &phase_num_steps) {

  auto phase_info_ground = IEVision60Robot::PhaseInfo::Ground();
  auto phase_info_air = IEVision60Robot::PhaseInfo::InAir();
  auto phase_info_boundary = IEVision60Robot::PhaseInfo::BoundaryLeave(
      *phase_info_ground, *phase_info_air);
  IEVision60Robot vision60_ground(params, phase_info_ground);
  IEVision60Robot vision60_air(params, phase_info_air);
  IEVision60Robot vision60_boundary(params, phase_info_boundary);

  std::vector<IEVision60Robot> phase_robots{vision60_ground, vision60_air};
  std::vector<IEVision60Robot> boundary_robots{vision60_boundary};
  return std::make_shared<IEVision60RobotMultiPhase>(
      phase_robots, boundary_robots, phase_num_steps);
}
} // namespace quadruped_vertical_jump

namespace quadruped_forward_jump {
IEVision60RobotMultiPhase::shared_ptr
GetVision60MultiPhase(const IEVision60Robot::Params::shared_ptr &params,
                      const std::vector<size_t> &phase_num_steps) {
  auto phase_info_ground = IEVision60Robot::PhaseInfo::Ground();
  auto phase_info_back = IEVision60Robot::PhaseInfo::BackOnGround();
  auto phase_info_air = IEVision60Robot::PhaseInfo::InAir();
  auto phase_info_boundary_gb = IEVision60Robot::PhaseInfo::BoundaryLeave(
      *phase_info_ground, *phase_info_back);
  auto phase_info_boundary_ba = IEVision60Robot::PhaseInfo::BoundaryLeave(
      *phase_info_back, *phase_info_air);
  IEVision60Robot vision60_ground(params, phase_info_ground);
  IEVision60Robot vision60_back(params, phase_info_back);
  IEVision60Robot vision60_air(params, phase_info_air);
  IEVision60Robot vision60_boundary_gb(params, phase_info_boundary_gb);
  IEVision60Robot vision60_boundary_ba(params, phase_info_boundary_ba);
  std::vector<IEVision60Robot> phase_robots{vision60_ground, vision60_back,
                                            vision60_air};
  std::vector<IEVision60Robot> boundary_robots{vision60_boundary_gb,
                                               vision60_boundary_ba};
  return std::make_shared<IEVision60RobotMultiPhase>(
      phase_robots, boundary_robots, phase_num_steps);
}
} // namespace quadruped_forward_jump

namespace quadruped_forward_jump_land {
IEVision60RobotMultiPhase::shared_ptr
GetVision60MultiPhase(const IEVision60Robot::Params::shared_ptr &params,
                      const std::vector<size_t> &phase_num_steps,
                      const double jump_distance) {
  auto phase_info_ground = IEVision60Robot::PhaseInfo::Ground();
  auto phase_info_back = IEVision60Robot::PhaseInfo::BackOnGround();
  auto phase_info_air = IEVision60Robot::PhaseInfo::InAir();
  auto phase_info_boundary_gb = IEVision60Robot::PhaseInfo::BoundaryLeave(
      *phase_info_ground, *phase_info_back);
  auto phase_info_boundary_ba = IEVision60Robot::PhaseInfo::BoundaryLeave(
      *phase_info_back, *phase_info_air);
  auto phase_info_boundary_ag = IEVision60Robot::PhaseInfo::BoundaryLand(
      *phase_info_air, *phase_info_ground);
  IEVision60Robot vision60_ground(params, phase_info_ground);
  IEVision60Robot vision60_back(params, phase_info_back);
  IEVision60Robot vision60_air(params, phase_info_air);
  IEVision60Robot vision60_ground_forward(params, phase_info_ground);

  IEVision60Robot vision60_boundary_gb(params, phase_info_boundary_gb);
  IEVision60Robot vision60_boundary_ba(params, phase_info_boundary_ba);
  IEVision60Robot vision60_boundary_ag(params, phase_info_boundary_ag);

  vision60_ground_forward.moveContactPoints(jump_distance);
  vision60_boundary_ag.moveContactPoints(jump_distance);

  std::vector<IEVision60Robot> phase_robots{
      vision60_ground, vision60_back, vision60_air, vision60_ground_forward};
  std::vector<IEVision60Robot> boundary_robots{
      vision60_boundary_gb, vision60_boundary_ba, vision60_boundary_ag};
  return std::make_shared<IEVision60RobotMultiPhase>(
      phase_robots, boundary_robots, phase_num_steps);
}
} // namespace quadruped_forward_jump_land
