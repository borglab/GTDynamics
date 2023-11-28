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

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/scenarios/IEQuadrupedUtils.h>
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

/* ************************************************************************* */
InequalityConstraints IEVision60RobotMultiPhase::iConstraints() const {
  size_t num_steps = numSteps();
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    const IEVision60Robot &vision60 = robotAtStep(k);
    i_constraints.add(vision60.iConstraints(k));
  }
  if (params()->include_phase_duration_limits) {
    i_constraints.add(phaseMinDurationConstraints());
  }
  return i_constraints;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::collocationCosts() const {
  NonlinearFactorGraph graph;
  size_t start_k = 0;
  for (size_t phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
    size_t end_k = start_k + phase_num_steps_[phase_idx];
    graph.add(phase_robots_.at(phase_idx).multiPhaseCollocationCosts(
        start_k, end_k, phase_idx));
    start_k = end_k;
  }
  return graph;
}

/* ************************************************************************* */
gtdynamics::InequalityConstraints
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
NonlinearFactorGraph IEVision60RobotMultiPhase::actuationCosts() const {
  NonlinearFactorGraph graph;
  const auto &params = phase_robots_[0].params;
  size_t num_steps = numSteps();
  if (params->actuation_cost_option == ACTUATION_RMSE_TORQUE) {
    for (size_t k = 0; k <= num_steps; k++) {
      graph.add(robotAtStep(k).actuationRmseTorqueCosts(k));
    }
  } else if (params->actuation_cost_option == ACTUATION_IMPULSE_SQR ||
             params->actuation_cost_option == ACTUATION_IMPULSE) {
    bool apply_sqrt = params->actuation_cost_option == ACTUATION_IMPULSE;
    size_t k = 0;
    for (int phase_idx = 0; phase_idx < phase_num_steps_.size(); phase_idx++) {
      for (size_t phase_step = 0; phase_step < phase_num_steps_.at(phase_idx);
           phase_step++) {
        graph.add(
            robotAtStep(k).actuationImpulseCosts(k, phase_idx, apply_sqrt));
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
        graph.add(robotAtStep(k).actuationWorkCosts(k, apply_sqrt));
        k++;
      }
    }
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
IEVision60RobotMultiPhase::phaseDurationPriorCosts() const {
  NonlinearFactorGraph graph;
  for (size_t phase_idx = 0; phase_idx < phase_robots_.size(); phase_idx++) {
    graph.addPrior(PhaseKey(0), params()->phase_prior_dt.at(phase_idx),
                   noiseModel::Isotropic::Sigma(1, params()->sigma_phase_dt));
  }
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph IEVision60RobotMultiPhase::costs() const {
  NonlinearFactorGraph graph;
  if (params()->include_collocation_costs) {
    graph.add(collocationCosts());
  }
  if (params()->include_actuation_costs) {
    graph.add(actuationCosts());
  }
  if (params()->include_jerk_costs) {
    graph.add(phase_robots_.at(0).jerkCosts(numSteps()));
  }
  if (params()->include_state_costs) {
    graph.add(phase_robots_.at(0).stateCosts());
  }
  if (params()->include_phase_duration_prior_costs) {
    graph.add(phaseDurationPriorCosts());
  }
  return graph;
}

/* ************************************************************************* */
gtdynamics::EqConsOptProblem::EvalFunc
IEVision60RobotMultiPhase::costsEvalFunc() const {
  NonlinearFactorGraph graph_collo, graph_actuation, graph_jerk, graph_state,
      graph_phase_duration;
  auto params_ = params();
  if (params_->include_collocation_costs) {
    graph_collo = collocationCosts();
  }
  if (params_->include_actuation_costs) {
    graph_actuation = actuationCosts();
  }
  if (params_->include_jerk_costs) {
    graph_jerk = phase_robots_.at(0).jerkCosts(numSteps());
  }
  if (params_->include_state_costs) {
    graph_state = phase_robots_.at(0).stateCosts();
  }
  if (params_->include_phase_duration_prior_costs) {
    graph_phase_duration = phaseDurationPriorCosts();
  }
  auto e_constraints = eConstraints();
  auto i_constraints = iConstraints();
  auto Evaluate = [=](const Values &values) {
    if (params_->include_collocation_costs) {
      std::cout << "collocation costs:\t" << graph_collo.error(values) << "\n";
    }
    if (params_->include_actuation_costs) {
      std::cout << "actuation costs:  \t" << graph_actuation.error(values)
                << "\n";
    }
    if (params_->include_jerk_costs) {
      std::cout << "jerk costs:       \t" << graph_jerk.error(values) << "\n";
    }
    if (params_->include_state_costs) {
      std::cout << "state costs:      \t" << graph_state.error(values) << "\n";
    }
    if (params_->include_phase_duration_prior_costs) {
      std::cout << "phase prior costs:\t" << graph_phase_duration.error(values)
                << "\n";
    }
    std::cout << "e_constraint vio:\t" << e_constraints.evaluateViolationL2Norm(values) << "\n";
    std::cout << "i_constraint vio:\t" << i_constraints.evaluateViolationL2Norm(values) << "\n";
  };
  return Evaluate;
}

} // namespace gtsam

using namespace gtsam;
namespace quadruped_vertical_jump {
gtsam::IEVision60RobotMultiPhase
GetVision60MultiPhase(const gtsam::IEVision60Robot::Params::shared_ptr &params,
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
  return IEVision60RobotMultiPhase(phase_robots, boundary_robots,
                                   phase_num_steps);
}
} // namespace quadruped_vertical_jump

namespace quadruped_forward_jump {
gtsam::IEVision60RobotMultiPhase
GetVision60MultiPhase(const gtsam::IEVision60Robot::Params::shared_ptr &params,
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
  return IEVision60RobotMultiPhase(phase_robots, boundary_robots,
                                   phase_num_steps);
}
} // namespace quadruped_forward_jump
