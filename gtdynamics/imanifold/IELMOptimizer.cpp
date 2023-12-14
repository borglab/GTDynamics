/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IELMOptimizer.cpp
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Richard Roberts
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @date    Feb 26, 2012
 */

#include <gtdynamics/imanifold/IELMOptimizer.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Values IELMOptimizer::optimizeManifolds(
    const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
    const Values &unconstrained_values,
    gtdynamics::ConstrainedOptResult *intermediate_result) const {

  // Construct initial state
  const LevenbergMarquardtParams &lm_params = ielm_params_.lm_params;
  IELMState state(manifolds, unconstrained_values, graph,
                  lm_params.lambdaInitial, lm_params.lambdaFactor, 0);

  // check if we're already close enough
  if (state.error <= lm_params.errorTol) {
    details_->emplace_back(state);
    return state.baseValues();
  }

  // Iterative loop
  if (lm_params.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    std::cout << "Initial error: " << state.error << "\n";
    PrintIELMTrialTitle();
  }

  IELMState prev_state;
  do {
    prev_state = state;
    IELMIterDetails iter_details = iterate(graph, state);
    state = IELMState::FromLastIteration(iter_details, graph, lm_params);
    details_->push_back(iter_details);
  } while (state.iterations < lm_params.maxIterations &&
           !checkConvergence(prev_state, state) &&
           checkLambdaWithinLimits(state.lambda) && std::isfinite(state.error));
  details_->emplace_back(state);
  return state.baseValues();
}

/* ************************************************************************* */
IELMIterDetails IELMOptimizer::iterate(const NonlinearFactorGraph &graph,
                                       const IELMState &state) const {

  const LevenbergMarquardtParams &lm_params = ielm_params_.lm_params;
  if (iecm_params_->retractor_creator->params()->use_varying_sigma) {
    *iecm_params_->retractor_creator->params()->metric_sigmas =
        state.computeMetricSigmas(graph);
  }

  IELMIterDetails iter_details(state);
  if (checkModeChange(graph, iter_details)) {
    if (lm_params.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
      PrintIELMTrial(state, iter_details.trials.back(), ielm_params_, true);
    }
    return iter_details;
  }

  // Set lambda for first trial.
  double lambda = state.lambda;
  double lambda_factor = state.lambda_factor;

  // Perform trials until any of follwing conditions is met
  // * 1) trial is successful
  // * 2) update is too small
  // * 3) lambda goes beyond limits
  while (true) {
    // Perform the trial.
    IELMTrial trial(state, graph, lambda, lm_params);
    if (lm_params.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
      PrintIELMTrial(state, trial, ielm_params_);
    }
    iter_details.trials.emplace_back(trial);

    // Check condition 1.
    if (trial.step_is_successful) {
      break;
    }

    // Check condition 2.
    if (trial.linear_update.solve_successful) {
      double abs_change_tol = std::max(
          lm_params.absoluteErrorTol, lm_params.relativeErrorTol * state.error);
      if (trial.linear_update.cost_change < abs_change_tol &&
          trial.nonlinear_update.cost_change < abs_change_tol) {
        break;
      }
    }

    // Set lambda for next trial.
    trial.setNextLambda(lambda, lambda_factor, lm_params);

    // Check condition 3.
    if (!checkLambdaWithinLimits(lambda)) {
      break;
    }
  }
  return iter_details;
}

/* ************************************************************************* */
bool IELMOptimizer::checkModeChange(
    const NonlinearFactorGraph &graph,
    IELMIterDetails &current_iter_details) const {
  if (details_->size() == 0) {
    return false;
  }

  // Find the first state in the sequence of states that have the same mode.
  int n = details_->size();
  int first_i = n - 1;
  while (first_i >= 0 && IsSameMode(current_iter_details.state.manifolds,
                                    details_->at(first_i).state.manifolds)) {
    first_i--;
  }
  first_i++;

  // Condition1: mode remain unchanged in consecutive states
  if (first_i >= n) {
    return false;
  }

  const auto &init_iter_dertails = details_->at(first_i);
  const auto &prev_iter_details = details_->back();

  // Find the last failed trial.
  size_t iter_idx = details_->size() - 1;
  size_t trial_idx = details_->back().trials.size() - 1;
  bool failed_trial_exists = false;
  while (true) {
    const auto &trial = details_->at(iter_idx).trials.at(trial_idx);
    if (!trial.step_is_successful) {
      failed_trial_exists = true;
      break;
    }
    // change to previous trial
    if (trial_idx == 0) {
      if (iter_idx == first_i) {
        break;
      }
      iter_idx--;
      trial_idx = details_->at(iter_idx).trials.size() - 1;
    } else {
      trial_idx--;
    }
  }

  // Condition2(1): exists failed trial
  if (!failed_trial_exists) {
    return false;
  }

  IndexSetMap change_indices_map =
      IdentifyChangeIndices(current_iter_details.state.manifolds,
                            details_->at(iter_idx)
                                .trials.at(trial_idx)
                                .nonlinear_update.new_manifolds);

  // Condition2(2): most recent failed trial results in other mode
  if (change_indices_map.size() == 0) {
    return false;
  }

  auto approach_indices_map = IdentifyApproachingIndices(
      init_iter_dertails.state.manifolds, current_iter_details.state.manifolds,
      change_indices_map, ielm_params_.boundary_approach_rate_threshold);

  // Condition3: approaching boundary with decent rate
  if (approach_indices_map.size() == 0) {
    return false;
  }

  // Enforce approaching indices;
  IELMTrial trial(current_iter_details.state, graph, approach_indices_map);
  current_iter_details.trials.emplace_back(trial);
  return true;
}

/* ************************************************************************* */
bool IELMOptimizer::checkLambdaWithinLimits(const double &lambda) const {
  return lambda <= ielm_params_.lm_params.lambdaUpperBound &&
         lambda >= ielm_params_.lm_params.lambdaLowerBound;
}

/* ************************************************************************* */
bool IELMOptimizer::checkConvergence(const IELMState &prev_state,
                                     const IELMState &state) const {

  if (state.error <= ielm_params_.lm_params.errorTol)
    return true;

  // check if mode changes
  if (!IsSameMode(prev_state.manifolds, state.manifolds)) {
    return false;
  }
  // check if diverges
  double absoluteDecrease = prev_state.error - state.error;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / prev_state.error;
  bool converged =
      (ielm_params_.lm_params.relativeErrorTol &&
       (relativeDecrease <= ielm_params_.lm_params.relativeErrorTol)) ||
      (absoluteDecrease <= ielm_params_.lm_params.absoluteErrorTol);
  return converged;
}

} /* namespace gtsam */
