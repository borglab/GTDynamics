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

#include <gtdynamics/cmcopt/IELMOptimizer.h>

using namespace std;

namespace gtdynamics {
using namespace gtsam;


/* ************************************************************************* */
Values IELMOptimizer::optimizeManifolds(
    const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
    const Values &unconstrainedValues) const {

  const LevenbergMarquardtParams &lmParams = ielm_params_.lmParams;

  // Construct manifold graph.
  std::map<Key, Key> keymap_var2manifold = varToManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph =
      ManifoldOptimizer::manifoldGraph(graph, keymap_var2manifold);

  // Construct initial state
  IELMState state(manifolds, unconstrainedValues, graph, manifold_graph,
                  lmParams.lambdaInitial, lmParams.lambdaFactor, 0);

  // check if we're already close enough
  if (state.error <= lmParams.errorTol) {
    details_->emplace_back(state);
    return state.baseValues();
  }

  // Iterative loop
  if (lmParams.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    std::cout << "Initial error: " << state.error << "\n";
    printIELMTrialTitle();
  }

  IELMState prev_state;
  do {
    prev_state = state;
    IELMIterationDetails iter_details = iterate(graph, state);
    state = IELMState::fromLastIteration(iter_details, graph, manifold_graph,
                                         lmParams);
    details_->push_back(iter_details);
  } while (state.iterations < lmParams.maxIterations &&
           !checkConvergence(prev_state, state) &&
           checkLambdaWithinLimits(state.lambda) && std::isfinite(state.error));
  details_->emplace_back(state);
  return state.baseValues();
}

/* ************************************************************************* */
IELMIterationDetails IELMOptimizer::iterate(const NonlinearFactorGraph &graph,
                                       const IELMState &state) const {

  const LevenbergMarquardtParams &lmParams = ielm_params_.lmParams;
  if (iecm_params_->retractorCreator->params()->useVaryingSigma) {
    *iecm_params_->retractorCreator->params()->metricSigmas =
        state.computeMetricSigmas(graph);
  }

  IELMIterationDetails iter_details(state);
  if (checkModeChange(graph, iter_details)) {
    if (lmParams.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
      printIELMTrial(state, iter_details.trials.back(), ielm_params_, true);
    }
    return iter_details;
  }

  // Set lambda for first trial.
  double lambda = state.lambda;
  double lambdaFactor = state.lambdaFactor;

  // Perform trials until any of follwing conditions is met
  // * 1) trial is successful
  // * 2) update is too small
  // * 3) lambda goes beyond limits
  while (true) {
    // Perform the trial.
    IELMTrial trial(state, graph, lambda, ielm_params_);
    if (lmParams.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
      printIELMTrial(state, trial, ielm_params_);
    }
    iter_details.trials.emplace_back(trial);

    // Check condition 1.
    if (trial.stepIsSuccessful) {
      break;
    }

    // Check condition 2.
    if (trial.linearUpdate.solveSuccessful) {
      double abs_change_tol = std::max(
          lmParams.absoluteErrorTol, lmParams.relativeErrorTol * state.error);
      if (trial.linearUpdate.costChange < abs_change_tol &&
          trial.nonlinearUpdate.costChange < abs_change_tol) {
        break;
      }
    }

    // Set lambda for next trial.
    trial.setNextLambda(lambda, lambdaFactor, lmParams);

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
    IELMIterationDetails &current_iter_details) const {
  if (details_->size() == 0) {
    return false;
  }

  // Find the first state in the sequence of states that have the same mode.
  int n = details_->size();
  int first_i = n - 1;
  while (first_i >= 0 && isSameMode(current_iter_details.state.manifolds,
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
    if (!trial.stepIsSuccessful && trial.linearUpdate.solveSuccessful) {
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
      identifyChangeIndices(current_iter_details.state.manifolds,
                            details_->at(iter_idx)
                                .trials.at(trial_idx)
                                .nonlinearUpdate.newManifolds);

  // Condition2(2): most recent failed trial results in other mode
  if (change_indices_map.size() == 0) {
    return false;
  }

  auto approach_indices_map = identifyApproachingIndices(
      init_iter_dertails.state.manifolds, current_iter_details.state.manifolds,
      change_indices_map, ielm_params_.boundaryApproachRateThreshold);

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
  return lambda <= ielm_params_.lmParams.lambdaUpperBound &&
         lambda >= ielm_params_.lmParams.lambdaLowerBound;
}

/* ************************************************************************* */
bool IELMOptimizer::checkConvergence(const IELMState &prev_state,
                                     const IELMState &state) const {

  if (state.error <= ielm_params_.lmParams.errorTol)
    return true;

  // check if mode changes
  if (!isSameMode(prev_state.manifolds, state.manifolds)) {
    return false;
  }
  // check if diverges
  double absoluteDecrease = prev_state.error - state.error;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / prev_state.error;
  bool converged =
      (ielm_params_.lmParams.relativeErrorTol &&
       (relativeDecrease <= ielm_params_.lmParams.relativeErrorTol)) ||
      (absoluteDecrease <= ielm_params_.lmParams.absoluteErrorTol);
  return converged;
}

} /* namespace gtdynamics */
