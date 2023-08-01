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
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using namespace std;

namespace gtsam {

typedef internal::LevenbergMarquardtState State;

/* ************************************************************************* */
Values IELMOptimizer::optimizeManifolds(
    const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
    gtdynamics::ConstrainedOptResult *intermediate_result) const {

  // Construct initial state
  IELMState state(manifolds, graph, 0);
  state.lambda = params_.lambdaInitial;
  state.lambda_factor = params_.lambdaFactor;

  // check if we're already close enough
  if (state.error <= params_.errorTol) {
    details_->emplace_back(state);
    return IEOptimizer::CollectManifoldValues(state.manifolds);
  }

  // Iterative loop
  IELMState prev_state;
  do {
    prev_state = state;
    IELMIterDetails iter_details = iterate(graph, state);
    state = IELMState::FromLastIteration(iter_details, graph, params_);
    details_->push_back(iter_details);
  } while (state.iterations < params_.maxIterations &&
           !checkConvergence(prev_state, state) &&
           checkLambdaWithinLimits(state.lambda) && std::isfinite(state.error));
  details_->emplace_back(state);
  return IEOptimizer::CollectManifoldValues(state.manifolds);
}

/* ************************************************************************* */
IELMIterDetails IELMOptimizer::iterate(const NonlinearFactorGraph &graph,
                                       const IELMState &state) const {

  IELMIterDetails iter_details(state);
  if (checkModeChange(graph, iter_details)) {
    return iter_details;
  }

  // Set lambda for first trial.
  IELMTrial trial;
  trial.setLambda(state);

  // Perform trials until any of follwing conditions is met
  // * 1) trial is successful
  // * 2) update is too small
  // * 3) lambda goes beyond limits
  while (true) {
    // Perform the trial.
    tryLambda(graph, state, trial);
    iter_details.trials.emplace_back(trial);

    // Check condition 1.
    if (trial.step_is_successful) {
      break;
    }

    // Check condition 2.
    if (trial.solve_successful) {
      double abs_change_tol = std::max(params_.absoluteErrorTol,
                                       params_.relativeErrorTol * state.error);
      if (trial.linear_cost_change < abs_change_tol) {
        if (trial.nonlinear_cost_change < abs_change_tol) {
          break;
        }
      }
    }

    // Set lambda for next trial.
    IELMTrial next_trial;
    trial.setNextLambda(next_trial.lambda, next_trial.lambda_factor, params_);
    trial = next_trial;

    // Check condition 3.
    if (!checkLambdaWithinLimits(trial.lambda)) {
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
    if (!details_->at(iter_idx).trials.at(trial_idx).step_is_successful) {
      failed_trial_exists = true;
      break;
    }
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

  IndexSetMap change_indices_map = IdentifyChangeIndices(
      current_iter_details.state.manifolds,
      details_->at(iter_idx).trials.at(trial_idx).new_manifolds);

  // Condition2(2): most recent failed trial results in other mode
  if (change_indices_map.size() == 0) {
    return false;
  }

  auto approach_indices_map = IdentifyApproachingIndices(
      init_iter_dertails.state.manifolds, current_iter_details.state.manifolds,
      change_indices_map);

  // Condition3: approaching boundary with decent rate
  if (approach_indices_map.size() == 0) {
    return false;
  }

  // Enforce approaching indices;
  IELMTrial trial;
  trial.setLambda(current_iter_details.state);
  trial.forced_indices_map = approach_indices_map;
  trial.new_manifolds = MoveToBoundaries(current_iter_details.state.manifolds,
                                         approach_indices_map);
  trial.new_error = graph.error(CollectManifoldValues(trial.new_manifolds));
  current_iter_details.trials.emplace_back(trial);
  return true;
}

/* ************************************************************************* */
void IELMOptimizer::tryLambda(const NonlinearFactorGraph &graph,
                              const IELMState &currentState,
                              IELMTrial &trial) const {

  auto start = std::chrono::high_resolution_clock::now();

  // std::cout << "compute Delta\n";
  trial.solve_successful = trial.computeDelta(graph, currentState, params_);
  if (!trial.solve_successful) {
    trial.step_is_successful = false;
    // std::cout << "solve not successful\n";
    return;
  }

  // std::cout << "compute new manifolds\n";
  trial.computeNewManifolds(currentState);

  // std::cout << "decide if accept or reject trial\n";
  Values newValues = CollectManifoldValues(trial.new_manifolds);
  trial.new_error = graph.error(newValues);
  trial.nonlinear_cost_change = currentState.error - trial.new_error;
  trial.model_fidelity = trial.nonlinear_cost_change / trial.linear_cost_change;
  if (trial.linear_cost_change <=
      std::numeric_limits<double>::epsilon() * trial.old_linear_error) {
    trial.step_is_successful = false;
  } else {
    trial.step_is_successful = trial.model_fidelity > params_.minModelFidelity;
  }

  auto end = std::chrono::high_resolution_clock::now();
  trial.trial_time =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1e6;

  if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    trial.print(currentState);
  }
}

/* ************************************************************************* */
bool IELMOptimizer::checkLambdaWithinLimits(const double &lambda) const {
  return lambda <= params_.lambdaUpperBound &&
         lambda >= params_.lambdaLowerBound;
}

/* ************************************************************************* */
bool IELMOptimizer::checkConvergence(const IELMState &prev_state,
                                     const IELMState &state) const {

  if (state.error <= params_.errorTol)
    return true;

  // check if mode changes
  std::cout << "check is Same Mode\n";
  if (!IsSameMode(prev_state.manifolds, state.manifolds)) {
    std::cout << "check is Same Mode done\n";
    return false;
  }
  std::cout << "check is Same Mode done\n";
  // check if diverges
  double absoluteDecrease = prev_state.error - state.error;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / prev_state.error;
  bool converged = (params_.relativeErrorTol &&
                    (relativeDecrease <= params_.relativeErrorTol)) ||
                   (absoluteDecrease <= params_.absoluteErrorTol);
  return converged;
}

} /* namespace gtsam */
