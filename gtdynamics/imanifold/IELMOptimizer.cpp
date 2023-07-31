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

#include "imanifold/IELMOptimizerState.h"
#include "imanifold/IEManifoldOptimizer.h"
#include "imanifold/IERetractor.h"
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

#ifdef GTSAM_USE_BOOST_FEATURES
#include <gtsam/base/timing.h>
#endif

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
  double currentError = state.error;

  // check if we're already close enough
  if (currentError <= params_.errorTol) {
    return IEOptimizer::CollectManifoldValues(state.manifolds);
  }

  // Iterative loop
  double newError;
  do {
    currentError = state.error;
    IELMIterDetails iter_details = iterate(graph, state);
    state = IELMState::FromLastIteration(iter_details, graph, params_);
    details_->push_back(iter_details);
    newError = state.error;
  } while (state.iterations < params_.maxIterations &&
           !checkConvergence(params_.relativeErrorTol, params_.absoluteErrorTol,
                             params_.errorTol, currentError, newError) &&
           state.lambda <= params_.lambdaUpperBound &&
           state.lambda >= params_.lambdaLowerBound &&
           std::isfinite(currentError));
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

  IELMTrial trial;
  trial.setLambda(state);

  while (true) {
    if (trial.lambda > params_.lambdaUpperBound) {
      break;
    }
    if (trial.lambda < params_.lambdaLowerBound) {
      break;
    }
    tryLambda(graph, state, trial);
    iter_details.trials.emplace_back(trial);

    IELMTrial next_trial;
    if (trial.step_is_successful) {
      break;
    } else {
      trial.setNextLambda(next_trial.lambda, next_trial.lambda_factor, params_);
    }
    trial = next_trial;
  }
  return iter_details;
}

bool IsSameMode(const IEManifoldValues &manifolds1,
                const IEManifoldValues &manifolds2) {
  for (const auto &it : manifolds1) {
    const IndexSet &indices1 = it.second.activeIndices();
    const IndexSet &indices2 = manifolds2.at(it.first).activeIndices();
    if (indices1.size() != indices2.size()) {
      return false;
    }
    if (!std::equal(indices1.begin(), indices1.end(), indices2.begin())) {
      return false;
    }
  }
  return true;
}

IndexSetMap IdentifyChangeIndices(const IEManifoldValues &manifolds,
                                  const IEManifoldValues &new_manifolds) {
  IndexSetMap change_indices_map;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const IndexSet &indices = it.second.activeIndices();
    const IndexSet &new_indices = new_manifolds.at(key).activeIndices();
    IndexSet change_indices;
    for (const auto &idx : new_indices) {
      if (indices.find(idx) == indices.end()) {
        change_indices.insert(idx);
      }
    }
    if (change_indices.size() > 0) {
      change_indices_map.insert({key, change_indices});
    }
  }
  return change_indices_map;
}

IndexSetMap IdentifyApproachingIndices(const IEManifoldValues &manifolds,
                                       const IEManifoldValues &new_manifolds,
                                       const IndexSetMap &change_indices_map) {
  IndexSetMap approach_indices_map;
  for (const auto &it : change_indices_map) {
    const Key &key = it.first;
    const IndexSet &change_indices = it.second;
    const IEConstraintManifold &manifold = manifolds.at(key);
    const IEConstraintManifold &new_manifold = new_manifolds.at(key);
    auto i_constraints = manifolds.at(key).iConstraints();
    IndexSet approach_indices;

    for (const auto &idx : change_indices) {
      const auto &constraint = i_constraints->at(idx);
      double eval = (*constraint)(manifold.values());
      double new_eval = (*constraint)(new_manifold.values());
      double approach_rate = eval / new_eval;
      std::cout << "eval: " << eval << "\n";
      std::cout << "new_eval: " << new_eval << "\n";
      std::cout << "approach_rate: " << approach_rate << "\n";
      if (approach_rate > 3) {
        approach_indices.insert(idx);
      }
    }
    if (approach_indices.size() > 0) {
      approach_indices_map.insert({key, change_indices});
    }
  }
  return approach_indices_map;
}

IEManifoldValues MoveToBoundaries(const IEManifoldValues &manifolds,
                                  const IndexSetMap &approach_indices_map) {
  IEManifoldValues new_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const auto &manifold = it.second;
    if (approach_indices_map.find(key) == approach_indices_map.end()) {
      new_manifolds.insert({key, manifold});
    } else {
      new_manifolds.insert(
          {key, manifold.moveToBoundary(approach_indices_map.at(key))});
    }
  }
  return new_manifolds;
}

/* ************************************************************************* */
bool IELMOptimizer::checkModeChange(
    const NonlinearFactorGraph &graph,
    IELMIterDetails &current_iter_details) const {
  if (details_->size() < 2) {
    return false;
  }

  // find the furthest state that have the same mode
  int n = details_->size();
  int i = n - 1;
  while (i >= 0 && IsSameMode(current_iter_details.state.manifolds,
                              details_->at(i).state.manifolds)) {
    i--;
  }
  i++;

  // Condition1: mode remain unchanged in consecutive states
  if (i <= n - 1) {
    const auto &init_iter_dertails = details_->at(i);
    const auto &prev_iter_details = details_->back();
    if (prev_iter_details.trials.size() == 2) {

      // Condition2: trial uses other mode and fails
      IndexSetMap change_indices_map =
          IdentifyChangeIndices(current_iter_details.state.manifolds,
                                prev_iter_details.trials.at(0).new_manifolds);
      if (change_indices_map.size() > 0) {

        // Condition3: approaching boundary with decent rate
        auto approach_indices_map = IdentifyApproachingIndices(
            init_iter_dertails.state.manifolds,
            current_iter_details.state.manifolds, change_indices_map);

        if (approach_indices_map.size() > 0) {
          // Enforce approaching indices;
          IELMTrial trial;
          trial.setLambda(current_iter_details.state);
          trial.forced_indices_map = approach_indices_map;
          trial.new_manifolds = MoveToBoundaries(
              current_iter_details.state.manifolds, approach_indices_map);
          trial.new_error =
              graph.error(CollectManifoldValues(trial.new_manifolds));
          current_iter_details.trials.emplace_back(trial);
          return true;
        }
      }
    }
  }
  return false;
}

/* ************************************************************************* */
void IELMOptimizer::tryLambda(const NonlinearFactorGraph &graph,
                              const IELMState &currentState,
                              IELMTrial &trial) const {

  // auto start = std::chrono::high_resolution_clock::now();

  // compute linear update
  // std::cout << "compute Delta\n";
  trial.solve_successful = trial.computeDelta(graph, currentState, params_);
  if (!trial.solve_successful) {
    trial.step_is_successful = false;
    return;
  }

  // retract and perform nonlinear update
  // std::cout << "compute new manifolds\n";
  trial.computeNewManifolds(currentState);

  // decide if accept or reject trial
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

  if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    trial.print(currentState);
    // auto end = std::chrono::high_resolution_clock::now();
    // double iterationTime =
    //     std::chrono::duration_cast<std::chrono::microseconds>(end - start)
    //         .count() /
    //     1e6;
  }

}

/* ************************************************************************* */
bool IELMOptimizer::checkConvergence(double relativeErrorTreshold,
                                     double absoluteErrorTreshold,
                                     double errorThreshold, double currentError,
                                     double newError) const {

  if (newError <= errorThreshold)
    return true;

  // check if diverges
  double absoluteDecrease = currentError - newError;

  if (newError > currentError) {
    return false;
  }

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / currentError;
  bool converged =
      (relativeErrorTreshold && (relativeDecrease <= relativeErrorTreshold)) ||
      (absoluteDecrease <= absoluteErrorTreshold);
  return converged;
}

} /* namespace gtsam */
