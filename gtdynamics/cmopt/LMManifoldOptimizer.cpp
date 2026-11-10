/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LMManifoldOptimizer.cpp
 * @brief   A nonlinear optimizer that uses the Levenberg-Marquardt trust-region
 * scheme
 * @author  Yetong Zhang
 * @date    Oct 26, 2023
 */

#include <gtdynamics/cmopt/LMManifoldOptimizer.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
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

namespace gtdynamics {

/* ************************************************************************* */
Values LMManifoldOptimizer::optimize(
    const NonlinearFactorGraph &costs,
    const NonlinearEqualityConstraints &constraints,
    const Values &init_values) const {
  auto mopt_problem = initializeManifoldOptimizationProblem(costs, constraints, init_values);
  return optimize(costs, mopt_problem);
}

/* ************************************************************************* */
Values LMManifoldOptimizer::optimize(
    const NonlinearFactorGraph &graph, const ManifoldOptimizationProblem &mopt_problem) const {
  // Construct initial state
  LMState state(graph, mopt_problem, params_.lambdaInitial,
                params_.lambdaFactor, 0);

  // Final projection shares the same recovery path as the GTSAM wrappers.
  const auto recover = [&](const LMState& finalState) {
    if (!p_.retractFinalValues) return finalState.baseValues();
    Values finalValues = finalState.unconstrainedValues;
    for (const auto& entry : finalState.points)
      finalValues.insert(entry.first, entry.second);
    return baseValues(mopt_problem, finalValues);
  };

  // check if we're already close enough
  if (state.error <= params_.errorTol || mopt_problem.values.empty()) {
    details_->emplace_back(state);
    return recover(state);
  }

  // Iterative loop
  if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    std::cout << "Initial error: " << state.error << "\n";
    LMTrial::printTitle();
  }

  LMState prev_state;
  do {
    // Copy-construct points before moving the snapshot: map copy assignment
    // may recycle nodes across components with different immutable bindings.
    prev_state = LMState(state);
    LMIterationDetails iter_details = iterate(graph, mopt_problem.graph, state);
    state = LMState::fromLastIteration(iter_details, graph, params_);
    details_->push_back(iter_details);
  } while (state.iterations < params_.maxIterations &&
           !checkConvergence(prev_state, state) &&
           checkLambdaWithinLimits(state.lambda) && std::isfinite(state.error));
  details_->emplace_back(state);
  return recover(state);
}

/* ************************************************************************* */
LMIterationDetails
LMManifoldOptimizer::iterate(const NonlinearFactorGraph &graph,
                             const NonlinearFactorGraph &manifold_graph,
                             const LMState &state) const {
  LMIterationDetails iter_details(state);

  // Set lambda for first trial.
  double lambda = state.lambda;
  double lambdaFactor = state.lambdaFactor;

  // Perform trials until any of follwing conditions is met
  // * 1) trial is successful
  // * 2) update is too small
  // * 3) lambda goes beyond limits
  while (true) {
    // Perform the trial.
    LMTrial trial(state, graph, manifold_graph, lambda, params_);
    if (params_.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
      trial.print(state);
    }
    iter_details.trials.emplace_back(trial);

    // Check condition 1.
    if (trial.stepIsSuccessful) {
      break;
    }

    // Check condition 2.
    if (trial.linearUpdate.solveSuccessful) {
      double abs_change_tol = std::max(params_.absoluteErrorTol,
                                       params_.relativeErrorTol * state.error);
      if (trial.linearUpdate.costChange < abs_change_tol &&
          trial.nonlinearUpdate.costChange < abs_change_tol) {
        break;
      }
    }

    // Set lambda for next trial.
    trial.setNextLambda(lambda, lambdaFactor, params_);

    // Check condition 3.
    if (!checkLambdaWithinLimits(lambda)) {
      break;
    }
  }
  return iter_details;
}

/* ************************************************************************* */
bool LMManifoldOptimizer::checkLambdaWithinLimits(const double &lambda) const {
  return lambda <= params_.lambdaUpperBound &&
         lambda >= params_.lambdaLowerBound;
}

/* ************************************************************************* */
bool LMManifoldOptimizer::checkConvergence(const LMState &prev_state,
                                           const LMState &state) const {

  if (state.error <= params_.errorTol)
    return true;

  // check if diverges
  double absoluteDecrease = prev_state.error - state.error;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / prev_state.error;
  bool converged = (params_.relativeErrorTol &&
                    (relativeDecrease <= params_.relativeErrorTol)) ||
                   (absoluteDecrease <= params_.absoluteErrorTol);
  return converged;
}

/* ************************************************************************* */
Values LMManifoldOptimizer::optimize(
    const NonlinearFactorGraph& graph, const ECMPoints& points,
    const Values& unconstrainedValues) const {
  auto problem = initializeManifoldOptimizationProblem(
      graph, points, unconstrainedValues);
  return optimize(graph, problem);
}

} /* namespace gtdynamics */
