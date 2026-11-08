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
Values IELMOptimizer::optimize(
    const NonlinearFactorGraph &graph, const CMPoints &points,
    const Values &unconstrainedValues) const {
  validatePoints(points, unconstrainedValues);
  if (ielm_params_.linearSolver == IELMParams::LinearSolver::Gaussian)
    for (const auto& [key, point] : points)
      if (point.manifold()->inequalityCount(point))
        throw std::invalid_argument("Gaussian CMC solves require empty inequalities");
  timings_ = {};
  details_->clear();
  mode_.reset();
  modeStartResiduals_.clear();
  lastFailedMode_.reset();

  const LevenbergMarquardtParams &lmParams = ielm_params_.lmParams;

  // Construct the cost graph on CMPoint values.
  std::map<Key, Key> ambientToPoint = varToManifoldKeyMap(points);
  NonlinearFactorGraph pointGraph =
      IEOptimizer::pointGraph(graph, points);

  // Construct initial state
  IELMState state(points, unconstrainedValues, graph, pointGraph,
                  lmParams.lambdaInitial, lmParams.lambdaFactor, 0);
  timings_.linearizationSeconds += state.linearizationSeconds;

  if (ielm_params_.iterationCallback) ielm_params_.iterationCallback(state);

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
    for (const auto& trial : iter_details.trials) {
      timings_.linearSolveSeconds += trial.linearUpdate.linearSolveSeconds;
      timings_.retractionSeconds += trial.retractionSeconds;
      timings_.gaussianSolves += trial.linearUpdate.gaussianSolves;
      timings_.qpSolves += trial.linearUpdate.qpSolves;
    }
    state = IELMState::fromLastIteration(iter_details, graph, pointGraph,
                                         lmParams);
    timings_.linearizationSeconds += state.linearizationSeconds;
    rememberMode(iter_details);
    details_->push_back(iter_details);
    if (ielm_params_.historyWindow &&
        details_->size() > ielm_params_.historyWindow)
      details_->erase(details_->begin());
    if (ielm_params_.iterationCallback) ielm_params_.iterationCallback(state);
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
  if (std::none_of(iter_details.trials.begin(), iter_details.trials.end(),
                   [](const IELMTrial &trial) {
                     return trial.linearUpdate.solveSuccessful;
                   }))
    throw std::runtime_error(
        "CMC LM exhausted damping trials: " +
        iter_details.trials.back().linearUpdate.failureReason);
  return iter_details;
}

/* ************************************************************************* */
namespace {
IndexSetMap activeModes(const CMPoints &points) {
  IndexSetMap result;
  for (const auto &[key, point] : points)
    result.emplace(key, point.activeIndices());
  return result;
}
}  // namespace

void IELMOptimizer::rememberMode(const IELMIterationDetails &iteration) const {
  const auto mode = activeModes(iteration.state.points);
  if (!mode_ || *mode_ != mode) {
    mode_ = mode;
    lastFailedMode_.reset();
    modeStartResiduals_.clear();
    for (const auto &[key, point] : iteration.state.points) {
      const size_t count = point.manifold()->inequalityCount(point);
      Vector residuals(count);
      for (size_t i = 0; i < count; ++i)
        residuals(i) = point.inequalityResidual(i)(0);
      modeStartResiduals_.emplace(key, std::move(residuals));
    }
  }
  // Match the historical search: last unsuccessful, successfully linearized
  // trial, even when it stayed in the same mode. Projection failures do not
  // have a candidate state and cannot supply a boundary decision.
  for (auto trial = iteration.trials.rbegin(); trial != iteration.trials.rend();
       ++trial)
    if (!trial->stepIsSuccessful && trial->linearUpdate.solveSuccessful) {
      lastFailedMode_ = activeModes(trial->nonlinearUpdate.newPoints);
      break;
    }
}

bool IELMOptimizer::checkModeChange(
    const NonlinearFactorGraph &graph,
    IELMIterationDetails &current_iter_details) const {
  const auto &points = current_iter_details.state.points;
  if (!mode_ || *mode_ != activeModes(points) || !lastFailedMode_)
    return false;
  IndexSetMap approaching;
  for (const auto &[key, point] : points) {
    const auto candidate = lastFailedMode_->find(key);
    if (candidate == lastFailedMode_->end()) continue;
    double best = ielm_params_.boundaryApproachRateThreshold;
    std::optional<size_t> selected;
    for (size_t index : candidate->second) {
      if (point.activeIndices().count(index)) continue;
      const double residual = point.inequalityResidual(index)(0);
      const double rate = modeStartResiduals_.at(key)(index) / residual;
      if (rate > best) {
        best = rate;
        selected = index;
      }
    }
    if (selected) approaching.emplace(key, IndexSet{*selected});
  }
  if (approaching.empty()) return false;
  current_iter_details.trials.emplace_back(current_iter_details.state, graph,
                                           approaching);
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
  if (state.error <= ielm_params_.lmParams.errorTol) return true;

  // check if mode changes
  if (!isSameMode(prev_state.points, state.points)) {
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
