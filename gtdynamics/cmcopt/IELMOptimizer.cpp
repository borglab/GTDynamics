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

  details_->clear();
  mode_.reset();
  modeStartResiduals_.clear();
  lastFailedMode_.reset();

  const LevenbergMarquardtParams &lmParams = ielm_params_.lmParams;

  // Construct manifold graph.
  std::map<Key, Key> keymap_var2manifold = varToManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph =
      ManifoldOptimizer::manifoldGraph(graph, keymap_var2manifold);

  // Construct initial state
  IELMState state(manifolds, unconstrainedValues, graph, manifold_graph,
                  lmParams.lambdaInitial, lmParams.lambdaFactor, 0);

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
    state = IELMState::fromLastIteration(iter_details, graph, manifold_graph,
                                         lmParams);
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
namespace {
IndexSetMap activeModes(const IEManifoldValues &manifolds) {
  IndexSetMap result;
  for (const auto &[key, manifold] : manifolds)
    result.emplace(key, manifold.activeIndices());
  return result;
}
}  // namespace

void IELMOptimizer::rememberMode(const IELMIterationDetails &iteration) const {
  const auto mode = activeModes(iteration.state.manifolds);
  if (!mode_ || *mode_ != mode) {
    mode_ = mode;
    lastFailedMode_.reset();
    modeStartResiduals_.clear();
    for (const auto &[key, manifold] : iteration.state.manifolds) {
      const auto &constraints = *manifold.iConstraints();
      Vector residuals(constraints.size());
      for (size_t i = 0; i < constraints.size(); ++i)
        residuals(i) = manifold.inequalityResidual(i)(0);
      modeStartResiduals_.emplace(key, std::move(residuals));
    }
  }
  // Match the historical search: last unsuccessful, successfully linearized
  // trial, even when it stayed in the same mode. Projection failures do not
  // have a candidate state and cannot supply a boundary decision.
  for (auto trial = iteration.trials.rbegin(); trial != iteration.trials.rend();
       ++trial)
    if (!trial->stepIsSuccessful && trial->linearUpdate.solveSuccessful) {
      lastFailedMode_ = activeModes(trial->nonlinearUpdate.newManifolds);
      break;
    }
}

bool IELMOptimizer::checkModeChange(
    const NonlinearFactorGraph &graph,
    IELMIterationDetails &current_iter_details) const {
  const auto &manifolds = current_iter_details.state.manifolds;
  if (!mode_ || *mode_ != activeModes(manifolds) || !lastFailedMode_)
    return false;
  IndexSetMap approaching;
  for (const auto &[key, manifold] : manifolds) {
    const auto candidate = lastFailedMode_->find(key);
    if (candidate == lastFailedMode_->end()) continue;
    double best = ielm_params_.boundaryApproachRateThreshold;
    std::optional<size_t> selected;
    for (size_t index : candidate->second) {
      if (manifold.activeIndices().count(index)) continue;
      const double residual = manifold.inequalityResidual(index)(0);
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
