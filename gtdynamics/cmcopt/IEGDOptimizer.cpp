/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEManifoldOptimizer.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/cmcopt/IEGDOptimizer.h>

#include <iomanip>

using std::cout, std::setw, std::setprecision, std::endl;
namespace gtdynamics {
using namespace gtsam;


/* ************************************************************************* */
IEGDState IEGDState::fromLastIteration(const IEGDIterationDetails &iter_details,
                                       const NonlinearFactorGraph &graph,
                                       const GradientDescentParams &params) {
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IEGDState state;
  if (last_trial.stepIsSuccessful) {
    state = IEGDState(last_trial.newPoints, graph, 0,
                      last_trial.newUnconstrainedValues);
  } else {
    // pick the trials with smallest error
    state = IEGDState(iter_details.state.points, graph, 0,
                      iter_details.state.unconstrainedValues);
    for (const auto &trial : iter_details.trials) {
      if (trial.newError < state.error) {
        state = IEGDState(trial.newPoints, graph, 0, trial.newUnconstrainedValues);
      }
    }
  }

  last_trial.setNextLambda(state.lambda, params);
  state.iterations = prev_state.iterations + 1;
  state.totalInnerIterations =
      prev_state.totalInnerIterations + iter_details.trials.size();
  return state;
}

/* ************************************************************************* */
void IEGDState::computeDescentDirection(const NonlinearFactorGraph &graph) {
  NonlinearFactorGraph pointGraph =
      IEOptimizer::pointGraph(graph, points);

  auto linear_graph = pointGraph.linearize(pointValues());

  gradient = linear_graph->gradientAtZero();
  for (const auto &[key, point] : points)
    if (!gradient.exists(key)) gradient.insert(key, Vector::Zero(point.dim()));
  for (Key key : unconstrainedValues.keys())
    if (!gradient.exists(key))
      gradient.insert(key, Vector::Zero(unconstrainedValues.at(key).dim()));
  descentDirection = -1 * gradient;
  std::tie(blockingIndicesMap, projectedDescentDirection) =
      IEOptimizer::projectTangentCone(points, descentDirection);
  for (Key key : unconstrainedValues.keys())
    projectedDescentDirection.insert(key, descentDirection.at(key));
}

/* ************************************************************************* */
Values IEGDState::baseValues() const {
  Values base_values = ambientValues(points);
  base_values.insert(unconstrainedValues);
  return base_values;
}

/* ************************************************************************* */
void IEGDTrial::setNextLambda(double &new_mu, const GradientDescentParams &params) const {
  if (forcedIndicesMap.size() > 0) {
    new_mu = lambda;
  } else {
    if (stepIsSuccessful) {
      new_mu = lambda / params.beta;
    } else {
      new_mu = lambda * params.beta;
    }
  }
}

/* ************************************************************************* */
void IEGDTrial::computeDelta(const IEGDState &state) {
  delta = lambda * state.projectedDescentDirection;
  tangentVector = IEOptimizer::computeTangentVector(state.points, delta);
  tangentVector.insert(SubValues(delta, state.unconstrainedValues.keys()));
  linearCostChange = state.descentDirection.dot(delta);
}

/* ************************************************************************* */
void IEGDTrial::computeNewPoints(const IEGDState &state) {
  newUnconstrainedValues = state.unconstrainedValues.retract(
      SubValues(delta, state.unconstrainedValues.keys()));
  newPoints = IECMPoints();
  for (const auto &[key, point] : state.points) {
    auto pointTangent = SubValues(tangentVector, point.values().keys());
    if (state.blockingIndicesMap.find(key) !=
        state.blockingIndicesMap.end()) {
      const auto &blocking_indices = state.blockingIndicesMap.at(key);
      newPoints.emplace(key, point.retract(pointTangent, blocking_indices));
    } else {
      newPoints.emplace(key, point.retract(pointTangent));
    }
  }
}

/* ************************************************************************* */
void IEGDTrial::computeNewError(const NonlinearFactorGraph &graph,
                                const IEGDState &state) {
  Values values = ambientValues(newPoints);
  values.insert(newUnconstrainedValues);
  newError = graph.error(values);
  nonlinearCostChange = state.error - newError;
  modelFidelity = nonlinearCostChange / linearCostChange;
}

/* ************************************************************************* */
void PrintIEGDTrialTitle() {
  cout << setw(10) << "iter   "
       << "|" << setw(12) << "error  "
       << "|" << setw(12) << "nonlinear "
       << "|" << setw(12) << "linear "
       << "|" << setw(10) << "lambda  "
       << "|" << setw(10) << "norm  " << endl;
}

/* ************************************************************************* */
void IEGDTrial::print(const IEGDState &state) const {
  cout << setw(10) << state.iterations << "|";
  cout << setw(12) << setprecision(4) << newError << "|";
  cout << setw(12) << setprecision(4) << nonlinearCostChange << "|";
  cout << setw(12) << setprecision(4) << linearCostChange << "|";
  cout << setw(10) << setprecision(2) << lambda << "|";
  cout << setw(10) << setprecision(2) << tangentVector.norm() << endl;
}

// /* *************************************************************************
// */ IECMPoints IEGDOptimizer::lineSearch(
//     const NonlinearFactorGraph &graph, const IECMPoints &points,
//     const VectorValues &proj_dir, const VectorValues
//     &descentDirection, VectorValues &delta) const {
//   double alpha = 0.2;
//   double beta = 0.5;
//   double t = 1;

//   Values values = CollectManifoldValues(points);
//   double eval = graph.error(values);

//   while (true) {
//     // count ++;
//     delta = t * proj_dir;
//     // delta.print("delta");

//     IECMPoints newPoints = retractPoints(points, delta);
//     Values new_values = CollectManifoldValues(newPoints);
//     double new_eval = graph.error(new_values);

//     double nonlinear_error_decrease = eval - new_eval;
//     double linear_error_decrease = descentDirection.dot(delta);

//     std::cout << "t: " << t << "\tnonlinear: " << nonlinear_error_decrease
//               << "\tlinear: " << linear_error_decrease << std::endl;
//     if (linear_error_decrease < 1e-10) {
//       return points;
//     }

//     if (nonlinear_error_decrease > linear_error_decrease * alpha) {
//       return newPoints;
//     }
//     t *= beta;
//   }
//   std::cout << "bad\n";
//   return points;
// }

/* ************************************************************************* */
IEGDIterationDetails IEGDOptimizer::iterate(const NonlinearFactorGraph &graph,
                                       const IEGDState &state) const {
  IEGDIterationDetails iter_details(state);
  if (checkModeChange(graph, iter_details)) {
    return iter_details;
  }

  // Set lambda for first trial.
  IEGDTrial trial;
  trial.lambda = state.lambda;

  // Perform trials until any of follwing conditions is met
  // * 1) trial is successful
  // * 2) update is too small
  // * 3) lambda goes beyond limits
  while (true) {
    // Perform the trial.
    tryLambda(graph, state, trial);
    iter_details.trials.emplace_back(trial);

    // Check condition 1.
    if (trial.stepIsSuccessful) {
      break;
    }

    // Check condition 2.
    double abs_change_tol = std::max(params_.absoluteErrorTol,
                                     params_.relativeErrorTol * state.error);
    if (trial.linearCostChange < abs_change_tol) {
      if (trial.nonlinearCostChange < abs_change_tol) {
        break;
      }
    }

    // Set lambda for next trial.
    IEGDTrial next_trial;
    trial.setNextLambda(next_trial.lambda, params_);
    trial = next_trial;

    // Check condition 3.
    if (!checkMuWithinLimits(trial.lambda)) {
      break;
    }
  }
  return iter_details;
}

/* ************************************************************************* */
void IEGDOptimizer::tryLambda(const NonlinearFactorGraph &graph,
                              const IEGDState &state, IEGDTrial &trial) const {
  // Perform the trial.
  trial.computeDelta(state);
  trial.computeNewPoints(state);
  trial.computeNewError(graph, state);

  // Check if successful.
  trial.stepIsSuccessful = false;
  if (trial.nonlinearCostChange > trial.linearCostChange * params_.alpha) {
    trial.stepIsSuccessful = true;
  }

  if (params_.verbose) {
    trial.print(state);
  }
}

/* ************************************************************************* */
Values IEGDOptimizer::optimize(
    const NonlinearFactorGraph &graph, const IECMPoints &points,
    const Values &unconstrainedValues) const {
  validatePoints(points, unconstrainedValues);
  details_->clear();
  // Discovery and explicit construction use the same point/free-variable state.
  IEGDState state(points, graph, 0, unconstrainedValues);
  state.lambda = params_.initialLambda;

  if (params_.verbose) {
    std::cout << "Initial error: " << state.error << "\n";
    PrintIEGDTrialTitle();
  }

  // check if we're already close enough
  if (state.error <= params_.errorTol) {
    details_->emplace_back(state);
    return state.baseValues();
  }

  // Iterative loop
  IEGDState prev_state;
  do {
    // Copy-construct points before moving the snapshot: map copy assignment
    // may recycle nodes across components with different immutable bindings.
    prev_state = IEGDState(state);
    IEGDIterationDetails iter_details = iterate(graph, state);
    state = IEGDState::fromLastIteration(iter_details, graph, params_);
    details_->push_back(iter_details);
  } while (state.iterations < params_.maxIterations &&
           !checkConvergence(prev_state, state) &&
           checkMuWithinLimits(state.lambda) && std::isfinite(state.error));
  details_->emplace_back(state);
  return state.baseValues();
}

/* ************************************************************************* */
bool IEGDOptimizer::checkMuWithinLimits(const double &lambda) const {
  return lambda > params_.muLowerBound;
}

/* ************************************************************************* */
bool IEGDOptimizer::checkModeChange(
    const NonlinearFactorGraph &graph,
    IEGDIterationDetails &current_iter_details) const {
  if (details_->size() == 0) {
    return false;
  }

  // Find the first state in the sequence of states that have the same mode.
  int n = details_->size();
  int first_i = n - 1;
  while (first_i >= 0 && isSameMode(current_iter_details.state.points,
                                    details_->at(first_i).state.points)) {
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
    if (!details_->at(iter_idx).trials.at(trial_idx).stepIsSuccessful) {
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

  IndexSetMap change_indices_map = identifyChangeIndices(
      current_iter_details.state.points,
      details_->at(iter_idx).trials.at(trial_idx).newPoints);

  // Condition2(2): most recent failed trial results in other mode
  if (change_indices_map.size() == 0) {
    return false;
  }

  auto approach_indices_map = identifyApproachingIndices(
      init_iter_dertails.state.points, current_iter_details.state.points,
      change_indices_map, params_.boundaryApproachRateThreshold);

  // Condition3: approaching boundary with decent rate
  if (approach_indices_map.size() == 0) {
    return false;
  }

  // Enforce approaching indices;
  IEGDTrial trial;
  trial.lambda = current_iter_details.state.lambda;
  trial.forcedIndicesMap = approach_indices_map;
  trial.newPoints = movePointsToBoundaries(current_iter_details.state.points,
      approach_indices_map);
  trial.newUnconstrainedValues = current_iter_details.state.unconstrainedValues;
  Values values = ambientValues(trial.newPoints);
  values.insert(trial.newUnconstrainedValues);
  trial.newError = graph.error(values);
  trial.stepIsSuccessful = true;
  current_iter_details.trials.emplace_back(trial);
  return true;
}

/* ************************************************************************* */
bool IEGDOptimizer::checkConvergence(const IEGDState &prev_state,
                                     const IEGDState &state) const {
  if (state.error <= params_.errorTol)
    return true;

  // check if mode changes
  if (!isSameMode(prev_state.points, state.points)) {
    return false;
  }

  // check if diverges
  double absoluteDecrease = prev_state.error - state.error;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / prev_state.error;
  bool converged = (params_.relativeErrorTol &&
                    (relativeDecrease <= params_.relativeErrorTol)) ||
                   (absoluteDecrease <= params_.absoluteErrorTol);
  return converged;
}

} // namespace gtdynamics
