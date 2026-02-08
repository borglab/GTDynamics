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

using std::cout, std::setw, std::setprecision, std::endl;
namespace gtdynamics {
using namespace gtsam;


/* ************************************************************************* */
IEGDState IEGDState::FromLastIteration(const IEGDIterDetails &iter_details,
                                       const NonlinearFactorGraph &graph,
                                       const GDParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IEGDState state;
  if (last_trial.step_is_successful) {
    state = IEGDState(last_trial.new_manifolds, graph);
  } else {
    // pick the trials with smallest error
    state = IEGDState(iter_details.state.manifolds, graph);
    for (const auto &trial : iter_details.trials) {
      if (trial.new_error < state.error) {
        state = IEGDState(trial.new_manifolds, graph);
      }
    }
  }

  last_trial.setNextLambda(state.lambda, params);
  state.iterations = prev_state.iterations + 1;
  state.totalNumberInnerIterations =
      prev_state.totalNumberInnerIterations + iter_details.trials.size();
  return state;
}

/* ************************************************************************* */
void IEGDState::computeDescentDirection(const NonlinearFactorGraph &graph) {
  std::map<Key, Key> keymap_var2manifold =
      IEOptimizer::Var2ManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph =
      ManifoldOptimizer::ManifoldGraph(graph, keymap_var2manifold);

  auto linear_graph = manifold_graph.linearize(e_manifolds);

  gradient = linear_graph->gradientAtZero();
  descent_dir = -1 * gradient;
  std::tie(blocking_indices_map, projected_descent_dir) =
      IEOptimizer::ProjectTangentCone(manifolds, descent_dir);
}

/* ************************************************************************* */
Values IEGDState::baseValues() const {
  Values base_values = manifolds.baseValues();
  // base_values.insert(unconstrainedValues());
  return base_values;
}

/* ************************************************************************* */
void IEGDTrial::setNextLambda(double &new_mu, const GDParams &params) const {
  if (forced_indices_map.size() > 0) {
    new_mu = lambda;
  } else {
    if (step_is_successful) {
      new_mu = lambda / params.beta;
    } else {
      new_mu = lambda * params.beta;
    }
  }
}

/* ************************************************************************* */
void IEGDTrial::computeDelta(const IEGDState &state) {
  delta = lambda * state.projected_descent_dir;
  tangent_vector = IEOptimizer::ComputeTangentVector(state.manifolds, delta);
  linear_cost_change = state.descent_dir.dot(delta);
}

/* ************************************************************************* */
void IEGDTrial::computeNewManifolds(const IEGDState &state) {
  new_manifolds = IEManifoldValues();
  for (const auto &[key, manifold] : state.manifolds) {
    auto manifold_tv = SubValues(tangent_vector, manifold.values().keys());
    if (state.blocking_indices_map.find(key) !=
        state.blocking_indices_map.end()) {
      const auto &blocking_indices = state.blocking_indices_map.at(key);
      new_manifolds.emplace(key, manifold.retract(manifold_tv, blocking_indices));
    } else {
      new_manifolds.emplace(key, manifold.retract(manifold_tv));
    }
  }
}

/* ************************************************************************* */
void IEGDTrial::computeNewError(const NonlinearFactorGraph &graph,
                                const IEGDState &state) {
  new_error = graph.error(new_manifolds.baseValues());
  nonlinear_cost_change = state.error - new_error;
  model_fidelity = nonlinear_cost_change / linear_cost_change;
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
  cout << setw(12) << setprecision(4) << new_error << "|";
  cout << setw(12) << setprecision(4) << nonlinear_cost_change << "|";
  cout << setw(12) << setprecision(4) << linear_cost_change << "|";
  cout << setw(10) << setprecision(2) << lambda << "|";
  cout << setw(10) << setprecision(2) << tangent_vector.norm() << endl;
}

// /* *************************************************************************
// */ IEManifoldValues IEGDOptimizer::lineSearch(
//     const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
//     const VectorValues &proj_dir, const VectorValues
//     &descent_dir, VectorValues &delta) const {
//   double alpha = 0.2;
//   double beta = 0.5;
//   double t = 1;

//   Values values = CollectManifoldValues(manifolds);
//   double eval = graph.error(values);

//   while (true) {
//     // count ++;
//     delta = t * proj_dir;
//     // delta.print("delta");

//     IEManifoldValues new_manifolds = RetractManifolds(manifolds, delta);
//     Values new_values = CollectManifoldValues(new_manifolds);
//     double new_eval = graph.error(new_values);

//     double nonlinear_error_decrease = eval - new_eval;
//     double linear_error_decrease = descent_dir.dot(delta);

//     std::cout << "t: " << t << "\tnonlinear: " << nonlinear_error_decrease
//               << "\tlinear: " << linear_error_decrease << std::endl;
//     if (linear_error_decrease < 1e-10) {
//       return manifolds;
//     }

//     if (nonlinear_error_decrease > linear_error_decrease * alpha) {
//       return new_manifolds;
//     }
//     t *= beta;
//   }
//   std::cout << "bad\n";
//   return manifolds;
// }

/* ************************************************************************* */
IEGDIterDetails IEGDOptimizer::iterate(const NonlinearFactorGraph &graph,
                                       const IEGDState &state) const {
  IEGDIterDetails iter_details(state);
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
    if (trial.step_is_successful) {
      break;
    }

    // Check condition 2.
    double abs_change_tol = std::max(params_.absoluteErrorTol,
                                     params_.relativeErrorTol * state.error);
    if (trial.linear_cost_change < abs_change_tol) {
      if (trial.nonlinear_cost_change < abs_change_tol) {
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
  trial.computeNewManifolds(state);
  trial.computeNewError(graph, state);

  // Check if successful.
  trial.step_is_successful = false;
  if (trial.nonlinear_cost_change > trial.linear_cost_change * params_.alpha) {
    trial.step_is_successful = true;
  }

  if (params_.verbose) {
    trial.print(state);
  }
}

/* ************************************************************************* */
Values IEGDOptimizer::optimizeManifolds(
    const NonlinearFactorGraph &graph, const IEManifoldValues &manifolds,
    const Values &unconstrained_values) const {
  // construct equivalent factors on e-manifolds
  // std::cout << "in optimize\n";
  std::map<Key, Key> keymap_var2manifold = Var2ManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph =
      ManifoldOptimizer::ManifoldGraph(graph, keymap_var2manifold);
  // std::cout << "graph done\n";

  // Construct initial state
  IEGDState state(manifolds, graph, 0);
  state.lambda = params_.init_lambda;

  if (params_.verbose) {
    std::cout << "Initial error: " << state.error << "\n";
    PrintIEGDTrialTitle();
  }

  // check if we're already close enough
  if (state.error <= params_.errorTol) {
    details_->emplace_back(state);
    return state.manifolds.baseValues();
  }

  // Iterative loop
  IEGDState prev_state;
  do {
    prev_state = state;
    IEGDIterDetails iter_details = iterate(graph, state);
    state = IEGDState::FromLastIteration(iter_details, graph, params_);
    details_->push_back(iter_details);
  } while (state.iterations < params_.maxIterations &&
           !checkConvergence(prev_state, state) &&
           checkMuWithinLimits(state.lambda) && std::isfinite(state.error));
  details_->emplace_back(state);
  return state.manifolds.baseValues();
}

/* ************************************************************************* */
bool IEGDOptimizer::checkMuWithinLimits(const double &lambda) const {
  return lambda > params_.muLowerBound;
}

/* ************************************************************************* */
bool IEGDOptimizer::checkModeChange(
    const NonlinearFactorGraph &graph,
    IEGDIterDetails &current_iter_details) const {
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
      change_indices_map, params_.boundary_approach_rate_threshold);

  // Condition3: approaching boundary with decent rate
  if (approach_indices_map.size() == 0) {
    return false;
  }

  // Enforce approaching indices;
  IEGDTrial trial;
  trial.lambda = current_iter_details.state.lambda;
  trial.forced_indices_map = approach_indices_map;
  trial.new_manifolds = current_iter_details.state.manifolds.moveToBoundaries(
      approach_indices_map);
  trial.new_error = graph.error(trial.new_manifolds.baseValues());
  trial.step_is_successful = true;
  current_iter_details.trials.emplace_back(trial);
  return true;
}

/* ************************************************************************* */
bool IEGDOptimizer::checkConvergence(const IEGDState &prev_state,
                                     const IEGDState &state) const {
  if (state.error <= params_.errorTol)
    return true;

  // check if mode changes
  if (!IsSameMode(prev_state.manifolds, state.manifolds)) {
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
