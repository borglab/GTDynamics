#include "imanifold/IELMOptimizer.h"
#include "utils/DynamicsSymbol.h"
#include <gtdynamics/imanifold/IELMOptimizerState.h>
#include <gtdynamics/imanifold/IEOptimizer.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <iomanip>

using std::cout, std::setw, std::setprecision, std::endl;

namespace gtsam {

/* ************************************************************************* */
/* <============================ IELMState ================================> */
/* ************************************************************************* */

IELMState::IELMState(const IEManifoldValues &_manifolds,
                     const Values &_unconstrained_values,
                     const NonlinearFactorGraph &graph, const double &_lambda,
                     const double &_lambda_factor, size_t _iterations)
    : manifolds(_manifolds), unconstrained_values(_unconstrained_values),
      error(EvaluateGraphError(graph, manifolds, _unconstrained_values)),
      lambda(_lambda), lambda_factor(_lambda_factor), iterations(_iterations) {
  ConstructEManifolds(graph);
}

/* ************************************************************************* */
IELMState IELMState::FromLastIteration(const IELMIterDetails &iter_details,
                                       const NonlinearFactorGraph &graph,
                                       const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IELMState state;
  if (last_trial.step_is_successful) {
    state =
        IELMState(last_trial.nonlinear_update.new_manifolds,
                  last_trial.nonlinear_update.new_unconstrained_values, graph,
                  last_trial.linear_update.lambda, prev_state.lambda_factor);
    // TODO: will this cause early ending? (converged in this mode, but not for
    // the overall problem)
    if (last_trial.forced_indices_map.size() > 0) {
      state.blocking_indices_map.mergeWith(last_trial.forced_indices_map);
    }
  } else {
    // pick the trials with smallest error
    state = IELMState(prev_state.manifolds, prev_state.unconstrained_values,
                      graph, prev_state.lambda, prev_state.lambda_factor);
    for (const auto &trial : iter_details.trials) {
      if (trial.linear_update.solve_successful &&
          trial.nonlinear_update.new_error < prev_state.error) {
        state =
            IELMState(trial.nonlinear_update.new_manifolds,
                      trial.nonlinear_update.new_unconstrained_values, graph,
                      trial.linear_update.lambda, prev_state.lambda_factor);
      }
    }
  }

  last_trial.setNextLambda(state.lambda, state.lambda_factor, params);
  state.iterations = prev_state.iterations + 1;
  state.totalNumberInnerIterations =
      prev_state.totalNumberInnerIterations + iter_details.trials.size();
  return state;
}

/* ************************************************************************* */
void IELMState::identifyGradBlockingIndices(
    const NonlinearFactorGraph &manifold_graph) {
  Values values = IEOptimizer::EManifolds(manifolds);
  values.insert(unconstrained_values);
  auto linear_graph = manifold_graph.linearize(values);

  gradient = linear_graph->gradientAtZero();
  VectorValues descent_dir = -1 * gradient;

  // identify blocking constraints
  blocking_indices_map =
      IEOptimizer::ProjectTangentCone(manifolds, descent_dir).first;
}

/* ************************************************************************* */
void IELMState::ConstructEManifolds(const NonlinearFactorGraph &graph) {
  std::map<Key, Key> keymap_var2manifold =
      IEOptimizer::Var2ManifoldKeyMap(manifolds);
  NonlinearFactorGraph manifold_graph =
      ManifoldOptimizer::ManifoldGraph(graph, keymap_var2manifold);
  identifyGradBlockingIndices(manifold_graph);
  // setting blocking constraints as equalities, create e-manifolds
  std::tie(e_manifolds, const_e_manifolds) =
      IEOptimizer::EManifolds(manifolds, blocking_indices_map);
}

/* ************************************************************************* */
double IELMState::EvaluateGraphError(const NonlinearFactorGraph &graph,
                                     const IEManifoldValues &_manifolds,
                                     const Values &_unconstrained_values) {
  Values all_values = _manifolds.baseValues();
  all_values.insert(_unconstrained_values);
  return graph.error(all_values);
}

/* ************************************************************************* */
Values IELMState::baseValues() const {
  Values base_values = manifolds.baseValues();
  base_values.insert(unconstrained_values);
  return base_values;
}

/* ************************************************************************* */
VectorValues
IELMState::computeMetricSigmas(const NonlinearFactorGraph &graph) const {
  Values base_values = baseValues();
  // PrintKeyVector(base_values.keys(), "base_values",
  //                gtdynamics::GTDKeyFormatter);
  // PrintKeySet(graph.keys(), "graph_keys", gtdynamics::GTDKeyFormatter);
  auto linear_graph = graph.linearize(base_values);
  auto hessian_diag = linear_graph->hessianDiagonal();
  VectorValues metric_sigmas;
  for (auto &[key, value] : hessian_diag) {
    Vector sigmas_sqr_inv = hessian_diag.at(key);
    Vector sigmas = sigmas_sqr_inv;
    for (int i = 0; i < sigmas.size(); i++) {
      sigmas(i) = 1 / sqrt(sigmas(i));
    }
    metric_sigmas.insert(key, sigmas);
  }
  metric_sigmas = 10 * metric_sigmas;
  // metric_sigmas.print("metric sigmas:", gtdynamics::GTDKeyFormatter);

  return metric_sigmas;
}

/* ************************************************************************* */
/* <============================ IELMTrial ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMTrial::IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
                     const double &lambda,
                     const LevenbergMarquardtParams &params) {
  // std::cout << "========= " << state.iterations << " ======= \n";
  auto start = std::chrono::high_resolution_clock::now();
  step_is_successful = false;

  // Compute linear update and linear cost change
  linear_update = LinearUpdate(lambda, graph, state, params);
  if (!linear_update.solve_successful) {
    return;
  }

  // Compute nonlinear update and nonlinear cost change
  nonlinear_update = NonlinearUpdate(state, linear_update, graph);

  // Decide if accept or reject trial
  model_fidelity = nonlinear_update.cost_change / linear_update.cost_change;
  if (linear_update.cost_change >
          std::numeric_limits<double>::epsilon() * linear_update.old_error &&
      model_fidelity > params.minModelFidelity) {
    step_is_successful = true;
  }

  auto end = std::chrono::high_resolution_clock::now();
  trial_time =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1e6;
}

/* ************************************************************************* */
IELMTrial::IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
                     const IndexSetMap &approach_indices_map) {
  linear_update.lambda = state.lambda;
  forced_indices_map = approach_indices_map;
  nonlinear_update.new_manifolds =
      state.manifolds.moveToBoundaries(approach_indices_map);
  nonlinear_update.new_unconstrained_values = state.unconstrained_values;
  nonlinear_update.computeError(graph, state.error);
  step_is_successful = true;
}

/* ************************************************************************* */
void IELMTrial::setNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const {
  if (forced_indices_map.size() > 0) {
    return;
  }
  if (step_is_successful) {
    setDecreasedNextLambda(new_lambda, new_lambda_factor, params);
  } else {
    setIncreasedNextLambda(new_lambda, new_lambda_factor, params);
  }
}

/* ************************************************************************* */
void IELMTrial::setIncreasedNextLambda(
    double &new_lambda, double &new_lambda_factor,
    const LevenbergMarquardtParams &params) const {
  new_lambda *= new_lambda_factor;
  if (!params.useFixedLambdaFactor) {
    new_lambda_factor *= 2.0;
  }
}

/* ************************************************************************* */
void IELMTrial::setDecreasedNextLambda(
    double &new_lambda, double &new_lambda_factor,
    const LevenbergMarquardtParams &params) const {
  if (params.useFixedLambdaFactor) {
    new_lambda /= new_lambda_factor;
  } else {
    new_lambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * model_fidelity - 1.0, 3));
    new_lambda_factor *= 2.0;
  }
  new_lambda = std::max(params.lambdaLowerBound, new_lambda);
}

/* ************************************************************************* */
void IELMTrial::PrintTitle() {
  cout << setw(10) << "iter"
       << " " << setw(12) << "error "
       << " " << setw(12) << "nonlinear "
       << " " << setw(12) << "linear "
       << " " << setw(10) << "lambda "
       << " " << setw(10) << "solve_succ "
       << " " << setw(10) << "time   "
       << " " << setw(10) << "delta_norm" << endl;
}

/* ************************************************************************* */
void IELMTrial::print(const IELMState &state) const {
  cout << setw(10) << state.iterations << " ";
  cout << setw(12) << setprecision(4) << nonlinear_update.new_error << " "
       << setw(12) << setprecision(4) << nonlinear_update.cost_change << " "
       << setw(10) << setprecision(4) << linear_update.cost_change << " "
       << setw(10) << setprecision(2) << linear_update.lambda << " ";
  cout << setw(10) << (linear_update.solve_successful ? "T   " : "F   ") << " "
       << setw(10) << setprecision(2) << trial_time << " ";
  cout << setw(10) << setprecision(4) << linear_update.delta.norm() << " ";
  cout << setw(10) << setprecision(4) << linear_update.tangent_vector.norm()
       << " ";
  cout << setw(10) << setprecision(4)
       << linear_update.tangent_vector.norm() / linear_update.delta.norm()
       << " ";
  cout << endl;
}

/* ************************************************************************* */
/* <===================== IELMTrial::LinearUpdate =========================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMTrial::LinearUpdate::LinearUpdate(const double &_lambda,
                                      const NonlinearFactorGraph &graph,
                                      const IELMState &state,
                                      const LevenbergMarquardtParams &params)
    : lambda(_lambda) {
  // initialize blocking constraints and e_manifolds
  std::map<Key, Key> keymap_var2manifold =
      IEOptimizer::Var2ManifoldKeyMap(state.manifolds);
  blocking_indices_map = state.blocking_indices_map;
  e_manifolds = state.e_manifolds;
  const_e_manifolds = state.const_e_manifolds;

  while (true) {
    // linearize and build damped system
    GaussianFactorGraph::shared_ptr linear =
        linearize(graph, state.unconstrained_values, keymap_var2manifold);
    VectorValues sqrt_hessian_diagonal = SqrtHessianDiagonal(*linear, params);
    auto damped_system =
        buildDampedSystem(*linear, sqrt_hessian_diagonal, state, params);

    // solve delta
    // VectorValues delta;
    try {
      delta = SolveLinear(damped_system, params);
      solve_successful = true;
    } catch (const IndeterminantLinearSystemException &) {
      solve_successful = false;
      return;
    }

    // check if satisfy tagent cone, if not, add constraints and recompute
    bool feasible = true;
    for (const Key &key : e_manifolds.keys()) {
      const Vector &xi = delta.at(key);
      ConstraintManifold e_manifold = e_manifolds.at<ConstraintManifold>(key);
      VectorValues tv = e_manifold.basis()->computeTangentVector(xi);

      IndexSet blocking_indices = state.manifolds.at(key).blockingIndices(tv);
      if (blocking_indices.size() > 0) {
        feasible = false;
        blocking_indices_map.addIndices(key, blocking_indices);
      }
    }

    if (feasible) {
      tangent_vector =
          computeTangentVector(delta, state.unconstrained_values.keys());
      old_error = linear->error(VectorValues::Zero(delta));
      new_error = linear->error(delta);
      cost_change = old_error - new_error;
      break;
    } else {
      // recompute e_manifolds
      std::tie(e_manifolds, const_e_manifolds) =
          IEOptimizer::EManifolds(state.manifolds, blocking_indices_map);
    }
  }
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr IELMTrial::LinearUpdate::linearize(
    const NonlinearFactorGraph &graph, const Values &unconstrained_values,
    const std::map<Key, Key> &keymap_var2manifold) const {
  // linearize on e-manifolds
  NonlinearFactorGraph manifold_graph = ManifoldOptimizer::ManifoldGraph(
      graph, keymap_var2manifold, const_e_manifolds);
  Values values = e_manifolds;
  values.insert(unconstrained_values);
  auto active_linear_graph = manifold_graph.linearize(values);
  return active_linear_graph;
}

/* ************************************************************************* */
LMCachedModel *IELMTrial::LinearUpdate::getCachedModel(size_t dim) const {
  if (dim >= noiseModelCache.size())
    noiseModelCache.resize(dim + 1);
  LMCachedModel *item = &noiseModelCache[dim];
  if (!item->model)
    *item = LMCachedModel(dim, 1.0 / std::sqrt(lambda));
  return item;
}

/* ************************************************************************* */
GaussianFactorGraph
IELMTrial::LinearUpdate::buildDampedSystem(GaussianFactorGraph damped,
                                           const IELMState &state) const {
  noiseModelCache.resize(0);
  // for each of the variables, add a prior
  damped.reserve(damped.size() + e_manifolds.size() +
                 state.unconstrained_values.size());
  std::map<Key, size_t> dims = e_manifolds.dims();
  std::map<Key, size_t> dims_unconstrained = state.unconstrained_values.dims();
  dims.insert(dims_unconstrained.begin(), dims_unconstrained.end());
  for (const auto &key_dim : dims) {
    const Key &key = key_dim.first;
    const size_t &dim = key_dim.second;
    const LMCachedModel *item = getCachedModel(dim);
    damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
  }
  return damped;
}

/* ************************************************************************* */
GaussianFactorGraph IELMTrial::LinearUpdate::buildDampedSystem(
    GaussianFactorGraph damped, // gets copied
    const VectorValues &sqrtHessianDiagonal) const {
  noiseModelCache.resize(0);
  damped.reserve(damped.size() + sqrtHessianDiagonal.size());
  for (const auto &key_vector : sqrtHessianDiagonal) {
    try {
      const Key key = key_vector.first;
      const size_t dim = key_vector.second.size();
      LMCachedModel *item = getCachedModel(dim);
      item->A.diagonal() = sqrtHessianDiagonal.at(key); // use diag(hessian)
      damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
    } catch (const std::out_of_range &) {
      continue; // Don't attempt any damping if no key found in diagonal
    }
  }
  return damped;
}

/* ************************************************************************* */
GaussianFactorGraph IELMTrial::LinearUpdate::buildDampedSystem(
    const GaussianFactorGraph &linear, const VectorValues &sqrtHessianDiagonal,
    const IELMState &state, const LevenbergMarquardtParams &params) const {

  if (params.diagonalDamping)
    return buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return buildDampedSystem(linear, state);
}

/* ************************************************************************* */
VectorValues IELMTrial::LinearUpdate::computeTangentVector(
    const VectorValues &delta, const KeyVector &unconstrained_keys) const {
  VectorValues tangent_vector;
  for (const Key &key : e_manifolds.keys()) {
    const Vector &xi = delta.at(key);
    ConstraintManifold e_manifold = e_manifolds.at<ConstraintManifold>(key);
    VectorValues tv = e_manifold.basis()->computeTangentVector(xi);
    tangent_vector.insert(tv);
  }
  for (const Key &key : const_e_manifolds.keys()) {
    ConstraintManifold e_manifold =
        const_e_manifolds.at<ConstraintManifold>(key);
    VectorValues tv = e_manifold.values().zeroVectors();
    tangent_vector.insert(tv);
  }
  for (const Key &key : unconstrained_keys) {
    tangent_vector.insert(key, delta.at(key));
  }
  return tangent_vector;
}

/* ************************************************************************* */
/* <==================== IELMTrial::NonlinearUpdate =======================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMTrial::NonlinearUpdate::NonlinearUpdate(const IELMState &state,
                                            const LinearUpdate &linear_update,
                                            const NonlinearFactorGraph &graph) {

  // copy const manifolds
  for (const Key &key : linear_update.const_e_manifolds.keys()) {
    new_manifolds.emplace(key, state.manifolds.at(key));
  }

  // retract for ie-manifolds
  for (const Key &key : linear_update.e_manifolds.keys()) {
    const auto &manifold = state.manifolds.at(key);
    VectorValues tv =
        SubValues(linear_update.tangent_vector, manifold.values().keys());
    const auto &blocking_indices_map = linear_update.blocking_indices_map;
    if (blocking_indices_map.find(key) != blocking_indices_map.end()) {
      const auto &blocking_indices = blocking_indices_map.at(key);
      new_manifolds.emplace(key, manifold.retract(tv, blocking_indices));
    } else {
      new_manifolds.emplace(key, manifold.retract(tv));
    }
  }

  // retract for unconstrained variables
  VectorValues tangent_vector_unconstrained = SubValues(
      linear_update.tangent_vector, state.unconstrained_values.keys());
  new_unconstrained_values =
      state.unconstrained_values.retract(tangent_vector_unconstrained);

  // compute error
  computeError(graph, state.error);
}

void IELMTrial::NonlinearUpdate::computeError(const NonlinearFactorGraph &graph,
                                              const double &old_error) {

  new_error = IELMState::EvaluateGraphError(graph, new_manifolds,
                                            new_unconstrained_values);
  cost_change = old_error - new_error;
}

/* ************************************************************************* */
/* <========================= IELMItersDetails ============================> */
/* ************************************************************************* */
void IELMItersDetails::exportFile(const std::string &state_file_path,
                                  const std::string &trial_file_path) const {
  std::ofstream state_file, trial_file;
  state_file.open(state_file_path);
  trial_file.open(trial_file_path);

  state_file << "iterations"
             << ","
             << "lambda"
             << ","
             << "error"
             << "\n";
  trial_file << "iterations"
             << ","
             << "lambda"
             << ","
             << "error"
             << ","
             << "step_is_successful"
             << ","
             << "linear_cost_change"
             << ","
             << "nonlinear_cost_change"
             << ","
             << "model_fidelity"
             << ","
             << "tangent_vector_norm"
             << "\n";

  for (const auto &iter_details : *this) {
    const auto &state = iter_details.state;
    state_file << state.iterations << "," << state.lambda << "," << state.error
               << "\n";
    for (const auto &trial : iter_details.trials) {
      trial_file << state.iterations << "," << trial.linear_update.lambda << ","
                 << trial.nonlinear_update.new_error << ","
                 << trial.step_is_successful << ","
                 << trial.linear_update.cost_change << ","
                 << trial.nonlinear_update.cost_change << ","
                 << trial.model_fidelity << ","
                 << trial.linear_update.tangent_vector.norm() << "\n";
    }
  }
  state_file.close();
  trial_file.close();
}

} // namespace gtsam
