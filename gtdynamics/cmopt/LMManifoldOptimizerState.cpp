#include <gtdynamics/cmopt/LMManifoldOptimizerState.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <iomanip>
#include <stdexcept>

using std::cout, std::setw, std::setprecision, std::endl;

namespace gtsam {

/* ************************************************************************* */
/* <============================= LMState =================================> */
/* ************************************************************************* */

LMState::LMState(const NonlinearFactorGraph &graph,
                 const ManifoldOptProblem &problem, const double &_lambda,
                 const double &_lambda_factor, size_t _iterations)
    : manifolds(problem.manifolds()),
      unconstrained_values(problem.unconstrainedValues()),
      const_manifolds(problem.constManifolds()),
      error(EvaluateGraphError(graph, manifolds, unconstrained_values,
                               const_manifolds)),
      lambda(_lambda), lambda_factor(_lambda_factor), iterations(_iterations) {}

/* ************************************************************************* */
LMState LMState::FromLastIteration(const LMIterDetails &iter_details,
                                   const NonlinearFactorGraph &graph,
                                   const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  LMState state;
  if (last_trial.step_is_successful) {
    state.manifolds = last_trial.nonlinear_update.new_manifolds;
    state.unconstrained_values =
        last_trial.nonlinear_update.new_unconstrained_values;
    state.const_manifolds = prev_state.const_manifolds;
    state.error = last_trial.nonlinear_update.new_error;
    state.lambda = last_trial.linear_update.lambda;
    state.lambda_factor = prev_state.lambda_factor;
    last_trial.setNextLambda(state.lambda, state.lambda_factor, params);
  } else {
    // pick the trials with smallest error
    throw std::runtime_error("not implemented");
  }
  state.iterations = prev_state.iterations + 1;
  state.totalNumberInnerIterations =
      prev_state.totalNumberInnerIterations + iter_details.trials.size();
  return state;
}

/* ************************************************************************* */
double LMState::EvaluateGraphError(const NonlinearFactorGraph &graph,
                                   const EManifoldValues &_manifolds,
                                   const Values &_unconstrained_values,
                                   const EManifoldValues &_const_manifolds) {
  Values base_values = _manifolds.baseValues();
  base_values.insert(_unconstrained_values);
  base_values.insert(_const_manifolds.baseValues());
  return graph.error(base_values);
}

/* ************************************************************************* */
Values LMState::baseValues() const {
  Values base_values = manifolds.baseValues();
  base_values.insert(unconstrained_values);
  base_values.insert(const_manifolds.baseValues());
  return base_values;
}

/* ************************************************************************* */
/* <============================= LMTrial =================================> */
/* ************************************************************************* */

/* ************************************************************************* */
LMTrial::LMTrial(const LMState &state, const NonlinearFactorGraph &graph,
                 const NonlinearFactorGraph &manifold_graph,
                 const double &lambda, const LevenbergMarquardtParams &params) {
  // std::cout << "========= " << state.iterations << " ======= \n";
  auto start = std::chrono::high_resolution_clock::now();
  step_is_successful = false;

  // Compute linear update and linear cost change
  linear_update = LinearUpdate(lambda, manifold_graph, state, params);
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
void LMTrial::setNextLambda(double &new_lambda, double &new_lambda_factor,
                            const LevenbergMarquardtParams &params) const {
  if (step_is_successful) {
    setDecreasedNextLambda(new_lambda, new_lambda_factor, params);
  } else {
    setIncreasedNextLambda(new_lambda, new_lambda_factor, params);
  }
}

/* ************************************************************************* */
void LMTrial::setIncreasedNextLambda(
    double &new_lambda, double &new_lambda_factor,
    const LevenbergMarquardtParams &params) const {
  new_lambda *= new_lambda_factor;
  if (!params.useFixedLambdaFactor) {
    new_lambda_factor *= 2.0;
  }
}

/* ************************************************************************* */
void LMTrial::setDecreasedNextLambda(
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
void LMTrial::PrintTitle() {
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
void LMTrial::print(const LMState &state) const {
  cout << setw(10) << state.iterations << " " << setw(12) << setprecision(4)
       << nonlinear_update.new_error << " " << setw(12) << setprecision(4)
       << nonlinear_update.cost_change << " " << setw(10) << setprecision(4)
       << linear_update.cost_change << " " << setw(10) << setprecision(2)
       << linear_update.lambda << " " << setw(10)
       << (linear_update.solve_successful ? "T   " : "F   ") << " " << setw(10)
       << setprecision(2) << trial_time << setw(10) << setprecision(4)
       << linear_update.delta.norm() << endl;
}

/* ************************************************************************* */
/* <===================== IELMTrial::LinearUpdate =========================> */
/* ************************************************************************* */

/* ************************************************************************* */
LMTrial::LinearUpdate::LinearUpdate(const double &_lambda,
                                    const NonlinearFactorGraph &manifold_graph,
                                    const LMState &state,
                                    const LevenbergMarquardtParams &params)
    : lambda(_lambda) {
  // linearize and build damped system
  Values state_values = state.unconstrained_values;
  for (const auto &it : state.manifolds) {
    state_values.insert(it.first, it.second);
  }
  GaussianFactorGraph::shared_ptr linear =
      manifold_graph.linearize(state_values);
  VectorValues sqrt_hessian_diagonal = SqrtHessianDiagonal(*linear, params);
  auto damped_system =
      buildDampedSystem(*linear, sqrt_hessian_diagonal, state, params);

  // solve delta
  try {
    delta = SolveLinear(damped_system, params);
    solve_successful = true;
  } catch (const IndeterminantLinearSystemException &) {
    solve_successful = false;
    return;
  }
  tangent_vector = computeTangentVector(delta, state);
  old_error = linear->error(VectorValues::Zero(delta));
  new_error = linear->error(delta);
  cost_change = old_error - new_error;
}

/* ************************************************************************* */
LMCachedModel *LMTrial::LinearUpdate::getCachedModel(size_t dim) const {
  if (dim >= noiseModelCache.size())
    noiseModelCache.resize(dim + 1);
  LMCachedModel *item = &noiseModelCache[dim];
  if (!item->model)
    *item = LMCachedModel(dim, 1.0 / std::sqrt(lambda));
  return item;
}

/* ************************************************************************* */
GaussianFactorGraph
LMTrial::LinearUpdate::buildDampedSystem(GaussianFactorGraph damped,
                                         const LMState &state) const {
  noiseModelCache.resize(0);
  // for each of the variables, add a prior
  damped.reserve(damped.size() + state.unconstrained_values.size());
  std::map<Key, size_t> dims = state.manifolds.dims();
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
GaussianFactorGraph LMTrial::LinearUpdate::buildDampedSystem(
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
GaussianFactorGraph LMTrial::LinearUpdate::buildDampedSystem(
    const GaussianFactorGraph &linear, const VectorValues &sqrtHessianDiagonal,
    const LMState &state, const LevenbergMarquardtParams &params) const {
  if (params.diagonalDamping)
    return buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return buildDampedSystem(linear, state);
}

/* ************************************************************************* */
VectorValues
LMTrial::LinearUpdate::computeTangentVector(const VectorValues &delta,
                                            const LMState &state) const {
  VectorValues tangent_vector;
  for (const auto &it : state.manifolds) {
    const Vector &xi = delta.at(it.first);
    tangent_vector.insert(it.second.basis()->computeTangentVector(xi));
  }
  for (const auto &it : state.const_manifolds) {
    tangent_vector.insert(it.second.values().zeroVectors());
  }
  for (const Key &key : state.unconstrained_values.keys()) {
    tangent_vector.insert(key, delta.at(key));
  }
  return tangent_vector;
}

/* ************************************************************************* */
/* <==================== IELMTrial::NonlinearUpdate =======================> */
/* ************************************************************************* */

/* ************************************************************************* */
LMTrial::NonlinearUpdate::NonlinearUpdate(const LMState &state,
                                          const LinearUpdate &linear_update,
                                          const NonlinearFactorGraph &graph) {

  // retract for manifolds
  VectorValues tangent_vector_manifold =
      SubValues(linear_update.delta, state.manifolds.keys());
  new_manifolds = state.manifolds.retract(tangent_vector_manifold);

  // retract for unconstrained variables
  VectorValues tangent_vector_unconstrained =
      SubValues(linear_update.delta, state.unconstrained_values.keys());
  new_unconstrained_values =
      state.unconstrained_values.retract(tangent_vector_unconstrained);

  // compute error
  new_error = LMState::EvaluateGraphError(
      graph, new_manifolds, new_unconstrained_values, state.const_manifolds);
  cost_change = state.error - new_error;
}

/* ************************************************************************* */
/* <========================= IELMItersDetails ============================> */
/* ************************************************************************* */
void LMItersDetails::exportFile(const std::string &state_file_path,
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
                 << trial.model_fidelity << "\n";
    }
  }
  state_file.close();
  trial_file.close();
}

} // namespace gtsam
