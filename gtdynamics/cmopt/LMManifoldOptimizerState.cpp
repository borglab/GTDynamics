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

namespace gtdynamics {

/* ************************************************************************* */
/* <============================= LMState =================================> */
/* ************************************************************************* */

LMState::LMState(const NonlinearFactorGraph &graph,
                 const ManifoldOptimizationProblem &problem, const double &_lambda,
                 const double &_lambdaFactor, size_t _iterations)
    : manifolds(problem.manifolds()),
      unconstrainedValues(problem.unconstrainedValues()),
      constManifolds(problem.constManifolds()),
      error(evaluateGraphError(graph, manifolds, unconstrainedValues,
                               constManifolds)),
      lambda(_lambda), lambdaFactor(_lambdaFactor), iterations(_iterations) {}

/* ************************************************************************* */
LMState LMState::fromLastIteration(const LMIterationDetails &iter_details,
                                   const NonlinearFactorGraph &graph,
                                   const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  LMState state;
  if (last_trial.stepIsSuccessful) {
    state.manifolds = last_trial.nonlinearUpdate.newManifolds;
    state.unconstrainedValues =
        last_trial.nonlinearUpdate.newUnconstrainedValues;
    state.constManifolds = prev_state.constManifolds;
    state.error = last_trial.nonlinearUpdate.newError;
    state.lambda = last_trial.linearUpdate.lambda;
    state.lambdaFactor = prev_state.lambdaFactor;
    last_trial.setNextLambda(state.lambda, state.lambdaFactor, params);
  } else {
    // pick the trials with smallest error
    throw std::runtime_error("not implemented");
  }
  state.iterations = prev_state.iterations + 1;
  state.totalInnerIterations =
      prev_state.totalInnerIterations + iter_details.trials.size();
  return state;
}

/* ************************************************************************* */
double LMState::evaluateGraphError(const NonlinearFactorGraph &graph,
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
  base_values.insert(unconstrainedValues);
  base_values.insert(constManifolds.baseValues());
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
  stepIsSuccessful = false;

  // Compute linear update and linear cost change
  linearUpdate = LinearUpdate(lambda, manifold_graph, state, params);
  if (!linearUpdate.solveSuccessful) {
    return;
  }

  // Compute nonlinear update and nonlinear cost change
  nonlinearUpdate = NonlinearUpdate(state, linearUpdate, graph);

  // Decide if accept or reject trial
  modelFidelity = nonlinearUpdate.costChange / linearUpdate.costChange;
  if (linearUpdate.costChange >
          std::numeric_limits<double>::epsilon() * linearUpdate.oldError &&
      modelFidelity > params.minModelFidelity) {
    stepIsSuccessful = true;
  }

  auto end = std::chrono::high_resolution_clock::now();
  trialTime =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1e6;
}

/* ************************************************************************* */
void LMTrial::setNextLambda(double &new_lambda, double &newLambdaFactor,
                            const LevenbergMarquardtParams &params) const {
  if (stepIsSuccessful) {
    setDecreasedNextLambda(new_lambda, newLambdaFactor, params);
  } else {
    setIncreasedNextLambda(new_lambda, newLambdaFactor, params);
  }
}

/* ************************************************************************* */
void LMTrial::setIncreasedNextLambda(
    double &new_lambda, double &newLambdaFactor,
    const LevenbergMarquardtParams &params) const {
  new_lambda *= newLambdaFactor;
  if (!params.useFixedLambdaFactor) {
    newLambdaFactor *= 2.0;
  }
}

/* ************************************************************************* */
void LMTrial::setDecreasedNextLambda(
    double &new_lambda, double &newLambdaFactor,
    const LevenbergMarquardtParams &params) const {
  if (params.useFixedLambdaFactor) {
    new_lambda /= newLambdaFactor;
  } else {
    new_lambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * modelFidelity - 1.0, 3));
    newLambdaFactor *= 2.0;
  }
  new_lambda = std::max(params.lambdaLowerBound, new_lambda);
}

/* ************************************************************************* */
void LMTrial::printTitle() {
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
       << nonlinearUpdate.newError << " " << setw(12) << setprecision(4)
       << nonlinearUpdate.costChange << " " << setw(10) << setprecision(4)
       << linearUpdate.costChange << " " << setw(10) << setprecision(2)
       << linearUpdate.lambda << " " << setw(10)
       << (linearUpdate.solveSuccessful ? "T   " : "F   ") << " " << setw(10)
       << setprecision(2) << trialTime << setw(10) << setprecision(4)
       << linearUpdate.delta.norm() << endl;
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
  Values state_values = state.unconstrainedValues;
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
    solveSuccessful = true;
  } catch (const gtsam::IndeterminantLinearSystemException &) {
    solveSuccessful = false;
    return;
  }
  tangentVector = computeTangentVector(delta, state);
  oldError = linear->error(VectorValues::Zero(delta));
  newError = linear->error(delta);
  costChange = oldError - newError;
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
  damped.reserve(damped.size() + state.unconstrainedValues.size());
  std::map<Key, size_t> dims = state.manifolds.dims();
  std::map<Key, size_t> dims_unconstrained = state.unconstrainedValues.dims();
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
  if (params.getDiagonalDamping())
    return buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return buildDampedSystem(linear, state);
}

/* ************************************************************************* */
VectorValues
LMTrial::LinearUpdate::computeTangentVector(const VectorValues &delta,
                                            const LMState &state) const {
  VectorValues tangentVector;
  for (const auto &it : state.manifolds) {
    const Vector &xi = delta.at(it.first);
    tangentVector.insert(it.second.basis()->computeTangentVector(xi));
  }
  for (const auto &it : state.constManifolds) {
    tangentVector.insert(it.second.values().zeroVectors());
  }
  for (const Key &key : state.unconstrainedValues.keys()) {
    tangentVector.insert(key, delta.at(key));
  }
  return tangentVector;
}

/* ************************************************************************* */
/* <==================== IELMTrial::NonlinearUpdate =======================> */
/* ************************************************************************* */

/* ************************************************************************* */
LMTrial::NonlinearUpdate::NonlinearUpdate(const LMState &state,
                                          const LinearUpdate &linearUpdate,
                                          const NonlinearFactorGraph &graph) {

  // retract for manifolds
  VectorValues tangent_vector_manifold =
      SubValues(linearUpdate.delta, state.manifolds.keys());
  newManifolds = state.manifolds.retract(tangent_vector_manifold);

  // retract for unconstrained variables
  VectorValues tangent_vector_unconstrained =
      SubValues(linearUpdate.delta, state.unconstrainedValues.keys());
  newUnconstrainedValues =
      state.unconstrainedValues.retract(tangent_vector_unconstrained);

  // compute error
  newError = LMState::evaluateGraphError(
      graph, newManifolds, newUnconstrainedValues, state.constManifolds);
  costChange = state.error - newError;
}

/* ************************************************************************* */
/* <========================= IELMOptimizationDetails ============================> */
/* ************************************************************************* */
void LMOptimizationDetails::exportFile(const std::string &state_file_path,
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
             << "stepIsSuccessful"
             << ","
             << "linearCostChange"
             << ","
             << "nonlinearCostChange"
             << ","
             << "modelFidelity"
             << "\n";

  for (const auto &iter_details : *this) {
    const auto &state = iter_details.state;
    state_file << state.iterations << "," << state.lambda << "," << state.error
               << "\n";
    for (const auto &trial : iter_details.trials) {
      trial_file << state.iterations << "," << trial.linearUpdate.lambda << ","
                 << trial.nonlinearUpdate.newError << ","
                 << trial.stepIsSuccessful << ","
                 << trial.linearUpdate.costChange << ","
                 << trial.nonlinearUpdate.costChange << ","
                 << trial.modelFidelity << "\n";
    }
  }
  state_file.close();
  trial_file.close();
}

} // namespace gtdynamics
