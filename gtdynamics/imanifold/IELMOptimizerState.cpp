#include <gtdynamics/imanifold/IELMOptimizerState.h>
#include <gtdynamics/imanifold/IEManifoldOptimizer.h>
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

using std::cout, std::setw, std::setprecision, std::endl;

namespace gtsam {

/* ************************************************************************* */
IELMState IELMState::FromLastIteration(const IELMIterDetails &iter_details,
                                       const NonlinearFactorGraph &graph,
                                       const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IELMState state;
  if (last_trial.step_is_successful) {
    state = IELMState(last_trial.new_manifolds, graph);
    // TODO: will this cause early ending? (converged in this mode, but not for
    // the overall problem)
    if (last_trial.forced_indices_map.size() > 0) {
      state.blocking_indices_map.mergeWith(last_trial.forced_indices_map);
    }
  } else {
    // pick the trials with smallest error
    state = IELMState(iter_details.state.manifolds, graph);
    for (const auto &trial : iter_details.trials) {
      if (trial.solve_successful && trial.new_error < state.error) {
        state = IELMState(trial.new_manifolds, graph);
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
  Values e_bare_manifolds = IEOptimizer::EManifolds(manifolds);
  auto linear_graph = manifold_graph.linearize(e_bare_manifolds);

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
void IELMTrial::setLambda(const IELMState &state) {
  lambda = state.lambda;
  lambda_factor = state.lambda_factor;
}

/* ************************************************************************* */
void IELMTrial::setNextLambda(double &new_lambda, double &new_lambda_factor,
                              const LevenbergMarquardtParams &params) const {
  if (forced_indices_map.size() > 0) {
    new_lambda = lambda;
    new_lambda_factor = lambda_factor;
  } else {
    if (step_is_successful) {
      setDecreasedNextLambda(new_lambda, new_lambda_factor, params);
    } else {
      setIncreasedNextLambda(new_lambda, new_lambda_factor, params);
    }
  }
}

/* ************************************************************************* */
void IELMTrial::setIncreasedNextLambda(
    double &new_lambda, double &new_lambda_factor,
    const LevenbergMarquardtParams &params) const {
  new_lambda = lambda * lambda_factor;
  new_lambda_factor = lambda_factor;
  if (!params.useFixedLambdaFactor) {
    new_lambda_factor *= 2.0;
  }
}

/* ************************************************************************* */
void IELMTrial::setDecreasedNextLambda(
    double &new_lambda, double &new_lambda_factor,
    const LevenbergMarquardtParams &params) const {
  new_lambda = lambda;
  new_lambda_factor = lambda_factor;
  if (params.useFixedLambdaFactor) {
    new_lambda /= lambda_factor;
  } else {
    new_lambda *= std::max(1.0 / 3.0, 1.0 - pow(2.0 * model_fidelity - 1.0, 3));
    new_lambda_factor = 2.0 * lambda_factor;
  }
  new_lambda = std::max(params.lambdaLowerBound, new_lambda);
}

/* ************************************************************************* */
VectorValues SqrtHessianDiagonal(const GaussianFactorGraph &graph,
                                 const LevenbergMarquardtParams &params) {
  VectorValues sqrt_hessian_diagonal;
  if (params.diagonalDamping) {
    sqrt_hessian_diagonal = graph.hessianDiagonal();
    for (auto &[key, value] : sqrt_hessian_diagonal) {
      value = value.cwiseMax(params.minDiagonal)
                  .cwiseMin(params.maxDiagonal)
                  .cwiseSqrt();
    }
  }
  return sqrt_hessian_diagonal;
}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr
IELMTrial::linearize(const NonlinearFactorGraph &graph,
                     const std::map<Key, Key> &keymap_var2manifold) const {
  // linearize on e-manifolds
  NonlinearFactorGraph manifold_graph = ManifoldOptimizer::ManifoldGraph(
      graph, keymap_var2manifold, const_e_manifolds);
  auto active_linear_graph = manifold_graph.linearize(e_manifolds);
  return active_linear_graph;
}

/* ************************************************************************* */
VectorValues IELMTrial::solve(const GaussianFactorGraph &gfg,
                              const NonlinearOptimizerParams &params) {
  // solution of linear solver is an update to the linearization point
  VectorValues delta;

  // Check which solver we are using
  if (params.isMultifrontal()) {
    // Multifrontal QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.optimize(*params.ordering, params.getEliminationFunction());
    else
      delta = gfg.optimize(params.getEliminationFunction());
  } else if (params.isSequential()) {
    // Sequential QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.eliminateSequential(*params.ordering,
                                      params.getEliminationFunction())
                  ->optimize();
    else
      delta = gfg.eliminateSequential(params.orderingType,
                                      params.getEliminationFunction())
                  ->optimize();
  } else if (params.isIterative()) {
    // Conjugate Gradient -> needs params.iterativeParams
    if (!params.iterativeParams)
      throw std::runtime_error(
          "NonlinearOptimizer::solve: cg parameter has to be assigned ...");

    if (auto pcg = std::dynamic_pointer_cast<PCGSolverParameters>(
            params.iterativeParams)) {
      delta = PCGSolver(*pcg).optimize(gfg);
    } else if (auto spcg = std::dynamic_pointer_cast<SubgraphSolverParameters>(
                   params.iterativeParams)) {
      if (!params.ordering)
        throw std::runtime_error("SubgraphSolver needs an ordering");
      delta = SubgraphSolver(gfg, *spcg, *params.ordering).optimize();
    } else {
      throw std::runtime_error(
          "NonlinearOptimizer::solve: special cg parameter type is not handled "
          "in LM solver ...");
    }
  } else {
    throw std::runtime_error(
        "NonlinearOptimizer::solve: Optimization parameter is invalid");
  }

  // return update
  return delta;
}

/* ************************************************************************* */
bool IELMTrial::computeDelta(const NonlinearFactorGraph &graph,
                             const IELMState &state,
                             const LevenbergMarquardtParams &params) {
  // initialize blocking constraints and e_manifolds
  std::map<Key, Key> keymap_var2manifold =
      IEOptimizer::Var2ManifoldKeyMap(state.manifolds);
  blocking_indices_map = state.blocking_indices_map;
  e_manifolds = state.e_manifolds;
  const_e_manifolds = state.const_e_manifolds;

  while (true) {
    // linearize and build damped system
    GaussianFactorGraph::shared_ptr linear =
        linearize(graph, keymap_var2manifold);
    VectorValues sqrt_hessian_diagonal = SqrtHessianDiagonal(*linear, params);
    auto damped_system =
        buildDampedSystem(*linear, sqrt_hessian_diagonal, params);

    // solve delta
    try {
      delta = solve(damped_system, params);
      solve_successful = true;
    } catch (const IndeterminantLinearSystemException &) {
      solve_successful = false;
      return false;
    }

    // check if satisfy tagent cone, if not, add constraints and recompute
    bool feasible = true;
    for (const auto &it : delta) {
      const Key &key = it.first;
      const Vector &xi = it.second;
      ConstraintManifold e_manifold = e_manifolds.at<ConstraintManifold>(key);
      VectorValues tv = e_manifold.basis()->computeTangentVector(xi);

      IndexSet blocking_indices = state.manifolds.at(key).blockingIndices(tv);
      if (blocking_indices.size() > 0) {
        feasible = false;
        blocking_indices_map.addIndices(key, blocking_indices);
      }
    }

    if (feasible) {
      computeTangentVector();
      old_linear_error = linear->error(VectorValues::Zero(delta));
      new_linear_error = linear->error(delta);
      linear_cost_change = old_linear_error - new_linear_error;
      break;
    } else {
      // recompute e_manifolds
      std::tie(e_manifolds, const_e_manifolds) =
          IEOptimizer::EManifolds(state.manifolds, blocking_indices_map);
    }
  }
  return true;
}

/* ************************************************************************* */
IELMTrial::CachedModel *IELMTrial::getCachedModel(size_t dim) const {
  if (dim >= noiseModelCache.size())
    noiseModelCache.resize(dim + 1);
  CachedModel *item = &noiseModelCache[dim];
  if (!item->model)
    *item = CachedModel(dim, 1.0 / std::sqrt(lambda));
  return item;
}

/* ************************************************************************* */
GaussianFactorGraph IELMTrial::buildDampedSystem(
    GaussianFactorGraph damped /* gets copied */) const {
  noiseModelCache.resize(0);
  // for each of the variables, add a prior
  damped.reserve(damped.size() + e_manifolds.size());
  std::map<Key, size_t> dims = e_manifolds.dims();
  for (const auto &key_dim : dims) {
    const Key &key = key_dim.first;
    const size_t &dim = key_dim.second;
    const CachedModel *item = getCachedModel(dim);
    damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
  }
  return damped;
}

/* ************************************************************************* */
GaussianFactorGraph
IELMTrial::buildDampedSystem(GaussianFactorGraph damped, // gets copied
                             const VectorValues &sqrtHessianDiagonal) const {
  noiseModelCache.resize(0);
  damped.reserve(damped.size() + e_manifolds.size());
  for (const auto &key_vector : sqrtHessianDiagonal) {
    try {
      const Key key = key_vector.first;
      const size_t dim = key_vector.second.size();
      CachedModel *item = getCachedModel(dim);
      item->A.diagonal() = sqrtHessianDiagonal.at(key); // use diag(hessian)
      damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
    } catch (const std::out_of_range &) {
      continue; // Don't attempt any damping if no key found in diagonal
    }
  }
  return damped;
}

/* ************************************************************************* */
GaussianFactorGraph
IELMTrial::buildDampedSystem(const GaussianFactorGraph &linear,
                             const VectorValues &sqrtHessianDiagonal,
                             const LevenbergMarquardtParams &params) const {

  if (params.diagonalDamping)
    return buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return buildDampedSystem(linear);
}

/* ************************************************************************* */
void IELMTrial::computeTangentVector() {
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
}

/* ************************************************************************* */
void IELMTrial::computeNewManifolds(const IELMState &state) {
  for (const auto &it : state.manifolds) {
    const Key &key = it.first;
    if (const_e_manifolds.exists(key)) {
      new_manifolds.emplace(key, it.second);
    } else {
      const Vector &xi = delta.at(key);
      ConstraintManifold e_manifold = e_manifolds.at<ConstraintManifold>(key);
      VectorValues tv = e_manifold.basis()->computeTangentVector(xi);
      if (blocking_indices_map.find(key) != blocking_indices_map.end()) {
        const auto &blocking_indices = blocking_indices_map.at(key);
        new_manifolds.emplace(key, it.second.retract(tv, blocking_indices));
      } else {
        new_manifolds.emplace(key, it.second.retract(tv));
      }
    }
  }
}

/* ************************************************************************* */
void IELMTrial::print(const IELMState &state) const {
  cout << setw(4) << state.iterations << " " << setw(10) << setprecision(4)
       << new_error << " " << setw(10) << setprecision(4)
       << nonlinear_cost_change << " " << setw(5) << setprecision(2) << lambda
       << " " << setw(4) << solve_successful << " " << setw(3)
       << setprecision(2) << trial_time << endl;
}

} // namespace gtsam
