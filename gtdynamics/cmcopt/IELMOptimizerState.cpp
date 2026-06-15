
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/IELMOptimizerState.h>
#include <gtdynamics/optimizer/ConvexIQPSolver.h>
#include <gtdynamics/utils/GraphUtils.h>

#include <iomanip>

using std::cout, std::setw, std::setprecision, std::endl;

namespace gtdynamics {
using namespace gtsam;


/* ************************************************************************* */
/* <============================ IELMState ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMState::IELMState(const IEManifoldValues &_manifolds,
                     const Values &unconstrainedValues,
                     const NonlinearFactorGraph &graph,
                     const NonlinearFactorGraph &manifold_graph,
                     const double &_lambda, const double &_lambdaFactor,
                     size_t _iterations)
    : manifolds(_manifolds),
      values(allValues(_manifolds, unconstrainedValues)),
      unconstrainedKeys(unconstrainedValues.keys()),
      error(evaluateGraphError(graph, manifolds, unconstrainedValues)),
      lambda(_lambda), lambdaFactor(_lambdaFactor), iterations(_iterations) {
  construct(graph, manifold_graph);
}

/* ************************************************************************* */
IELMState
IELMState::fromLastIteration(const IELMIterationDetails &iter_details,
                             const NonlinearFactorGraph &graph,
                             const NonlinearFactorGraph &manifold_graph,
                             const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IELMState state;
  if (last_trial.stepIsSuccessful) {
    state = IELMState(last_trial.nonlinearUpdate.newManifolds,
                      last_trial.nonlinearUpdate.newUnconstrainedValues,
                      graph, manifold_graph, last_trial.linearUpdate.lambda,
                      prev_state.lambdaFactor);
    // TODO: will this cause early ending? (converged in this mode, but not for
    // the overall problem)
    if (last_trial.forcedIndicesMap.size() > 0) {
      state.gradientBlockingIndicesMap.mergeWith(last_trial.forcedIndicesMap);
    }
  } else {
    // pick the trials with smallest error
    state =
        IELMState(prev_state.manifolds, prev_state.unconstrainedValues(), graph,
                  manifold_graph, prev_state.lambda, prev_state.lambdaFactor);
    for (const auto &trial : iter_details.trials) {
      if (trial.linearUpdate.solveSuccessful &&
          trial.nonlinearUpdate.newError < prev_state.error) {
        state = IELMState(trial.nonlinearUpdate.newManifolds,
                          trial.nonlinearUpdate.newUnconstrainedValues,
                          graph, manifold_graph, trial.linearUpdate.lambda,
                          prev_state.lambdaFactor);
      }
    }
  }

  last_trial.setNextLambda(state.lambda, state.lambdaFactor, params);
  state.iterations = prev_state.iterations + 1;
  state.totalInnerIterations =
      prev_state.totalInnerIterations + iter_details.trials.size();
  return state;
}

/* ************************************************************************* */
Values IELMState::allValues(const IEManifoldValues &manifolds,
                            const Values &unconstrainedValues) {
  Values values = IEOptimizer::equalityManifolds(manifolds);
  values.insert(unconstrainedValues);
  return values;
}

/* ************************************************************************* */
void IELMState::construct(const NonlinearFactorGraph &graph,
                          const NonlinearFactorGraph &manifold_graph) {
  // linearize costs
  baseLinear = graph.linearize(baseValues());
  linearManifoldGraph = manifold_graph.linearize(values);

  // linearize active i-constraints
  linearizeIConstraints();

  // compute graident
  computeGradient(manifold_graph);
}

/* ************************************************************************* */
void IELMState::linearizeIConstraints() {
  linearManifoldInequalityConstraints.resize(0);
  size_t index = 0;
  for (const auto &[key, manifold] : manifolds) {
    auto man_constraints = manifold.linearActiveManifoldInequalityConstraints(key);
    auto base_constraints = manifold.linearActiveBaseInequalityConstraints();
    for (const auto &[constraint_idx, constraint] : man_constraints) {
      linearManifoldInequalityConstraints.push_back(constraint);
      linearBaseInequalityConstraints.push_back(base_constraints.at(constraint_idx));
      linearInequalityIndexTranslator.insert(index, key, constraint_idx);
      index++;
    }
  }
}

/* ************************************************************************* */
void IELMState::computeGradient(const NonlinearFactorGraph &manifold_graph) {
  gradient = linearManifoldGraph->gradientAtZero();
  VectorValues descentDirection = -1 * gradient;

  // identify blocking constraints
  gradientBlockingIndicesMap =
      IEOptimizer::projectTangentCone(manifolds, descentDirection).first;
}

/* ************************************************************************* */
double IELMState::evaluateGraphError(const NonlinearFactorGraph &graph,
                                     const IEManifoldValues &_manifolds,
                                     const Values &_unconstrained_values) {
  Values all_values = _manifolds.baseValues();
  all_values.insert(_unconstrained_values);
  return graph.error(all_values);
}

/* ************************************************************************* */
Values IELMState::unconstrainedValues() const {
  return SubValues(values, unconstrainedKeys);
}

/* ************************************************************************* */
Values IELMState::baseValues() const {
  Values base_values = manifolds.baseValues();
  base_values.insert(unconstrainedValues());
  return base_values;
}

/* ************************************************************************* */
VectorValues
IELMState::computeMetricSigmas(const NonlinearFactorGraph &graph) const {
  Values base_values = baseValues();
  auto linear_graph = graph.linearize(base_values);
  auto hessian_diag = linear_graph->hessianDiagonal();
  // hessian_diag.print("hessian diag:", GTDKeyFormatter);
  VectorValues metricSigmas;
  for (auto &[key, value] : hessian_diag) {
    Vector sigmas_sqr_inv = hessian_diag.at(key);
    if (sigmas_sqr_inv.norm() < 1e-8) {
      continue;
    }
    Vector sigmas = sigmas_sqr_inv;
    for (int i = 0; i < sigmas.size(); i++) {
      if (sigmas_sqr_inv(i) < 1e-8) {
        sigmas(i) = 1e4;
      } else {
        sigmas(i) = 1 / sqrt(sigmas_sqr_inv(i));
      }
    }
    metricSigmas.insert(key, sigmas);
  }
  metricSigmas = 10 * metricSigmas;
  // metricSigmas.print("metric sigmas:", GTDKeyFormatter);

  return metricSigmas;
}

/* ************************************************************************* */
/* <============================ IELMTrial ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMTrial::IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
                     const double &lambda, const IELMParams &params) {
  // std::cout << "========= " << state.iterations << " ======= \n";
  auto start = std::chrono::high_resolution_clock::now();
  stepIsSuccessful = false;

  // Compute linear update and linear cost change
  linearUpdate = LinearUpdate(lambda, graph, state, params);
  if (!linearUpdate.solveSuccessful) {
    return;
  }

  // Compute nonlinear update and nonlinear cost change
  nonlinearUpdate = NonlinearUpdate(state, linearUpdate, graph);

  // Decide if accept or reject trial
  modelFidelity = nonlinearUpdate.costChange / linearUpdate.costChange;
  if (linearUpdate.costChange >
          std::numeric_limits<double>::epsilon() * linearUpdate.oldError &&
      modelFidelity > params.lmParams.minModelFidelity) {
    stepIsSuccessful = true;
  }

  auto end = std::chrono::high_resolution_clock::now();
  trialTime =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1e6;
}

/* ************************************************************************* */
IELMTrial::IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
                     const IndexSetMap &approach_indices_map) {
  auto start = std::chrono::high_resolution_clock::now();

  forcedIndicesMap = approach_indices_map;
  linearUpdate = LinearUpdate::zero(state);
  nonlinearUpdate = NonlinearUpdate(state, forcedIndicesMap, graph);
  stepIsSuccessful = true;

  auto end = std::chrono::high_resolution_clock::now();
  trialTime =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1e6;
}

/* ************************************************************************* */
void IELMTrial::setNextLambda(double &new_lambda, double &newLambdaFactor,
                              const LevenbergMarquardtParams &params) const {
  if (forcedIndicesMap.size() > 0) {
    return;
  }
  if (stepIsSuccessful) {
    setDecreasedNextLambda(new_lambda, newLambdaFactor, params);
  } else {
    setIncreasedNextLambda(new_lambda, newLambdaFactor, params);
  }
}

/* ************************************************************************* */
void IELMTrial::setIncreasedNextLambda(
    double &new_lambda, double &newLambdaFactor,
    const LevenbergMarquardtParams &params) const {
  new_lambda *= newLambdaFactor;
  if (!params.useFixedLambdaFactor) {
    newLambdaFactor *= 2.0;
  }
}

/* ************************************************************************* */
void IELMTrial::setDecreasedNextLambda(
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
/* <============================= Logging =================================> */
/* ************************************************************************* */

/* ************************************************************************* */
std::map<std::pair<Key, size_t>, size_t>
IdentifyConstraintType(const IEManifoldValues &state_manifolds,
                       const IndexSetMap blockingIndicesMap,
                       const IEManifoldValues &newManifolds) {
  std::map<std::pair<Key, size_t>, size_t> constraint_type_map;

  for (const auto &[key, new_manifold] : newManifolds) {
    const auto &state_manifold = state_manifolds.at(key);
    IndexSet blocking_indices;
    if (blockingIndicesMap.exists(key)) {
      blocking_indices = blockingIndicesMap.at(key);
    }
    for (const auto &constraint_idx : new_manifold.activeIndices()) {
      size_t constraint_type = 0;
      if (!state_manifold.activeIndices().exists(constraint_idx)) {
        constraint_type = 2;
      } else {
        if (blocking_indices.exists(constraint_idx)) {
          constraint_type = 1;
        }
      }
      constraint_type_map.insert({{key, constraint_idx}, constraint_type});
    }
    for (const auto &constraint_idx : state_manifold.activeIndices()) {
      if (!new_manifold.activeIndices().exists(constraint_idx)) {
        constraint_type_map.insert({{key, constraint_idx}, 3});
      }
    }
  }
  return constraint_type_map;
}

/* ************************************************************************* */
std::pair<std::string, size_t> SplitStr(const std::string &str) {
  size_t i = str.size();
  while (i > 0) {
    if (str[i - 1] >= '0' && str[i - 1] <= '9') {
      i--;
    } else {
      break;
    }
  }
  std::string category_str = str.substr(0, i);
  std::string k_str = str.substr(i, str.size() - i);
  return {category_str, std::stoi(k_str)};
}

/* ************************************************************************* */
std::string ColoredStr(const std::string &str, const size_t constraint_type,
                       const std::string &default_color_str) {
  std::string color_str;
  if (constraint_type == 0) {
    color_str = "90"; // dark_gray
  }
  if (constraint_type == 1) {
    color_str = "30"; // black
  }
  if (constraint_type == 2) {
    color_str = "31"; // red
  }
  if (constraint_type == 3) {
    color_str = "37"; // light_gray
  }
  return "\033[0;" + color_str + "m" + str + default_color_str;
}

/* ************************************************************************* */
std::string ConstraintInfoStr(const IEManifoldValues &state_manifolds,
                              const IndexSetMap blockingIndicesMap,
                              const IEManifoldValues &newManifolds,
                              const KeyFormatter &key_formatter,
                              bool stepIsSuccessful,
                              bool group_as_categories) {

  std::string default_color_str = stepIsSuccessful ? "\033[0m" : "\033[090m";

  auto constraint_type_map = IdentifyConstraintType(
      state_manifolds, blockingIndicesMap, newManifolds);

  if (!group_as_categories) {
    std::string str = "";
    for (const auto &[key, manifold] : newManifolds) {
      for (const auto &constraint_idx : manifold.activeIndices()) {
        auto constraint_type = constraint_type_map.at({key, constraint_idx});
        str += ColoredStr(
            " " + key_formatter(key) + ":" + std::to_string(constraint_idx),
            constraint_type,
                          default_color_str);
      }
    }
    return str;
  }

  std::map<std::string, std::vector<std::pair<size_t, size_t>>>
      category_constraints;

  for (const auto &[key, manifold] : newManifolds) {
    for (const auto &constraint_idx : manifold.activeIndices()) {
      auto category = key_formatter(key);
      size_t k = constraint_idx;
      size_t constraint_type = constraint_type_map.at({key, constraint_idx});
      if (category_constraints.find(category) == category_constraints.end()) {
        category_constraints.insert(
            {category, std::vector<std::pair<size_t, size_t>>()});
      }
      category_constraints.at(category).emplace_back(k, constraint_type);
    }
  }

  std::string str = "";
  for (const auto &[category, constraints_info] : category_constraints) {
    str += " " + category + "(";
    for (const auto &[k, constraint_type] : constraints_info) {
      if (str.back() != '(') {
        str += ",";
      }
      str += ColoredStr(std::to_string(k), constraint_type, default_color_str);
    }
    str += ")";
  }

  return str;
}

/* ************************************************************************* */
void printIELMTrialTitle() {
  cout << setw(10) << "iter   "
       << "|" << setw(12) << "error  "
       << "|" << setw(12) << "nonlinear "
       << "|" << setw(12) << "linear   "
       << "|" << setw(12) << "linear_retr"
       << "|" << setw(10) << "lambda  "
       << "|" << setw(10) << "numSolves"
       << "|" << setw(17) << "retract_devi "
       << "|" << setw(10) << "time  "
       << "|" << setw(10) << "delta_norm"
       << "|" << setw(30) << "active constraints" << endl;
}

double VectorMean(const std::vector<double> &vec) {
  if (vec.size() == 0) {
    return 0;
  }
  return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}

double VectorMax(const std::vector<double> &vec) {
  if (vec.size() == 0) {
    return 0;
  }
  return *std::max_element(std::begin(vec), std::end(vec));
}

/* ************************************************************************* */
std::string ConstraintInfoStr(const IEManifoldValues &manifolds,
                              const IndexSetMap &indices_map,
                              const KeyFormatter &key_formatter) {
  std::string str = "";
  for (const auto &[key, index_set] : indices_map) {
    for (const auto &i_idx : index_set) {
      str += " " + key_formatter(key) + ":" + std::to_string(i_idx);
    }
  }
  return str;
}

/* ************************************************************************* */
void printIELMTrial(const IELMState &state, const IELMTrial &trial,
                    const IELMParams &params, bool forced,
                    const KeyFormatter &key_formatter) {
  if (!trial.stepIsSuccessful) {
    cout << "\033[90m";
  }
  const auto &nonlinearUpdate = trial.nonlinearUpdate;
  const auto &linearUpdate = trial.linearUpdate;
  cout << setw(10) << state.iterations << "|";
  cout << setw(12) << setprecision(4) << nonlinearUpdate.newError << "|";
  cout << setw(12) << setprecision(4) << nonlinearUpdate.costChange << "|";
  if (!forced) {
    cout << setw(12) << setprecision(4) << linearUpdate.costChange << "|";
    cout << setw(12) << setprecision(4)
         << nonlinearUpdate.linearCostChangeWithRetractionDelta << "|";
    cout << setw(10) << setprecision(2) << linearUpdate.lambda << "|";
    if (!linearUpdate.solveSuccessful) {
      cout << "linear solve not successful\n";
      return;
    }
    cout << setw(4) << linearUpdate.numSolves << "|" << setw(5) << std::left
         << nonlinearUpdate.numRetractionIterations << std::right << "|";
    cout << setw(8) << VectorMean(nonlinearUpdate.retractionDeviationRates) << "|"
         << setw(8) << std::left
         << VectorMax(nonlinearUpdate.retractionDeviationRates) << std::right
         << "|";
    cout << setw(10) << setprecision(2) << trial.trialTime << "|";
    cout << setw(10) << setprecision(4) << linearUpdate.delta.norm() << "|";
    if (params.showActiveConstraints) {
      cout << ConstraintInfoStr(
          state.manifolds, linearUpdate.blockingIndicesMap,
          nonlinearUpdate.newManifolds, GTDKeyFormatter,
          trial.stepIsSuccessful,
          params.activeConstraintsGroupedAsCategories);
    }
    // cout << setw(10) << setprecision(4) <<
    // linearUpdate.tangentVector.norm()
    //      << " ";
    // cout << setw(10) << setprecision(4)
    //      << linearUpdate.tangentVector.norm() / linearUpdate.delta.norm()
    //      << " ";
  } else {
    std::string forced_i_str =
        "forced:" + ConstraintInfoStr(state.manifolds, trial.forcedIndicesMap,
                                      key_formatter);
    cout << forced_i_str;
  }
  if (!trial.stepIsSuccessful) {
    cout << "\033[0m";
  }
  cout << endl;
}

/* ************************************************************************* */
/* <===================== IELMTrial::LinearUpdate =========================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMTrial::LinearUpdate::LinearUpdate(const double &_lambda,
                                      const NonlinearFactorGraph &graph,
                                      const IELMState &state,
                                      const IELMParams &params)
    : lambda(_lambda), numSolves(0) {

  // build damped system
  GaussianFactorGraph::shared_ptr linear = state.linearManifoldGraph;
  VectorValues sqrt_hessian_diagonal =
      SqrtHessianDiagonal(*linear, params.lmParams);
  auto damped_system = buildDampedSystem(*linear, sqrt_hessian_diagonal, state,
                                         params.lmParams);
  IndexSet blocking_indices;

  // no active constraints
  if (state.linearBaseInequalityConstraints.size() == 0) {
    try {
      delta = SolveLinear(damped_system, params.lmParams);
      solveSuccessful = true;
    } catch (const IndeterminantLinearSystemException &) {
      solveSuccessful = false;
    }
    numSolves = 1;
  } else {
    // solve IQP init estimate
    std::tie(delta, blocking_indices, numSolves, solveSuccessful) =
        initialEstimate(damped_system, state, params.lmParams);
    if (!solveSuccessful) {
      return;
    }

    // solve IQP
    if (params.iqpMaxIterations > 0) {
      size_t num_new_solves;
      std::tie(delta, blocking_indices, num_new_solves, solveSuccessful) =
          SolveConvexIQP(damped_system, state.linearManifoldInequalityConstraints,
                         blocking_indices, delta, params.iqpMaxIterations);
      numSolves += num_new_solves;
    }
  }

  // record linear update info
  blockingIndicesMap =
      state.linearInequalityIndexTranslator.decodeIndices(blocking_indices);
  tangentVector = computeTangentVector(state, delta, state.unconstrainedKeys);
  checkSolutionValid(state, tangentVector, blocking_indices);
  oldError = linear->error(VectorValues::Zero(delta));
  newError = linear->error(delta);
  costChange = oldError - newError;
}

/* ************************************************************************* */
IELMTrial::LinearUpdate IELMTrial::LinearUpdate::zero(const IELMState &state) {
  LinearUpdate linearUpdate;
  linearUpdate.lambda = state.lambda;
  linearUpdate.delta = state.values.zeroVectors();
  linearUpdate.tangentVector = state.baseValues().zeroVectors();
  return linearUpdate;
}

/* ************************************************************************* */
std::tuple<VectorValues, IndexSet, size_t, bool>
IELMTrial::LinearUpdate::initialEstimate(const GaussianFactorGraph &quadratic_cost,
                                      const IELMState &state,
                                      const LevenbergMarquardtParams &params) {

  IndexSet blocking_indices =
      state.linearInequalityIndexTranslator.encodeIndices(state.gradientBlockingIndicesMap);
  size_t numSolves = 0;

  while (true) {
    numSolves += 1;
    GaussianFactorGraph graph = quadratic_cost;
    GaussianFactorGraph constraint_graph =
        state.linearManifoldInequalityConstraints.constraintGraph(blocking_indices);
    graph.push_back(constraint_graph.begin(), constraint_graph.end());

    VectorValues delta;
    try {
      delta = SolveLinear(graph, params);
    } catch (const IndeterminantLinearSystemException &) {
      return {delta, blocking_indices, numSolves, false};
    }

    // check if satisfy tangent cone, if not, add constraints and recompute
    bool feasible = true;
    for (const auto &[key, manifold] : state.manifolds) {
      const Vector &xi = delta.at(key);
      VectorValues tv = manifold.eBasis()->computeTangentVector(xi);

      IndexSet man_blocking_indices = manifold.blockingIndices(tv);
      for (const auto &constraint_idx : man_blocking_indices) {
        size_t index =
            state.linearInequalityIndexTranslator.encoder.at({key, constraint_idx});
        if (!blocking_indices.exists(index)) {
          feasible = false;
          blocking_indices.insert(index);
        }
      }
    }
    if (feasible) {
      return {delta, blocking_indices, numSolves, true};
    }
  }
}

/* ************************************************************************* */
bool IELMTrial::LinearUpdate::checkSolutionValid(
    const IELMState &state, const VectorValues &tangentVector,
    const IndexSet &blocking_indices) {
  for (const auto &constraint : state.linearBaseInequalityConstraints) {
    if (!constraint->feasible(tangentVector, 1e-5)) {
      std::cout << "tangent vector violating constraint: "
                << (*constraint)(tangentVector).transpose() << "\n";
      return false;
    }
  }
  for (const auto &constraint_idx : blocking_indices) {
    const auto &constraint = state.linearBaseInequalityConstraints.at(constraint_idx);
    if (!constraint->isActive(tangentVector)) {
      std::cout << "blocking constraint is not active: "
                << (*constraint)(tangentVector).transpose() << "\n";
      return false;
    }
  }
  return true;
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
  damped.reserve(damped.size() + state.values.size());
  std::map<Key, size_t> dims = state.values.dims();
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

  if (params.getDiagonalDamping())
    return buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return buildDampedSystem(linear, state);
}

/* ************************************************************************* */
VectorValues IELMTrial::LinearUpdate::computeTangentVector(
    const IELMState &state, const VectorValues &delta,
    const KeySet &unconstrainedKeys) const {
  VectorValues tangentVector;
  for (const auto &[key, xi] : delta) {
    if (unconstrainedKeys.exists(key)) {
      tangentVector.insert(key, delta.at(key));
    } else {
      const auto &manifold = state.manifolds.at(key);
      VectorValues tv = manifold.eBasis()->computeTangentVector(xi);
      tangentVector.insert(tv);
    }
  }

  return tangentVector;
}

/* ************************************************************************* */
/* <==================== IELMTrial::NonlinearUpdate =======================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMTrial::NonlinearUpdate::NonlinearUpdate(const IELMState &state,
                                            const LinearUpdate &linearUpdate,
                                            const NonlinearFactorGraph &graph) {

  // retract for ie-manifolds
  numRetractionIterations = 0;
  VectorValues retract_delta;
  for (const auto &[key, manifold] : state.manifolds) {
    VectorValues tv =
        SubValues(linearUpdate.tangentVector, manifold.values().keys());
    const auto &blockingIndicesMap = linearUpdate.blockingIndicesMap;
    IERetractionInfo retract_info;
    if (blockingIndicesMap.find(key) != blockingIndicesMap.end()) {
      const auto &blocking_indices = blockingIndicesMap.at(key);
      newManifolds.emplace(
          key, manifold.retract(tv, blocking_indices, &retract_info));
    } else {
      newManifolds.emplace(key, manifold.retract(tv, {}, &retract_info));
    }
    numRetractionIterations += retract_info.numLMIterations;
    auto [manifold_retract_delta, retract_deviation_rate] =
        evaluateRetractionDeviation(manifold, newManifolds.at(key), tv);
    retractionDeviationRates.push_back(retract_deviation_rate);
    retract_delta.insert(manifold_retract_delta);
  }

  // retract for unconstrained variables
  VectorValues tangent_vector_unconstrained =
      SubValues(linearUpdate.tangentVector, state.unconstrainedKeys);
  newUnconstrainedValues =
      state.unconstrainedValues().retract(tangent_vector_unconstrained);
  retract_delta.insert(tangent_vector_unconstrained);

  // compute error
  computeError(graph, state.error);
  VectorValues zero_vec = VectorValues::Zero(retract_delta);
  // KeyVector zero_vec_kv;
  // for (const auto &[key, v] : zero_vec) {
  //   zero_vec_kv.push_back(key);
  // }
  // PrintKeyVector(zero_vec_kv, "zero_vec", GTDKeyFormatter);
  // PrintKeySet(state.baseLinear->keys(), "graph",
  // GTDKeyFormatter);
  double base_linear_error = state.baseLinear->error(zero_vec);
  double base_linear_error_retract = state.baseLinear->error(retract_delta);
  linearCostChangeWithRetractionDelta =
      base_linear_error - base_linear_error_retract;
}

/* ************************************************************************* */
IELMTrial::NonlinearUpdate::NonlinearUpdate(
    const IELMState &state, const IndexSetMap &forcedIndicesMap,
    const NonlinearFactorGraph &graph) {
  newManifolds = state.manifolds.moveToBoundaries(forcedIndicesMap);
  newUnconstrainedValues = state.unconstrainedValues();
  computeError(graph, state.error);
}

/* ************************************************************************* */
std::pair<VectorValues, double>
IELMTrial::NonlinearUpdate::evaluateRetractionDeviation(
    const IEConstraintManifold &manifold,
    const IEConstraintManifold &new_manifold,
    const VectorValues &tangentVector) {
  VectorValues retract_delta =
      manifold.values().localCoordinates(new_manifold.values());
  auto vec_diff = retract_delta - tangentVector;
  double tangent_vector_norm = tangentVector.norm();
  double retract_deviation_rate;
  if (abs(tangent_vector_norm) < 1e-10) {
    retract_deviation_rate = 0;
  } else {
    double diff_norm = vec_diff.norm();
    retract_deviation_rate = diff_norm / tangent_vector_norm;
  }
  return {retract_delta, retract_deviation_rate};
}

/* ************************************************************************* */
void IELMTrial::NonlinearUpdate::computeError(const NonlinearFactorGraph &graph,
                                              const double &oldError) {

  newError = IELMState::evaluateGraphError(graph, newManifolds,
                                            newUnconstrainedValues);
  costChange = oldError - newError;
}

/* ************************************************************************* */
/* <========================= IELMOptimizationDetails ============================> */
/* ************************************************************************* */
void IELMOptimizationDetails::exportFile(const std::string &state_file_path,
                                  const std::string &trial_file_path) const {
  std::ofstream state_file, trial_file;
  state_file.open(state_file_path);
  trial_file.open(trial_file_path);

  state_file << "iterations"
             << ",lambda"
             << ",error"
             << "\n";
  trial_file << "iterations"
             << ",lambda"
             << ",error"
             << ",stepIsSuccessful"
             << ",linearCostChange"
             << ",linearCostChangeWithRetractionDelta"
             << ",nonlinearCostChange"
             << ",modelFidelity"
             << ",tangent_vector_norm"
             << ",num_solves_linear"
             << ",num_solves_retraction"
             << ",avg_retract_deviation_rate"
             << ",max_retract_deviation_rate"
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
                 << trial.nonlinearUpdate.linearCostChangeWithRetractionDelta
                 << "," << trial.nonlinearUpdate.costChange << ","
                 << trial.modelFidelity << ","
                 << trial.linearUpdate.tangentVector.norm() << ","
                 << trial.linearUpdate.numSolves << ","
                 << trial.nonlinearUpdate.numRetractionIterations << ","
                 << VectorMean(trial.nonlinearUpdate.retractionDeviationRates)
                 << ","
                 << VectorMax(trial.nonlinearUpdate.retractionDeviationRates)
                 << "\n";
    }
  }
  state_file.close();
  trial_file.close();
}

} // namespace gtdynamics
