/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file IELMOptimizerState.cpp
 * @brief Implement CMC Levenberg-Marquardt states and trial updates.
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/AdmmSolver.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/IELMOptimizerState.h>
#include <gtdynamics/cmcopt/ProxQpSolver.h>
#include <gtdynamics/constrained/internal/IndependentComponents.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/ActiveSetSolver.h>

#include <algorithm>
#include <chrono>
#include <iomanip>

using std::cout, std::setw, std::setprecision, std::endl;

namespace gtdynamics {
using namespace gtsam;

namespace {
// Record elapsed time even when a solve rejects a trial or throws.
class ElapsedTime {
  double& seconds_;
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();

 public:
  explicit ElapsedTime(double& seconds) : seconds_(seconds) {}
  ~ElapsedTime() {
    seconds_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_).count();
  }
};
}  // namespace


/* ************************************************************************* */
/* <============================ IELMState ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMState::IELMState(const IECMPoints &componentPoints,
                     const Values &unconstrainedValues,
                     const NonlinearFactorGraph &graph,
                     const NonlinearFactorGraph &pointGraph,
                     const double &_lambda, const double &_lambdaFactor,
                     size_t _iterations)
    : points(componentPoints),
      values(allValues(componentPoints, unconstrainedValues)),
      unconstrainedKeys(unconstrainedValues.keys()),
      error(evaluateGraphError(graph, points, unconstrainedValues)),
      lambda(_lambda), lambdaFactor(_lambdaFactor), iterations(_iterations) {
  construct(graph, pointGraph);
}

/* ************************************************************************* */
IELMState
IELMState::fromLastIteration(const IELMIterationDetails &iter_details,
                             const NonlinearFactorGraph &graph,
                             const NonlinearFactorGraph &pointGraph,
                             const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IELMState state;
  if (last_trial.stepIsSuccessful) {
    state = IELMState(last_trial.nonlinearUpdate.newPoints,
                      last_trial.nonlinearUpdate.newUnconstrainedValues,
                      graph, pointGraph, last_trial.linearUpdate.lambda,
                      prev_state.lambdaFactor);
  } else {
    // pick the trials with smallest error
    state =
        IELMState(prev_state.points, prev_state.unconstrainedValues(), graph,
                  pointGraph, prev_state.lambda, prev_state.lambdaFactor);
    for (const auto &trial : iter_details.trials) {
      if (trial.linearUpdate.solveSuccessful &&
          trial.nonlinearUpdate.newError < prev_state.error) {
        state = IELMState(trial.nonlinearUpdate.newPoints,
                          trial.nonlinearUpdate.newUnconstrainedValues,
                          graph, pointGraph, trial.linearUpdate.lambda,
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
Values IELMState::allValues(const IECMPoints &points,
                            const Values &unconstrainedValues) {
  Values values;
  for (const auto &[key, point] : points) values.insert(key, point);
  values.insert(unconstrainedValues);
  return values;
}

/* ************************************************************************* */
void IELMState::construct(const NonlinearFactorGraph &graph,
                          const NonlinearFactorGraph &pointGraph) {
  ElapsedTime timing(linearizationSeconds);
  equalityOnly = std::all_of(points.begin(), points.end(), [](const auto& item) {
    return item.second.manifold()->inequalityCount(item.second) == 0;
  });
  // linearize costs
  baseLinear = graph.linearize(baseValues());
  linearPointGraph = pointGraph.linearize(values);

  const auto dimensions = values.dims();
  std::map<KeyVector, GaussianFactorGraph> costScopes;
  for (auto &factor : *linearPointGraph) {
    if (!factor) continue;
    // A fully fixed component can expose an all-zero placeholder Jacobian
    // column. Its Values dimension is authoritative, not the factor width.
    bool hasFixedComponent = false;
    for (Key key : *factor) hasFixedComponent |= dimensions.at(key) == 0;
    if (hasFixedComponent) {
      const JacobianFactor jacobian(*factor);
      std::vector<std::pair<Key, Matrix>> terms;
      for (auto key = jacobian.begin(); key != jacobian.end(); ++key)
        if (dimensions.at(*key)) terms.emplace_back(*key, jacobian.getA(key));
      // Keep constant factors for undamped model-error bookkeeping.
      factor = std::make_shared<JacobianFactor>(terms, jacobian.getb(),
                                                jacobian.get_model());
    }
    if (!factor->empty()) {
      KeyVector scope = factor->keys();
      std::sort(scope.begin(), scope.end());
      costScopes[scope].push_back(factor);
    }
  }
  // Combine matching scopes without forming normal equations. Sparse QP
  // elimination can then retain rank-deficient costs in square-root form.
  for (const auto &[scope, factors] : costScopes) {
    const auto factor = std::make_shared<JacobianFactor>(factors, Ordering(scope));
    reducedObjective.push_back(factor);
    if (!equalityOnly) tangentQp.addCost(*factor);
  }
  gradient = linearPointGraph->gradientAtZero();
  for (const auto &[key, dimension] : dimensions)
    if (!dimension) gradient.insert(key, Vector(0));

  size_t firstRow = 0;
  for (const auto &[key, point] : points) {
    const auto &local = *point.tangentSpace();
    for (size_t index : local.indices()) {
      // Whiten exactly once, using the cached directional Jacobian. The RHS
      // stays zero even for explicitly selected near-boundary modes.
      const Matrix &rows = local.rows(index, true);
      if (!rows.cols() || !rows.rows()) continue;
      tangentQp.addConstraint(LinearConstraint::LessEqual(
          JacobianFactor(key, rows, Vector::Zero(rows.rows()))));
      qpConstraintBlocks.push_back(
          {key, index, firstRow, static_cast<size_t>(rows.rows())});
      firstRow += rows.rows();
    }
  }
}

/* ************************************************************************* */
double IELMState::evaluateGraphError(const NonlinearFactorGraph &graph,
                                     const IECMPoints &componentPoints,
                                     const Values &_unconstrained_values) {
  Values all_values = ambientValues(componentPoints);
  all_values.insert(_unconstrained_values);
  return graph.error(all_values);
}

/* ************************************************************************* */
Values IELMState::unconstrainedValues() const {
  return SubValues(values, unconstrainedKeys);
}

/* ************************************************************************* */
Values IELMState::baseValues() const {
  Values base_values = ambientValues(points);
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
                     const double &lambda, const IELMParams &params,
                     internal::AdmmWorkspace *admmWorkspace) {
  // std::cout << "========= " << state.iterations << " ======= \n";
  auto start = std::chrono::high_resolution_clock::now();
  stepIsSuccessful = false;

  // Compute linear update and linear cost change
  linearUpdate = LinearUpdate(lambda, graph, state, params, admmWorkspace);
  if (!linearUpdate.solveSuccessful) {
    return;
  }

  // Compute nonlinear update and nonlinear cost change
  try {
    ElapsedTime timing(retractionSeconds);
    nonlinearUpdate =
        NonlinearUpdate(state, linearUpdate, graph, params.retractionThreads);
  } catch (const RetractionFailure &failure) {
    // A failed projection is not a candidate state. Retry at a smaller LM step;
    // do not let stopping tests accept an uncomputed nonlinear error or delta.
    linearUpdate.solveSuccessful = false;
    linearUpdate.failureReason = failure.what();
    nonlinearUpdate.newError = std::numeric_limits<double>::infinity();
    nonlinearUpdate.costChange = -std::numeric_limits<double>::infinity();
    modelFidelity = -std::numeric_limits<double>::infinity();
    trialTime = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - start)
                    .count();
    if (params.lmParams.verbosityLM == LevenbergMarquardtParams::SUMMARY)
      std::cout << "Retraction rejected at lambda " << lambda << ": "
                << failure.what() << std::endl;
    return;
  }

  // Decide if accept or reject trial
  modelFidelity = nonlinearUpdate.costChange / linearUpdate.costChange;
  if (linearUpdate.costChange >
          std::numeric_limits<double>::epsilon() * linearUpdate.oldError &&
      modelFidelity > params.lmParams.minModelFidelity) {
    stepIsSuccessful = true;
  }

  auto end = std::chrono::high_resolution_clock::now();
  trialTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                  .count() /
              1e6;
}

/* ************************************************************************* */
IELMTrial::IELMTrial(const IELMState &state, const NonlinearFactorGraph &graph,
                     const IndexSetMap &approach_indices_map) {
  auto start = std::chrono::high_resolution_clock::now();

  forcedIndicesMap = approach_indices_map;
  linearUpdate = LinearUpdate::zero(state);
  {
    ElapsedTime timing(retractionSeconds);
    nonlinearUpdate = NonlinearUpdate(state, forcedIndicesMap, graph);
  }
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
IdentifyConstraintType(const IECMPoints &statePoints,
                       const IndexSetMap blockingIndicesMap,
                       const IECMPoints &newPoints) {
  std::map<std::pair<Key, size_t>, size_t> constraint_type_map;

  for (const auto &[key, newPoint] : newPoints) {
    const auto &statePoint = statePoints.at(key);
    IndexSet blocking_indices;
    if (blockingIndicesMap.exists(key)) {
      blocking_indices = blockingIndicesMap.at(key);
    }
    for (const auto &constraint_idx : newPoint.activeIndices()) {
      size_t constraint_type = 0;
      if (!statePoint.activeIndices().exists(constraint_idx)) {
        constraint_type = 2;
      } else {
        if (blocking_indices.exists(constraint_idx)) {
          constraint_type = 1;
        }
      }
      constraint_type_map.insert({{key, constraint_idx}, constraint_type});
    }
    for (const auto &constraint_idx : statePoint.activeIndices()) {
      if (!newPoint.activeIndices().exists(constraint_idx)) {
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
std::string ConstraintInfoStr(const IECMPoints &statePoints,
                              const IndexSetMap blockingIndicesMap,
                              const IECMPoints &newPoints,
                              const KeyFormatter &key_formatter,
                              bool stepIsSuccessful,
                              bool group_as_categories) {

  std::string default_color_str = stepIsSuccessful ? "\033[0m" : "\033[090m";

  auto constraint_type_map = IdentifyConstraintType(
      statePoints, blockingIndicesMap, newPoints);

  if (!group_as_categories) {
    std::string str = "";
    for (const auto &[key, point] : newPoints) {
      for (const auto &constraint_idx : point.activeIndices()) {
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

  for (const auto &[key, point] : newPoints) {
    for (const auto &constraint_idx : point.activeIndices()) {
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
       << "|" << setw(10) << "QP/retract"
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
std::string ConstraintInfoStr(const IECMPoints &points,
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
      cout << "trial failed: " << linearUpdate.failureReason << "\033[0m\n";
      return;
    }
    cout << setw(4) << linearUpdate.qpIterations << "|" << setw(5) << std::left
         << nonlinearUpdate.numRetractionIterations << std::right << "|";
    cout << setw(8) << VectorMean(nonlinearUpdate.retractionDeviationRates) << "|"
         << setw(8) << std::left
         << VectorMax(nonlinearUpdate.retractionDeviationRates) << std::right
         << "|";
    cout << setw(10) << setprecision(2) << trial.trialTime << "|";
    cout << setw(10) << setprecision(4) << linearUpdate.delta.norm() << "|";
    if (params.showActiveConstraints) {
      cout << ConstraintInfoStr(
          state.points, linearUpdate.blockingIndicesMap,
          nonlinearUpdate.newPoints, GTDKeyFormatter,
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
        "forced:" + ConstraintInfoStr(state.points, trial.forcedIndicesMap,
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
                                      const IELMParams &params,
                                      internal::AdmmWorkspace *admmWorkspace)
    : lambda(_lambda) {
  const bool gaussian =
      params.linearSolver == IELMParams::LinearSolver::Gaussian ||
      (params.linearSolver == IELMParams::LinearSolver::Automatic &&
       state.equalityOnly);
  if (gaussian && !state.equalityOnly)
    throw std::invalid_argument("Gaussian CMC solves require empty inequalities");

  // Validate a complete reduced ordering before either backend sees it.
  auto lmParams = params.lmParams;
  if (lmParams.ordering) {
    Ordering ordering;
    KeySet seen;
    const auto dimensions = state.values.dims();
    for (Key key : *lmParams.ordering) {
      const auto found = dimensions.find(key);
      if (found == dimensions.end() || !seen.insert(key).second)
        throw std::invalid_argument("Invalid reduced ordering key");
      if (found->second) ordering.push_back(key);
    }
    const size_t count = std::count_if(dimensions.begin(), dimensions.end(),
        [](const auto& item) { return item.second != 0; });
    if (ordering.size() != count)
      throw std::invalid_argument("Incomplete reduced ordering");
    lmParams.ordering = ordering;
  }

  ElapsedTime timing(linearSolveSeconds);
  if (gaussian) {
    delta = state.values.zeroVectors();
    try {
      if (std::any_of(delta.begin(), delta.end(),
                      [](const auto& item) { return item.second.size() != 0; })) {
        const auto diagonal =
            SqrtHessianDiagonal(*state.linearPointGraph, lmParams);
        const auto damped = buildDampedSystem(
            state.reducedObjective, diagonal, state, lmParams);
        ++gaussianSolves;
        const auto solution = SolveLinear(damped, lmParams);
        for (auto& [key, value] : delta) {
          if (value.size()) value = solution.at(key);
          if (!value.allFinite()) {
            failureReason = "Gaussian solve returned a nonfinite increment";
            return;
          }
        }
      }
    } catch (const IndeterminateSystemException& error) {
      failureReason = error.what();
      return;
    } catch (const std::runtime_error& error) {
      failureReason = error.what();
      return;
    }
    solveSuccessful = true;
    recordSolution(state, {});
    return;
  }

  if (params.qpBackend == IELMParams::QpBackend::ProxQP) {
#ifndef GTDYNAMICS_WITH_PROXQP
    throw std::invalid_argument(
        "ProxQP backend requires a build with GTDYNAMICS_WITH_PROXQP=ON");
#endif
  }
  if (params.qpBackend != IELMParams::QpBackend::Gtsam &&
      (params.qpUseFeasibleStep ||
       params.qpUseProjectedGradientInitialization))
    throw std::invalid_argument(
        "ProxQP and ADMM require full QP solves and no "
        "projected-gradient initialization");
  const auto &linear = *state.linearPointGraph;
  const auto sqrtHessianDiagonal =
      SqrtHessianDiagonal(linear, params.lmParams);
  QpProblem problem = state.tangentQp;
  // Equality-only states do not pack a QP unless explicitly requested.
  if (state.equalityOnly)
    for (const auto& factor : state.reducedObjective) problem.addCost(factor);
  // Only damping changes across trials; the undamped QP belongs to the state.
  for (const auto &factor :
       buildDampedSystem({}, sqrtHessianDiagonal, state, params.lmParams))
    problem.addCost(factor);

  delta = state.values.zeroVectors();
  Values initial;
  for (const auto &[key, value] : delta)
    if (value.size()) initial.insert(key, value);
  IndexSetMap blockingIndices;
  if (!initial.empty()) {
    auto qpParams = std::make_shared<ActiveSetSolverParams>();
    qpParams->qpSubproblemSolver =
        ActiveSetSolverParams::QpSubproblemSolver::Sparse;
    qpParams->regularization = 0;
    qpParams->maxIterations = params.qpMaxIterations;
    qpParams->qpInitialization =
        params.qpUseProjectedGradientInitialization
            ? gtsam::ActiveSetSolverParams::QpInitialization::ProjectedGradient
            : gtsam::ActiveSetSolverParams::QpInitialization::TightConstraints;
    if (lmParams.ordering) qpParams->setOrdering(*lmParams.ordering);
    try {
      ++qpSolves;
      Values solution;
      std::vector<bool> activeRows;
      if (params.qpBackend == IELMParams::QpBackend::ADMM) {
        internal::AdmmParams admm;
        admm.rho = params.admmRho;
        admm.useDiagonalMetric = params.admmUseDiagonalMetric;
        admm.splitting = params.admmSplitting;
        admm.absoluteTolerance = params.admmAbsoluteTolerance;
        admm.relativeTolerance = params.admmRelativeTolerance;
        admm.maxProjectionSweeps = params.admmProjectionSweeps;
        admm.projectionThreads = params.admmProjectionThreads;
        internal::AdmmWorkspace localWorkspace;
        auto &workspace = admmWorkspace ? *admmWorkspace : localWorkspace;
        const auto result = workspace.solve(
            problem, initial, qpParams->ordering, params.qpMaxIterations, admm);
        qpIterations = result.iterations;
        if (!result.converged) {
          failureReason = result.message;
          return;
        }
        solution = result.values;
        activeRows = result.activeRows;
      }
#ifdef GTDYNAMICS_WITH_PROXQP
      else if (params.qpBackend == IELMParams::QpBackend::ProxQP) {
        const auto result = internal::solveProxQp(
            problem, initial, qpParams->ordering, params.qpMaxIterations);
        qpIterations = result.iterations;
        if (!result.converged) {
          failureReason = result.message;
          return;
        }
        solution = result.values;
        activeRows = result.activeRows;
      }
#endif
      else {
        const ActiveSetSolver solver(problem, qpParams);
        if (params.qpUseFeasibleStep) {
          const auto result = solver.computeFeasibleStep(initial);
          qpIterations = result.iterations;
          using Status = ActiveSetFeasibleStepResult::Status;
          if (result.status != Status::FeasibleProgress &&
              result.status != Status::NoProgress) {
            failureReason = result.message;
            return;
          }
          solution = result.values;
          activeRows = result.activeInequalityRows;
        } else {
          const auto [values, result] = solver.optimizeWithState(initial);
          qpIterations = result.iterations;
          if (!result.converged) {
            failureReason = "Sparse QP did not converge";
            return;
          }
          solution = values;
          activeRows = result.activeInequalityRows;
        }
      }
      for (auto &[key, value] : delta) {
        if (value.size()) value = solution.at<Vector>(key);
        if (!value.allFinite()) {
          failureReason = "Sparse QP returned a nonfinite increment";
          return;
        }
      }
      for (const auto &block : state.qpConstraintBlocks) {
        const auto begin = activeRows.begin() + block.firstRow;
        // Retraction promotes whole blocks only. Partially active vector
        // constraints remain inequalities, as in the existing cone projection.
        if (std::all_of(begin, begin + block.numRows,
                        [](bool active) { return active; }))
          blockingIndices[block.component].insert(block.constraint);
      }
    } catch (const IndeterminateSystemException &error) {
      failureReason = error.what();
      return;
    } catch (const std::runtime_error &error) {
      // Includes iteration exhaustion. Invalid inputs still propagate.
      failureReason = error.what();
      return;
    }
  }
  solveSuccessful = true;
  recordSolution(state, blockingIndices);
}

/* ************************************************************************* */
void IELMTrial::LinearUpdate::recordSolution(
    const IELMState &state, const IndexSetMap &blockingIndices) {
  // Retain original constraint indices while lifting to ambient tangents for
  // retraction, then evaluate the undamped model's predicted reduction.
  blockingIndicesMap = blockingIndices;
  tangentVector = computeTangentVector(state, delta, state.unconstrainedKeys);
  if (!checkSolutionValid(state, delta, blockingIndices)) {
    solveSuccessful = false;
    failureReason = "Reduced step failed directional feasibility checks";
    return;
  }
  const auto &linear = *state.linearPointGraph;
  oldError = linear.error(VectorValues::Zero(delta));
  newError = linear.error(delta);
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
bool IELMTrial::LinearUpdate::checkSolutionValid(
    const IELMState &state, const VectorValues &delta,
    const IndexSetMap &blockingIndices) {
  for (const auto &[key, point] : state.points) {
    const auto &local = *point.tangentSpace();
    if (local.indices().empty()) continue;
    if (!local.violated(delta.at(key)).empty()) return false;
    const auto found = blockingIndices.find(key);
    if (found != blockingIndices.end() &&
        !local.active(delta.at(key), found->second))
      return false;
  }
  return true;
}

/* ************************************************************************* */
LMCachedModel *IELMTrial::LinearUpdate::getCachedModel(size_t dim) const {
  if (dim >= noiseModelCache.size()) noiseModelCache.resize(dim + 1);
  LMCachedModel *item = &noiseModelCache[dim];
  if (!item->model) *item = LMCachedModel(dim, 1.0 / std::sqrt(lambda));
  return item;
}

/* ************************************************************************* */
GaussianFactorGraph IELMTrial::LinearUpdate::buildDampedSystem(
    GaussianFactorGraph damped, const IELMState &state) const {
  noiseModelCache.resize(0);
  // for each of the variables, add a prior
  damped.reserve(damped.size() + state.values.size());
  std::map<Key, size_t> dims = state.values.dims();
  for (const auto &key_dim : dims) {
    const Key &key = key_dim.first;
    const size_t &dim = key_dim.second;
    if (!dim) continue;
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
      if (!dim) continue;
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
      const auto &point = state.points.at(key);
      VectorValues tv = point.toAmbient(xi);
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
                                            const NonlinearFactorGraph &graph,
                                            size_t retractionThreads) {
  // Each retractor may update its own cached ConstVarFactors. A worker owns
  // exactly one component; map insertion, diagnostics and delta assembly stay
  // ordered on this thread after all workers have completed successfully.
  std::optional<VectorValues> metric;
  for (const auto &[key, point] : state.points)
    if (point.manifold()->usesVaryingMetric()) {
      metric = state.computeMetricSigmas(graph);
      break;
    }
  const KeyVector keys = pointKeys(state.points);
  std::vector<std::optional<IECMPoint>> projected(keys.size());
  std::vector<IERetractionInfo> information(keys.size());
  internal::forIndependentComponents(
      keys.size(), retractionThreads, [&](size_t i) {
        const Key key = keys[i];
        const auto &point = state.points.at(key);
        const auto tangent =
            SubValues(linearUpdate.tangentVector, point.values().keys());
        const auto blocking = linearUpdate.blockingIndicesMap.find(key);
        projected[i] =
            point.retract(tangent,
                             blocking == linearUpdate.blockingIndicesMap.end()
                                 ? std::optional<IndexSet>{}
                                 : blocking->second,
                             &information[i],
            metric && point.manifold()->usesVaryingMetric() ? &*metric
                                                               : nullptr);
      });
  numRetractionIterations = 0;
  VectorValues retract_delta;
  for (size_t i = 0; i < keys.size(); ++i) {
    const Key key = keys[i];
    const auto &point = state.points.at(key);
    newPoints.emplace(key, std::move(*projected[i]));
    numRetractionIterations += information[i].numLMIterations;
    const auto tangent =
        SubValues(linearUpdate.tangentVector, point.values().keys());
    auto [difference, deviation] =
        evaluateRetractionDeviation(point, newPoints.at(key), tangent);
    retractionDeviationRates.push_back(deviation);
    retract_delta.insert(difference);
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
  std::optional<VectorValues> metric;
  for (const auto &[key, point] : state.points)
    if (point.manifold()->usesVaryingMetric()) {
      metric = state.computeMetricSigmas(graph);
      break;
    }
  newPoints = movePointsToBoundaries(state.points, forcedIndicesMap,
                                     metric ? &*metric : nullptr);
  newUnconstrainedValues = state.unconstrainedValues();
  computeError(graph, state.error);
}

/* ************************************************************************* */
std::pair<VectorValues, double>
IELMTrial::NonlinearUpdate::evaluateRetractionDeviation(
    const IECMPoint &point,
    const IECMPoint &newPoint,
    const VectorValues &tangentVector) {
  VectorValues retract_delta =
      point.values().localCoordinates(newPoint.values());
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

  newError = IELMState::evaluateGraphError(graph, newPoints,
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
             << ",qp_iterations"
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
                 << trial.linearUpdate.qpIterations << ","
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
