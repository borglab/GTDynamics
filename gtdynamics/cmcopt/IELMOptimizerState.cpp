
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/IELMOptimizerState.h>
#include <gtdynamics/optimizer/ConvexIQPSolver.h>
#include <gtdynamics/utils/GraphUtils.h>

using std::cout, std::setw, std::setprecision, std::endl;

namespace gtsam {

/* ************************************************************************* */
/* <============================ IELMState ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
IELMState::IELMState(const IEManifoldValues &_manifolds,
                     const Values &unconstrained_values,
                     const NonlinearFactorGraph &graph,
                     const NonlinearFactorGraph &manifold_graph,
                     const double &_lambda, const double &_lambda_factor,
                     size_t _iterations)
    : manifolds(_manifolds),
      values(AllValues(_manifolds, unconstrained_values)),
      unconstrained_keys(unconstrained_values.keys()),
      error(EvaluateGraphError(graph, manifolds, unconstrained_values)),
      lambda(_lambda), lambda_factor(_lambda_factor), iterations(_iterations) {
  construct(graph, manifold_graph);
}

/* ************************************************************************* */
IELMState
IELMState::FromLastIteration(const IELMIterDetails &iter_details,
                             const NonlinearFactorGraph &graph,
                             const NonlinearFactorGraph &manifold_graph,
                             const LevenbergMarquardtParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  IELMState state;
  if (last_trial.step_is_successful) {
    state = IELMState(last_trial.nonlinear_update.new_manifolds,
                      last_trial.nonlinear_update.new_unconstrained_values,
                      graph, manifold_graph, last_trial.linear_update.lambda,
                      prev_state.lambda_factor);
    // TODO: will this cause early ending? (converged in this mode, but not for
    // the overall problem)
    if (last_trial.forced_indices_map.size() > 0) {
      state.grad_blocking_indices_map.mergeWith(last_trial.forced_indices_map);
    }
  } else {
    // pick the trials with smallest error
    state =
        IELMState(prev_state.manifolds, prev_state.unconstrainedValues(), graph,
                  manifold_graph, prev_state.lambda, prev_state.lambda_factor);
    for (const auto &trial : iter_details.trials) {
      if (trial.linear_update.solve_successful &&
          trial.nonlinear_update.new_error < prev_state.error) {
        state = IELMState(trial.nonlinear_update.new_manifolds,
                          trial.nonlinear_update.new_unconstrained_values,
                          graph, manifold_graph, trial.linear_update.lambda,
                          prev_state.lambda_factor);
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
Values IELMState::AllValues(const IEManifoldValues &manifolds,
                            const Values &unconstrained_values) {
  Values values = IEOptimizer::EManifolds(manifolds);
  values.insert(unconstrained_values);
  return values;
}

/* ************************************************************************* */
void IELMState::construct(const NonlinearFactorGraph &graph,
                          const NonlinearFactorGraph &manifold_graph) {
  // linearize costs
  base_linear = graph.linearize(baseValues());
  linear_manifold_graph = manifold_graph.linearize(values);

  // linearize active i-constraints
  linearizeIConstraints();

  // compute graident
  computeGradient(manifold_graph);
}

/* ************************************************************************* */
void IELMState::linearizeIConstraints() {
  linear_manifold_i_constraints.resize(0);
  size_t index = 0;
  for (const auto &[key, manifold] : manifolds) {
    auto man_constraints = manifold.linearActiveManIConstraints(key);
    auto base_constraints = manifold.linearActiveBaseIConstraints();
    for (const auto &[constraint_idx, constraint] : man_constraints) {
      linear_manifold_i_constraints.push_back(constraint);
      linear_base_i_constraints.push_back(base_constraints.at(constraint_idx));
      lic_index_translator.insert(index, key, constraint_idx);
      index++;
    }
  }
}

/* ************************************************************************* */
void IELMState::computeGradient(const NonlinearFactorGraph &manifold_graph) {
  gradient = linear_manifold_graph->gradientAtZero();
  VectorValues descent_dir = -1 * gradient;

  // identify blocking constraints
  grad_blocking_indices_map =
      IEOptimizer::ProjectTangentCone(manifolds, descent_dir).first;
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
Values IELMState::unconstrainedValues() const {
  return SubValues(values, unconstrained_keys);
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
  // hessian_diag.print("hessian diag:", gtdynamics::GTDKeyFormatter);
  VectorValues metric_sigmas;
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
                     const double &lambda, const IELMParams &params) {
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
      model_fidelity > params.lm_params.minModelFidelity) {
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
  auto start = std::chrono::high_resolution_clock::now();

  forced_indices_map = approach_indices_map;
  linear_update = LinearUpdate::Zero(state);
  nonlinear_update = NonlinearUpdate(state, forced_indices_map, graph);
  step_is_successful = true;

  auto end = std::chrono::high_resolution_clock::now();
  trial_time =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1e6;
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
/* <============================= Logging =================================> */
/* ************************************************************************* */

/* ************************************************************************* */
std::map<std::pair<Key, size_t>, size_t>
IdentifyConstraintType(const IEManifoldValues &state_manifolds,
                       const IndexSetMap blocking_indices_map,
                       const IEManifoldValues &new_manifolds) {
  std::map<std::pair<Key, size_t>, size_t> constraint_type_map;

  for (const auto &[key, new_manifold] : new_manifolds) {
    const auto &state_manifold = state_manifolds.at(key);
    IndexSet blocking_indices;
    if (blocking_indices_map.exists(key)) {
      blocking_indices = blocking_indices_map.at(key);
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
                              const IndexSetMap blocking_indices_map,
                              const IEManifoldValues &new_manifolds,
                              const KeyFormatter &key_formatter,
                              bool step_is_successful,
                              bool group_as_categories) {

  std::string default_color_str = step_is_successful ? "\033[0m" : "\033[090m";

  auto constraint_type_map = IdentifyConstraintType(
      state_manifolds, blocking_indices_map, new_manifolds);

  if (!group_as_categories) {
    std::string str = "";
    for (const auto &[key, manifold] : new_manifolds) {
      for (const auto &constraint_idx : manifold.activeIndices()) {
        const auto &constraint = manifold.iConstraints()->at(constraint_idx);
        auto constraint_type = constraint_type_map.at({key, constraint_idx});
        str += ColoredStr(" " + constraint->name_tmp(), constraint_type,
                          default_color_str);
      }
    }
    return str;
  }

  std::map<std::string, std::vector<std::pair<size_t, size_t>>>
      category_constraints;

  for (const auto &[key, manifold] : new_manifolds) {
    for (const auto &constraint_idx : manifold.activeIndices()) {
      const auto &constraint = manifold.iConstraints()->at(constraint_idx);
      auto [category, k] = SplitStr(constraint->name_tmp());
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
void PrintIELMTrialTitle() {
  cout << setw(10) << "iter   "
       << "|" << setw(12) << "error  "
       << "|" << setw(12) << "nonlinear "
       << "|" << setw(12) << "linear   "
       << "|" << setw(12) << "linear_retr"
       << "|" << setw(10) << "lambda  "
       << "|" << setw(10) << "num_solves"
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
    const auto &manifold = manifolds.at(key);
    for (const auto &i_idx : index_set) {
      const auto &constraint = manifold.iConstraints()->at(i_idx);
      str += " " + constraint->name_tmp();
    }
  }
  return str;
}

/* ************************************************************************* */
void PrintIELMTrial(const IELMState &state, const IELMTrial &trial,
                    const IELMParams &params, bool forced,
                    const KeyFormatter &key_formatter) {
  if (!trial.step_is_successful) {
    cout << "\033[90m";
  }
  const auto &nonlinear_update = trial.nonlinear_update;
  const auto &linear_update = trial.linear_update;
  cout << setw(10) << state.iterations << "|";
  cout << setw(12) << setprecision(4) << nonlinear_update.new_error << "|";
  cout << setw(12) << setprecision(4) << nonlinear_update.cost_change << "|";
  if (!forced) {
    cout << setw(12) << setprecision(4) << linear_update.cost_change << "|";
    cout << setw(12) << setprecision(4)
         << nonlinear_update.linear_cost_change_with_retract_delta << "|";
    cout << setw(10) << setprecision(2) << linear_update.lambda << "|";
    if (!linear_update.solve_successful) {
      cout << "linear solve not successful\n";
      return;
    }
    cout << setw(4) << linear_update.num_solves << "|" << setw(5) << std::left
         << nonlinear_update.num_retract_iters << std::right << "|";
    cout << setw(8) << VectorMean(nonlinear_update.retract_divate_rates) << "|"
         << setw(8) << std::left
         << VectorMax(nonlinear_update.retract_divate_rates) << std::right
         << "|";
    cout << setw(10) << setprecision(2) << trial.trial_time << "|";
    cout << setw(10) << setprecision(4) << linear_update.delta.norm() << "|";
    if (params.show_active_constraints) {
      cout << ConstraintInfoStr(
          state.manifolds, linear_update.blocking_indices_map,
          nonlinear_update.new_manifolds, gtdynamics::GTDKeyFormatter,
          trial.step_is_successful,
          params.active_constraints_group_as_categories);
    }
    // cout << setw(10) << setprecision(4) <<
    // linear_update.tangent_vector.norm()
    //      << " ";
    // cout << setw(10) << setprecision(4)
    //      << linear_update.tangent_vector.norm() / linear_update.delta.norm()
    //      << " ";
  } else {
    std::string forced_i_str =
        "forced:" + ConstraintInfoStr(state.manifolds, trial.forced_indices_map,
                                      key_formatter);
    cout << forced_i_str;
  }
  if (!trial.step_is_successful) {
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
    : lambda(_lambda), num_solves(0) {

  // build damped system
  GaussianFactorGraph::shared_ptr linear = state.linear_manifold_graph;
  VectorValues sqrt_hessian_diagonal =
      SqrtHessianDiagonal(*linear, params.lm_params);
  auto damped_system = buildDampedSystem(*linear, sqrt_hessian_diagonal, state,
                                         params.lm_params);
  IndexSet blocking_indices;

  // no active constraints
  if (state.linear_base_i_constraints.size() == 0) {
    try {
      delta = SolveLinear(damped_system, params.lm_params);
      solve_successful = true;
    } catch (const IndeterminantLinearSystemException &) {
      solve_successful = false;
    }
    num_solves = 1;
  } else {
    // solve IQP init estimate
    std::tie(delta, blocking_indices, num_solves, solve_successful) =
        InitEstimate(damped_system, state, params.lm_params);
    if (!solve_successful) {
      return;
    }

    // solve IQP
    if (params.iqp_max_iters > 0) {
      size_t num_new_solves;
      std::tie(delta, blocking_indices, num_new_solves, solve_successful) =
          SolveConvexIQP(damped_system, state.linear_manifold_i_constraints,
                         blocking_indices, delta, params.iqp_max_iters);
      num_solves += num_new_solves;
    }
  }

  // record linear update info
  blocking_indices_map =
      state.lic_index_translator.decodeIndices(blocking_indices);
  tangent_vector = computeTangentVector(state, delta, state.unconstrained_keys);
  CheckSolutionValid(state, tangent_vector, blocking_indices);
  old_error = linear->error(VectorValues::Zero(delta));
  new_error = linear->error(delta);
  cost_change = old_error - new_error;
}

/* ************************************************************************* */
IELMTrial::LinearUpdate IELMTrial::LinearUpdate::Zero(const IELMState &state) {
  LinearUpdate linear_update;
  linear_update.lambda = state.lambda;
  linear_update.delta = state.values.zeroVectors();
  linear_update.tangent_vector = state.baseValues().zeroVectors();
  return linear_update;
}

/* ************************************************************************* */
std::tuple<VectorValues, IndexSet, size_t, bool>
IELMTrial::LinearUpdate::InitEstimate(const GaussianFactorGraph &quadratic_cost,
                                      const IELMState &state,
                                      const LevenbergMarquardtParams &params) {

  IndexSet blocking_indices =
      state.lic_index_translator.encodeIndices(state.grad_blocking_indices_map);
  size_t num_solves = 0;

  while (true) {
    num_solves += 1;
    GaussianFactorGraph graph = quadratic_cost;
    GaussianFactorGraph constraint_graph =
        state.linear_manifold_i_constraints.constraintGraph(blocking_indices);
    graph.push_back(constraint_graph.begin(), constraint_graph.end());

    VectorValues delta;
    try {
      delta = SolveLinear(graph, params);
    } catch (const IndeterminantLinearSystemException &) {
      return {delta, blocking_indices, num_solves, false};
    }

    // check if satisfy tagent cone, if not, add constraints and recompute
    bool feasible = true;
    for (const auto &[key, manifold] : state.manifolds) {
      const Vector &xi = delta.at(key);
      VectorValues tv = manifold.eBasis()->computeTangentVector(xi);

      IndexSet man_blocking_indices = manifold.blockingIndices(tv);
      for (const auto &constraint_idx : man_blocking_indices) {
        size_t index =
            state.lic_index_translator.encoder.at({key, constraint_idx});
        if (!blocking_indices.exists(index)) {
          feasible = false;
          blocking_indices.insert(index);
        }
      }
    }
    if (feasible) {
      return {delta, blocking_indices, num_solves, true};
    }
  }
}

/* ************************************************************************* */
bool IELMTrial::LinearUpdate::CheckSolutionValid(
    const IELMState &state, const VectorValues &tangent_vector,
    const IndexSet &blocking_indices) {
  for (const auto &constraint : state.linear_base_i_constraints) {
    if (!constraint->feasible(tangent_vector, 1e-5)) {
      std::cout << "tangent vector violating constraint: "
                << (*constraint)(tangent_vector).transpose() << "\n";
      return false;
    }
  }
  for (const auto &constraint_idx : blocking_indices) {
    const auto &constraint = state.linear_base_i_constraints.at(constraint_idx);
    if (!constraint->isActive(tangent_vector)) {
      std::cout << "blocking constraint is not active: "
                << (*constraint)(tangent_vector).transpose() << "\n";
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

  if (params.diagonalDamping)
    return buildDampedSystem(linear, sqrtHessianDiagonal);
  else
    return buildDampedSystem(linear, state);
}

/* ************************************************************************* */
VectorValues IELMTrial::LinearUpdate::computeTangentVector(
    const IELMState &state, const VectorValues &delta,
    const KeySet &unconstrained_keys) const {
  VectorValues tangent_vector;
  for (const auto &[key, xi] : delta) {
    if (unconstrained_keys.exists(key)) {
      tangent_vector.insert(key, delta.at(key));
    } else {
      const auto &manifold = state.manifolds.at(key);
      VectorValues tv = manifold.eBasis()->computeTangentVector(xi);
      tangent_vector.insert(tv);
    }
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

  // retract for ie-manifolds
  num_retract_iters = 0;
  VectorValues retract_delta;
  for (const auto &[key, manifold] : state.manifolds) {
    VectorValues tv =
        SubValues(linear_update.tangent_vector, manifold.values().keys());
    const auto &blocking_indices_map = linear_update.blocking_indices_map;
    IERetractInfo retract_info;
    if (blocking_indices_map.find(key) != blocking_indices_map.end()) {
      const auto &blocking_indices = blocking_indices_map.at(key);
      new_manifolds.emplace(
          key, manifold.retract(tv, blocking_indices, &retract_info));
    } else {
      new_manifolds.emplace(key, manifold.retract(tv, {}, &retract_info));
    }
    num_retract_iters += retract_info.num_lm_iters;
    auto [manifold_retract_delta, retract_deviation_rate] =
        evaluateRetractionDeviation(manifold, new_manifolds.at(key), tv);
    retract_divate_rates.push_back(retract_deviation_rate);
    retract_delta.insert(manifold_retract_delta);
  }

  // retract for unconstrained variables
  VectorValues tangent_vector_unconstrained =
      SubValues(linear_update.tangent_vector, state.unconstrained_keys);
  new_unconstrained_values =
      state.unconstrainedValues().retract(tangent_vector_unconstrained);
  retract_delta.insert(tangent_vector_unconstrained);

  // compute error
  computeError(graph, state.error);
  VectorValues zero_vec = VectorValues::Zero(retract_delta);
  // KeyVector zero_vec_kv;
  // for (const auto &[key, v] : zero_vec) {
  //   zero_vec_kv.push_back(key);
  // }
  // PrintKeyVector(zero_vec_kv, "zero_vec", gtdynamics::GTDKeyFormatter);
  // PrintKeySet(state.base_linear->keys(), "graph",
  // gtdynamics::GTDKeyFormatter);
  double base_linear_error = state.base_linear->error(zero_vec);
  double base_linear_error_retract = state.base_linear->error(retract_delta);
  linear_cost_change_with_retract_delta =
      base_linear_error - base_linear_error_retract;
}

/* ************************************************************************* */
IELMTrial::NonlinearUpdate::NonlinearUpdate(
    const IELMState &state, const IndexSetMap &forced_indices_map,
    const NonlinearFactorGraph &graph) {
  new_manifolds = state.manifolds.moveToBoundaries(forced_indices_map);
  new_unconstrained_values = state.unconstrainedValues();
  computeError(graph, state.error);
}

/* ************************************************************************* */
std::pair<VectorValues, double>
IELMTrial::NonlinearUpdate::evaluateRetractionDeviation(
    const IEConstraintManifold &manifold,
    const IEConstraintManifold &new_manifold,
    const VectorValues &tangent_vector) {
  VectorValues retract_delta =
      manifold.values().localCoordinates(new_manifold.values());
  auto vec_diff = retract_delta - tangent_vector;
  double tangent_vector_norm = tangent_vector.norm();
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
             << ",lambda"
             << ",error"
             << "\n";
  trial_file << "iterations"
             << ",lambda"
             << ",error"
             << ",step_is_successful"
             << ",linear_cost_change"
             << ",linear_cost_change_with_retract_delta"
             << ",nonlinear_cost_change"
             << ",model_fidelity"
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
      trial_file << state.iterations << "," << trial.linear_update.lambda << ","
                 << trial.nonlinear_update.new_error << ","
                 << trial.step_is_successful << ","
                 << trial.linear_update.cost_change << ","
                 << trial.nonlinear_update.linear_cost_change_with_retract_delta
                 << "," << trial.nonlinear_update.cost_change << ","
                 << trial.model_fidelity << ","
                 << trial.linear_update.tangent_vector.norm() << ","
                 << trial.linear_update.num_solves << ","
                 << trial.nonlinear_update.num_retract_iters << ","
                 << VectorMean(trial.nonlinear_update.retract_divate_rates)
                 << ","
                 << VectorMax(trial.nonlinear_update.retract_divate_rates)
                 << "\n";
    }
  }
  state_file.close();
  trial_file.close();
}

} // namespace gtsam
