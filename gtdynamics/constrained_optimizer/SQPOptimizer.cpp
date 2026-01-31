/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SQPOptimizer.cpp
 * @brief SQP implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/SQPOptimizer.h>

using namespace gtdynamics;
using std::cout, std::setprecision, std::setw, std::endl;

namespace gtsam {

/* ************************************************************************* */
/* <============================= SQPState ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
SQPState::SQPState(const Values &_values, const NonlinearFactorGraph &graph,
                   const EqualityConstraints &e_constraints,
                   const InequalityConstraints &i_constraints,
                   const SQPParams &params, const double _lambda,
                   const double _lambda_factor, const size_t _iterations)
    : lambda(_lambda), lambda_factor(_lambda_factor), values(_values),
      eval(SQPOptimizer::MeritFunction(graph, e_constraints, i_constraints,
                                       values, params)),
      iterations(_iterations), totalNumberInnerIterations(0) {
  linear_cost = *graph.linearize(values);
  linear_e_merit = *e_constraints.meritGraph().linearize(values);
  linear_e_constraints = *e_constraints.constrainedGraph().linearize(values);
  linear_i_constraints = GaussianFactorGraph();
  linear_i_merit = GaussianFactorGraph();
  for (const auto &i_constraint : i_constraints) {
    if (!i_constraint->feasible(values)) {
      auto e_constraint = i_constraint->createEqualityConstraint();
      linear_i_constraints.push_back(
          e_constraint->createConstrainedFactor()->linearize(values));
      linear_i_merit.push_back(
          e_constraint->createFactor(1.0)->linearize(values));
    }
  }
  computeMinVector();
}

/* ************************************************************************* */
SQPState SQPState::FromLastIteration(const SQPIterDetails &iter_details,
                                     const NonlinearFactorGraph &graph,
                                     const EqualityConstraints &e_constraints,
                                     const InequalityConstraints &i_constraints,
                                     const SQPParams &params) {
  double lambda;
  const auto &last_trial = iter_details.trials.back();
  const auto &prev_state = iter_details.state;
  SQPState state;
  if (last_trial.step_is_successful) {
    auto state = SQPState(last_trial.new_values, graph, e_constraints,
                          i_constraints, params, last_trial.lambda,
                          prev_state.lambda_factor, prev_state.iterations + 1);
    state.totalNumberInnerIterations =
        prev_state.totalNumberInnerIterations + iter_details.trials.size();
    last_trial.setNextLambda(state.lambda, state.lambda_factor,
                             params.lm_params);
    return state;
  } else {
    throw std::runtime_error("no success trials");
  }
}

/* ************************************************************************* */
void SQPState::computeMinVector() {
  GaussianFactorGraph graph;
  graph.push_back(linear_e_constraints);
  graph.push_back(linear_i_constraints);
  for (const auto &key : values.keys()) {
    size_t dim = values.at(key).dim();
    graph.emplace_shared<JacobianFactor>(key, Matrix::Identity(dim, dim),
                                         Vector::Zero(dim),
                                         noiseModel::Unit::Create(dim));
  }
  min_vector = graph.optimize();
}

/* ************************************************************************* */
/* <============================= SQPTrial ================================> */
/* ************************************************************************* */

/* ************************************************************************* */
SQPTrial::SQPTrial(const SQPState &state, const double _lambda,
                   const NonlinearFactorGraph &graph,
                   const EqualityConstraints &e_constraints,
                   const InequalityConstraints &i_constraints,
                   const SQPParams &params)
    : lambda(_lambda) {
  // build damped system
  GaussianFactorGraph qp_problem = constructConstrainedSystem(state, params);
  GaussianFactorGraph damped_system =
      buildDampedSystem(qp_problem, state, params.lm_params);
  // damped_system.print();

  // solve linear update
  try {
    // delta = damped_system.optimize();
    // GaussianFactorGraph new_system;
    // for (auto factor : damped_system) {
    //   JacobianFactor::shared_ptr jacobian_factor(
    //       std::dynamic_pointer_cast<JacobianFactor>(factor));
    //   // if (jacobian_factor) {
    //   auto noise_model = jacobian_factor->get_model();
    //   if (noise_model && noise_model->isConstrained()) {
    //     new_system.push_back(ZerobFactor(jacobian_factor));
    //   } else {
    //     new_system.push_back(factor);
    //   }
    // }
    // for (const auto& key: state.values.keys()) {
    //   size_t dim = state.values.at(key).dim();
    //   new_system.emplace_shared<JacobianFactor>(key, Matrix::Identity(dim,
    //   dim)*1e3, Vector::Ones(dim), noiseModel::Unit::Create(dim));
    // }
    delta = SolveLinear(damped_system, params.lm_params);
    // auto delta1 = SolveLinear(new_system, params.lm_params);
    // // delta = damped_system.optimize();
    // // auto delta1 = new_system.optimize();
    // std::cout << "linear error zero: " <<
    // damped_system.error(VectorValues::Zero(delta)) << "\n"; std::cout <<
    // "linear error delta: " << damped_system.error(delta) << "\n"; std::cout
    // << "linear error delta1: " << damped_system.error(delta1) << "\n";

    // std::cout << "new linear error delta1: " << new_system.error(delta1) <<
    // "\n"; std::cout << "new linear error zero: " <<
    // new_system.error(VectorValues::Zero(delta)) << "\n"; std::cout <<
    // "delta1_norm: " << delta1.norm() << "\n";
    solve_successful = true;
  } catch (const IndeterminantLinearSystemException &) {
    return;
  }
  VectorValues zero_delta = VectorValues::Zero(delta);
  auto linear_eval_zero =
      SQPOptimizer::MeritFunctionApprox(state, zero_delta, params);
  auto linear_eval_delta =
      SQPOptimizer::MeritFunctionApprox(state, delta, params);
  linear_merit_change = linear_eval_zero.merit - linear_eval_delta.merit;
  bound_rate = delta.norm() / state.min_vector.norm();
  if (linear_merit_change < 0 && bound_rate < 1.1) {
    resolveLinearUsingMeritSystem(state, params);
  }

  // apply nonlinear update
  new_values = state.values.retract(delta);
  eval = SQPOptimizer::MeritFunction(graph, e_constraints, i_constraints,
                                     new_values, params);
  nonlinear_merit_change = state.eval.merit - eval.merit;

  // model fidelity
  if (linear_merit_change > 0) {
    model_fidelity = nonlinear_merit_change / linear_merit_change;
  } else if (nonlinear_merit_change > 0) {
    model_fidelity = 1;
  } else {
    model_fidelity = 0;
  }

  if (model_fidelity > params.lm_params.minModelFidelity) {
    step_is_successful = true;
  }
}

/* ************************************************************************* */
GaussianFactorGraph
SQPTrial::constructConstrainedSystem(const SQPState &state,
                                     const SQPParams &params) const {
  GaussianFactorGraph graph = state.linear_cost;
  if (params.use_qp_constrained_mu) {
    graph.push_back(
        ScaledBiasedFactors(state.linear_e_merit, params.qp_constrained_mu, 1));
    graph.push_back(
        ScaledBiasedFactors(state.linear_i_merit, params.qp_constrained_mu, 1));
  } else {
    graph.push_back(state.linear_e_constraints);
    graph.push_back(state.linear_i_constraints);
  }
  return graph;
}

/* ************************************************************************* */
GaussianFactorGraph
SQPTrial::constructMeritSystem(const SQPState &state,
                               const SQPParams &params) const {
  GaussianFactorGraph graph = state.linear_cost;
  VectorValues zero_vec = state.values.zeroVectors();
  double e_b_norm = sqrt(2 * state.linear_e_merit.error(zero_vec));
  double e_b_scale = 1 + params.merit_e_l1_mu / params.merit_e_l2_mu / e_b_norm;
  graph.push_back(ScaledBiasedFactors(state.linear_e_merit,
                                      params.merit_e_l2_mu, e_b_scale));
  double i_b_norm = sqrt(2 * state.linear_i_merit.error(zero_vec));
  double i_b_scale = 1 + params.merit_i_l1_mu / params.merit_i_l2_mu / i_b_norm;
  graph.push_back(ScaledBiasedFactors(state.linear_i_merit,
                                      params.merit_i_l2_mu, i_b_scale));
  return graph;
}

/* ************************************************************************* */
void SQPTrial::resolveLinearUsingMeritSystem(const SQPState &state,
                                             const SQPParams &params) {
  use_merit_system = true;
  GaussianFactorGraph qp_problem = constructMeritSystem(state, params);
  GaussianFactorGraph damped_system =
      buildDampedSystem(qp_problem, state, params.lm_params);

  // solve linear update
  try {
    delta = SolveLinear(damped_system, params.lm_params);
    solve_successful = true;
  } catch (const IndeterminantLinearSystemException &) {
    solve_successful = false;
    return;
  }
  VectorValues zero_delta = VectorValues::Zero(delta);
  auto linear_eval_zero =
      SQPOptimizer::MeritFunctionApprox(state, zero_delta, params);
  auto linear_eval_delta =
      SQPOptimizer::MeritFunctionApprox(state, delta, params);
  linear_eval_zero.merit = qp_problem.error(zero_delta);
  linear_eval_delta.merit = qp_problem.error(delta);
  linear_merit_change = linear_eval_zero.merit - linear_eval_delta.merit;
}

/* ************************************************************************* */
LMCachedModel *SQPTrial::getCachedModel(size_t dim) const {
  if (dim >= noiseModelCache.size())
    noiseModelCache.resize(dim + 1);
  LMCachedModel *item = &noiseModelCache[dim];
  if (!item->model)
    *item = LMCachedModel(dim, 1.0 / std::sqrt(lambda));
  return item;
}

/* ************************************************************************* */
GaussianFactorGraph
SQPTrial::buildDampedSystemUniform(GaussianFactorGraph damped,
                                   const SQPState &state) const {
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
GaussianFactorGraph SQPTrial::buildDampedSystemDiagonal(
    GaussianFactorGraph damped, // gets copied
    const SQPState &state, const LevenbergMarquardtParams &lm_params) const {
  VectorValues sqrt_hessian_diagonal =
      SqrtHessianDiagonal(state.linear_cost, lm_params);
  noiseModelCache.resize(0);
  damped.reserve(damped.size() + sqrt_hessian_diagonal.size());
  for (const auto &key_vector : sqrt_hessian_diagonal) {
    try {
      const Key key = key_vector.first;
      const size_t dim = key_vector.second.size();
      LMCachedModel *item = getCachedModel(dim);
      item->A.diagonal() = sqrt_hessian_diagonal.at(key); // use diag(hessian)
      damped.emplace_shared<JacobianFactor>(key, item->A, item->b, item->model);
    } catch (const std::out_of_range &) {
      continue; // Don't attempt any damping if no key found in diagonal
    }
  }
  return damped;
}

/* ************************************************************************* */
GaussianFactorGraph
SQPTrial::buildDampedSystem(const GaussianFactorGraph &linear,
                            const SQPState &state,
                            const LevenbergMarquardtParams &lm_params) const {
  if (lm_params.diagonalDamping)
    return buildDampedSystemDiagonal(linear, state, lm_params);
  else
    return buildDampedSystemUniform(linear, state);
}

/* ************************************************************************* */
void SQPTrial::setNextLambda(double &new_lambda, double &new_lambda_factor,
                             const LevenbergMarquardtParams &params) const {
  if (step_is_successful) {
    setDecreasedNextLambda(new_lambda, new_lambda_factor, params);
  } else {
    setIncreasedNextLambda(new_lambda, new_lambda_factor, params);
  }
}

/* ************************************************************************* */
void SQPTrial::setIncreasedNextLambda(
    double &new_lambda, double &new_lambda_factor,
    const LevenbergMarquardtParams &params) const {
  new_lambda *= new_lambda_factor;
  if (!params.useFixedLambdaFactor) {
    new_lambda_factor *= 2.0;
  }
}

/* ************************************************************************* */
void SQPTrial::setDecreasedNextLambda(
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
/* <============================ SQPLogging ===============================> */
/* ************************************************************************* */

/* ************************************************************************* */
void PrintSQPTrialTitle() {
  cout << setw(10) << "iter   "
       << "|" << setw(12) << "merit "
       << "|" << setw(12) << "cost  "
       << "|" << setw(12) << "e_vio  "
       << "|" << setw(12) << "i_vio  "
       << "|" << setw(10) << "m_system "
       << "|" << setw(12) << "merit_change"
       << "|" << setw(12) << "apprx_change"
       << "|" << setw(12) << "m_fidelity"
       << "|" << setw(10) << "lambda  "
       << "|" << setw(10) << "time  "
       << "|" << setw(10) << "delta_norm"
       << "|" << endl;
}

/* ************************************************************************* */
void PrintSQPTrial(const SQPState &state, const SQPTrial &trial,
                   const SQPParams &params) {
  if (trial.step_is_successful) {
    cout << "\033[0m";
  } else {
    cout << "\033[90m";
  }
  cout << setw(10) << state.iterations << "|";
  if (!trial.solve_successful) {
    cout << "linear solve not successful\n";
    return;
  }
  cout << setw(12) << setprecision(4) << trial.eval.merit << "|";
  cout << setw(12) << setprecision(4) << trial.eval.cost << "|";
  cout << setw(12) << setprecision(4) << trial.eval.e_violation << "|";
  cout << setw(12) << setprecision(4) << trial.eval.i_violation << "|";
  cout << setw(10) << trial.use_merit_system << "|";
  cout << setw(12) << setprecision(4) << trial.nonlinear_merit_change << "|";
  cout << setw(12) << setprecision(4) << trial.linear_merit_change << "|";
  cout << setw(12) << setprecision(4) << trial.model_fidelity << "|";
  cout << setw(10) << setprecision(2) << trial.lambda << "|";
  cout << setw(10) << setprecision(2) << trial.trial_time << "|";
  cout << setw(10) << setprecision(4) << trial.delta.norm() << "|";
  cout << setw(10) << setprecision(4) << trial.bound_rate << "|";
  cout << endl;
  cout << "\033[0m";
}

/* ************************************************************************* */
Eval SQPOptimizer::MeritFunction(const NonlinearFactorGraph &graph,
                                 const EqualityConstraints &e_constraints,
                                 const InequalityConstraints &i_constraints,
                                 const Values &values,
                                 const SQPParams &params) {
  Eval eval;
  eval.cost = graph.error(values);
  eval.e_violation = e_constraints.evaluateViolationL2Norm(values);
  eval.i_violation = i_constraints.evaluateViolationL2Norm(values);
  double e_vio_l2 = 0.5 * pow(eval.e_violation, 2);
  double i_vio_l2 = 0.5 * pow(eval.i_violation, 2);
  eval.merit = eval.cost + params.merit_e_l1_mu * eval.e_violation +
               params.merit_e_l2_mu * e_vio_l2 +
               params.merit_i_l1_mu * eval.i_violation +
               params.merit_i_l2_mu * i_vio_l2;
  return eval;
}

/* ************************************************************************* */
Eval SQPOptimizer::MeritFunctionApprox(const SQPState &state,
                                       const VectorValues &delta,
                                       const SQPParams &params) {
  Eval eval;
  eval.cost = state.linear_cost.error(delta);
  double e_vio_l2 = state.linear_e_merit.error(delta);
  double i_vio_l2 = state.linear_i_merit.error(delta);
  eval.e_violation = sqrt(2 * e_vio_l2);
  eval.i_violation = sqrt(2 * i_vio_l2);
  eval.merit = eval.cost + params.merit_e_l1_mu * eval.e_violation +
               params.merit_e_l2_mu * e_vio_l2 +
               params.merit_i_l1_mu * eval.i_violation +
               params.merit_i_l2_mu * i_vio_l2;
  return eval;
}

/* ************************************************************************* */
/* <=========================== SQPOptimizer ==============================> */
/* ************************************************************************* */

/* ************************************************************************* */
Values SQPOptimizer::optimize(const NonlinearFactorGraph &graph,
                              const EqualityConstraints &e_constraints,
                              const InequalityConstraints &i_constraints,
                              const Values &init_values) {
  SQPState state(init_values, graph, e_constraints, i_constraints, *p_,
                 p_->lm_params.lambdaInitial, p_->lm_params.lambdaFactor, 0);

  if (p_->lm_params.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
    std::cout << "Initial cost: " << state.eval.cost
              << "\te_vio: " << state.eval.e_violation
              << "\ti_vio: " << state.eval.i_violation
              << "\tmerit: " << state.eval.merit << "\n";
    PrintSQPTrialTitle();
  }

  SQPState new_state = state;
  do {
    state = new_state;
    SQPIterDetails iter_details =
        iterate(graph, e_constraints, i_constraints, state);
    details_->push_back(iter_details);
    if (!checkSuccessfulTrial(iter_details)) {
      return state.values;
    }
    new_state = SQPState::FromLastIteration(iter_details, graph, e_constraints,
                                            i_constraints, *p_);
  } while (state.iterations < p_->lm_params.maxIterations &&
           !checkConvergence(state, new_state) &&
           checkLambdaWithinLimits(state.lambda));
  details_->emplace_back(new_state);
  return new_state.values;
}

/* ************************************************************************* */
SQPIterDetails SQPOptimizer::iterate(const NonlinearFactorGraph &graph,
                                     const EqualityConstraints &e_constraints,
                                     const InequalityConstraints &i_constraints,
                                     const SQPState &state) {
  const LevenbergMarquardtParams &lm_params = p_->lm_params;
  SQPIterDetails iter_details(state);

  // Set lambda for first trial.
  double lambda = state.lambda;
  double lambda_factor = state.lambda_factor;

  // Perform trials until any of follwing conditions is met
  // * 1) trial is successful
  // * 2) update is too small
  // * 3) lambda goes beyond limits
  while (true) {
    // Perform the trial.
    SQPTrial trial(state, lambda, graph, e_constraints, i_constraints, *p_);
    if (lm_params.verbosityLM == LevenbergMarquardtParams::SUMMARY) {
      PrintSQPTrial(state, trial, *p_);
    }
    iter_details.trials.emplace_back(trial);

    // Check condition 1.
    if (trial.step_is_successful) {
      break;
    }

    // Check condition 2.
    if (trial.solve_successful) {
      double abs_change_tol =
          std::max(lm_params.absoluteErrorTol,
                   lm_params.relativeErrorTol * state.eval.merit);
      if (abs(trial.linear_merit_change) < abs_change_tol &&
          abs(trial.nonlinear_merit_change) < abs_change_tol) {
        break;
      }
    }

    // Set lambda for next trial.
    trial.setNextLambda(lambda, lambda_factor, lm_params);

    // Check condition 3.
    if (!checkLambdaWithinLimits(lambda)) {
      break;
    }
  }
  return iter_details;
}

/* ************************************************************************* */
bool SQPOptimizer::checkLambdaWithinLimits(const double &lambda) const {
  return lambda <= p_->lm_params.lambdaUpperBound &&
         lambda >= p_->lm_params.lambdaLowerBound;
}

/* ************************************************************************* */
bool SQPOptimizer::checkConvergence(const SQPState &prev_state,
                                    const SQPState &state) const {

  if (state.eval.merit <= p_->lm_params.errorTol)
    return true;

  // check if diverges
  double absoluteDecrease = prev_state.eval.merit - state.eval.merit;

  // calculate relative error decrease and update currentError
  double relativeDecrease = absoluteDecrease / prev_state.eval.merit;
  bool converged = (p_->lm_params.relativeErrorTol &&
                    (relativeDecrease <= p_->lm_params.relativeErrorTol)) ||
                   (absoluteDecrease <= p_->lm_params.absoluteErrorTol);
  return converged;
}

/* ************************************************************************* */
bool SQPOptimizer::checkSuccessfulTrial(
    const SQPIterDetails &iter_details) const {
  for (const auto &trial : iter_details.trials) {
    if (trial.step_is_successful) {
      return true;
    }
  }
  return false;
}

} // namespace gtsam
