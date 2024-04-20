/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  AugmentedLagrangianOptimizer.cpp
 * @brief Augmented Lagrangian optimization routines.
 * @author: Yetong Zhang, Frank Dellaert
 */

#include <gtdynamics/constrained_optimizer/AugmentedLagrangianOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
/* ********************  Equality Constraints Only  ************************ */
/* ************************************************************************* */

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::updateEParameters(
    const EqualityConstraints &constraints, const Values &previous_values,
    const Values &current_values, double &mu, std::vector<Vector> &z) const {
  // Update Lagrangian multipliers.
  for (size_t constraint_index = 0; constraint_index < constraints.size();
       constraint_index++) {
    auto constraint = constraints.at(constraint_index);
    auto violation = constraint->toleranceScaledViolation(current_values);
    double dual_step_size =
        std::min(p_->max_dual_step_size_e, mu * p_->dual_step_size_factor_e);
    z[constraint_index] -= dual_step_size * violation;
  }

  // Update penalty parameter.
  double previous_error = constraints.evaluateViolationL2Norm(previous_values);
  double current_error = constraints.evaluateViolationL2Norm(current_values);
  if (current_error >= p_->mu_increase_threshold * previous_error) {
    mu *= p_->mu_e_increase_rate;
  }
}

/* ************************************************************************* */
NonlinearFactorGraph AugmentedLagrangianOptimizer::LagrangeDualFunction(
    const NonlinearFactorGraph &graph, const EqualityConstraints &e_constraints,
    const double mu, const std::vector<Vector> &lambda) {

  NonlinearFactorGraph dual_graph = graph;

  // Create factors corresponding to penalty terms of constraints.
  for (size_t constraint_index = 0; constraint_index < e_constraints.size();
       constraint_index++) {
    auto constraint = e_constraints.at(constraint_index);
    Vector bias = -lambda[constraint_index] / mu * constraint->tolerance();
    dual_graph.add(constraint->createFactor(mu, bias));
  }
  return dual_graph;
}

/* ************************************************************************* */
Values
AugmentedLagrangianOptimizer::optimize(const NonlinearFactorGraph &graph,
                                       const EqualityConstraints &constraints,
                                       const Values &initial_values) const {
  Values values = initial_values;

  // Set initial values for penalty parameter and Lagrangian multipliers.
  double mu = p_->initial_mu_e; // penalty parameter
  std::vector<Vector> lambda;   // Lagrangian multiplier
  for (const auto &constraint : constraints) {
    lambda.push_back(Vector::Zero(constraint->dim()));
  }

  // Solve the constrained optimization problem by solving a sequence of
  // unconstrained optimization problems.
  for (int i = 0; i < p_->num_iterations; i++) {
    // Minimize Lagrange dual function.
    NonlinearFactorGraph dual_function =
        LagrangeDualFunction(graph, constraints, mu, lambda);
    auto optimizer = CreateIterLMOptimizer(dual_function, values, i);
    auto result = optimizer->optimize();

    // Update parameters.
    updateEParameters(constraints, values, result, mu, lambda);

    // Update values.
    values = result;

    /// Store intermediate results.
    if (p_->store_iter_details) {
      auto iter_details = RetrieveIterDetails(optimizer, values);
      iter_details.mu_e = mu;
      iter_details.lambda_e = lambda;
      details_->emplace_back(iter_details);
    }
  }
  return values;
}

/* ************************************************************************* */
/* *****************  I-E Constraints using Dual Ascent  ******************* */
/* ************************************************************************* */

/* ************************************************************************* */
NonlinearFactorGraph AugmentedLagrangianOptimizer::LagrangeDualFunction(
    const NonlinearFactorGraph &graph, const EqualityConstraints &e_constraints,
    const InequalityConstraints &i_constraints, const double mu_e,
    const double mu_i, const std::vector<Vector> &lambda_e,
    const std::vector<double> &lambda_i, const double d) {
  NonlinearFactorGraph merit_graph =
      LagrangeDualFunction(graph, e_constraints, mu_e, lambda_e);

  // Create factors corresponding to penalty terms of i-constraints.
  merit_graph.add(i_constraints.meritGraph(mu_i));

  // Create factors corresponding to Lagrange multiplier terms of i-constraints.
  for (size_t c_idx = 0; c_idx < i_constraints.size(); c_idx++) {
    const auto &constraint = i_constraints.at(c_idx);
    double bias = -lambda_i[c_idx] / d * constraint->tolerance()(0);
    Vector bias_vec = Vector1(bias);
    merit_graph.add(
        constraint->createEqualityConstraint()->createFactor(d, bias_vec));
  }
  return merit_graph;
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::updateIParameters(
    const InequalityConstraints &constraints, const Values &previous_values,
    const Values &current_values, double &mu,
    std::vector<double> &lambda) const {

  // Update Lagrangian multipliers.
  for (size_t c_idx = 0; c_idx < constraints.size(); c_idx++) {
    auto constraint = constraints.at(c_idx);
    double violation = constraint->toleranceScaledEvaluation(current_values);
    double dual_step_size =
        std::min(p_->max_dual_step_size_i, mu * p_->dual_step_size_factor_i);
    // std::cout << "violation: " << violation << "\n";
    lambda[c_idx] = std::max(0.0, lambda[c_idx] - dual_step_size * violation);
    // std::cout << "lambda: " << lambda[c_idx] << "\n";
  }

  // Update penalty parameter.
  double previous_error = constraints.evaluateViolationL2Norm(previous_values);
  double current_error = constraints.evaluateViolationL2Norm(current_values);
  // std::cout << "previous_error: " << previous_error << "\tcurrent_error: " <<
  // current_error << "\n";
  if (current_error >= p_->mu_increase_threshold * previous_error) {
    mu *= p_->mu_i_increase_rate;
  }
}

/* ************************************************************************* */
Values AugmentedLagrangianOptimizer::optimize(
    const NonlinearFactorGraph &graph, const EqualityConstraints &e_constraints,
    const InequalityConstraints &i_constraints,
    const Values &init_values) const {

  // Ensure that all i-constraints are 1-dimensional
  if (i_constraints.size() != i_constraints.dim()) {
    InequalityConstraints new_i_constraints;
    for (const auto &constraint : i_constraints) {
      if (constraint->dim() == 1) {
        new_i_constraints.push_back(constraint);
      } else {
        auto twin_constraint =
            std::static_pointer_cast<TwinDoubleExpressionInequality>(
                constraint);
        new_i_constraints.push_back(twin_constraint->constraint1());
        new_i_constraints.push_back(twin_constraint->constraint2());
      }
    }
    return optimize(graph, e_constraints, new_i_constraints, init_values);
  }


  // Set initial values for penalty parameter and Lagrangian multipliers.
  double mu_e = p_->initial_mu_e;
  double mu_i = p_->initial_mu_i;
  std::vector<Vector> lambda_e;
  for (const auto &constraint : e_constraints) {
    lambda_e.push_back(Vector::Zero(constraint->dim()));
  }
  std::vector<double> lambda_i(i_constraints.size(), 0);

  // Solve the constrained optimization problem by solving a sequence of
  // unconstrained optimization problems.
  Values values = init_values;
  for (int i = 0; i < p_->num_iterations; i++) {
    // Minimize Lagrange dual function.
    NonlinearFactorGraph dual_function = LagrangeDualFunction(
        graph, e_constraints, i_constraints, mu_e, mu_i, lambda_e, lambda_i);
    auto optimizer = CreateIterLMOptimizer(dual_function, values, i);
    auto result = optimizer->optimize();

    // Update parameters.
    updateEParameters(e_constraints, values, result, mu_e, lambda_e);
    updateIParameters(i_constraints, values, result, mu_i, lambda_i);

    // Update values.
    values = result;

    if (p_->verbose) {
      std::cout << "mu_e: " << mu_e << "\tmu_i: " << mu_i << "\n";
    }

    /// Store intermediate results.
    if (p_->store_iter_details) {
      auto iter_details = RetrieveIterDetails(optimizer, values);
      iter_details.mu_e = mu_e;
      iter_details.mu_i = mu_i;
      iter_details.lambda_e = lambda_e;
      iter_details.lambda_i = lambda_i;
      details_->emplace_back(iter_details);
    }
  }
  return values;
}

/* ************************************************************************* */
/* ********************************  misc  ********************************* */
/* ************************************************************************* */

/* ************************************************************************* */
std::shared_ptr<LevenbergMarquardtOptimizer>
AugmentedLagrangianOptimizer::CreateIterLMOptimizer(
    const NonlinearFactorGraph &graph, const Values &values,
    const int i) const {
  const auto &lm_params = p_->iters_lm_params.size() > 0
                              ? p_->iters_lm_params.at(i)
                              : p_->lm_params;
  return CreateLMOptimizer(graph, values, p_->store_lm_details, lm_params);
}

/* ************************************************************************* */
AugmentedLagrangianIterDetails
AugmentedLagrangianOptimizer::RetrieveIterDetails(
    std::shared_ptr<LevenbergMarquardtOptimizer> optimizer,
    const Values &values) const {
  AugmentedLagrangianIterDetails iter_details;
  iter_details.values = values;
  iter_details.num_lm_iters = optimizer->iterations();
  iter_details.num_lm_inner_iters = optimizer->getInnerIterations();

  if (p_->store_lm_details) {
    std::tie(iter_details.lm_iters_values, iter_details.lm_inner_iters) =
        RetrieveLMItersValues(optimizer);
  }
  return iter_details;
}

} // namespace gtsam
