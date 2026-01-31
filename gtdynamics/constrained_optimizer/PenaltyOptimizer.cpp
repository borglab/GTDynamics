/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyOptimizer.cpp
 * @brief Penalty method optimization routines.
 * @author: Yetong Zhang
 */

#include <gtdynamics/constrained_optimizer/PenaltyOptimizer.h>

namespace gtdynamics {

/* ************************************************************************* */
NonlinearFactorGraph
PenaltyOptimizer::MeritFunction(const NonlinearFactorGraph &cost,
                                const EqualityConstraints &constraints,
                                const double mu) {
  NonlinearFactorGraph graph = cost;
  graph.add(constraints.meritGraph(mu));
  return graph;
}

/* ************************************************************************* */
NonlinearFactorGraph
PenaltyOptimizer::MeritFunction(const NonlinearFactorGraph &cost,
                                const EqualityConstraints &e_constraints,
                                const InequalityConstraints &i_constraints,
                                const double mu_e, const double mu_i) {
  NonlinearFactorGraph graph = cost;
  graph.add(e_constraints.meritGraph(mu_e));
  graph.add(i_constraints.meritGraph(mu_i));
  return graph;
}

/* ************************************************************************* */
std::shared_ptr<LevenbergMarquardtOptimizer>
PenaltyOptimizer::CreateIterLMOptimizer(const NonlinearFactorGraph &graph,
                                        const Values &values,
                                        const int i) const {
  const auto &lm_params = p_->iters_lm_params.size() > 0
                              ? p_->iters_lm_params.at(i)
                              : p_->lm_params;
  return CreateLMOptimizer(graph, values, p_->store_lm_details, lm_params);
}

/* ************************************************************************* */
PenaltyIterDetails PenaltyOptimizer::RetrieveIterDetails(
    std::shared_ptr<LevenbergMarquardtOptimizer> optimizer,
    const Values &values) const {
  PenaltyIterDetails iter_details;
  iter_details.values = values;
  iter_details.num_lm_iters = optimizer->iterations();
  iter_details.num_lm_inner_iters = optimizer->getInnerIterations();

  if (p_->store_lm_details) {
    std::tie(iter_details.lm_iters_values, iter_details.lm_inner_iters) =
        RetrieveLMItersValues(optimizer);
  }
  return iter_details;
}

/* ************************************************************************* */
Values PenaltyOptimizer::optimize(const NonlinearFactorGraph &cost,
                                  const EqualityConstraints &constraints,
                                  const Values &initial_values) const {
  Values values = initial_values;
  double mu = p_->initial_mu;

  // Solve the constrained optimization problem by solving a sequence of
  // unconstrained optimization problems.
  for (int i = 0; i < p_->num_iterations; i++) {
    // Construct merit function.
    NonlinearFactorGraph merit_graph = MeritFunction(cost, constraints, mu);

    // Run optimization.
    auto optimizer = CreateIterLMOptimizer(merit_graph, values, i);
    values = optimizer->optimize();

    // Update parameters.
    mu *= p_->mu_increase_rate;

    /// Store intermediate results.
    if (p_->store_iter_details) {
      details_->emplace_back(RetrieveIterDetails(optimizer, values));
    }
  }
  return values;
}

/* ************************************************************************* */
Values PenaltyOptimizer::optimize(const NonlinearFactorGraph &cost,
                                  const EqualityConstraints &e_constraints,
                                  const InequalityConstraints &i_constraints,
                                  const Values &init_values) const {

  double mu = p_->initial_mu;
  size_t total_iters = 0;
  size_t total_inner_iters = 0;
  Values values = init_values;
  for (size_t i = 0; i < p_->num_iterations; i++) {
    // Construct merit function.
    NonlinearFactorGraph merit_graph =
        MeritFunction(cost, e_constraints, i_constraints, mu, mu);

    // Run optimization.
    auto optimizer = CreateIterLMOptimizer(merit_graph, values, i);
    values = optimizer->optimize();

    // Display info
    if (p_->verbose) {
      std::cout << "====== iteration " << i << " mu = " << mu << " =======\n";
      std::cout << "\tLM_iters: " << optimizer->getInnerIterations() << "\n";
      std::cout << "\tcost: " << cost.error(values) << "\n";
      std::cout << "\te_violation: "
                << e_constraints.evaluateViolationL2Norm(values) << "\n";
      std::cout << "\ti_violation: "
                << i_constraints.evaluateViolationL2Norm(values) << "\n";
    }

    // Update parameters.
    mu *= p_->mu_increase_rate;

    // Store intermediate results.
    if (p_->store_iter_details) {
      details_->emplace_back(RetrieveIterDetails(optimizer, values));
    }
  }

  return values;
}

} // namespace gtdynamics
