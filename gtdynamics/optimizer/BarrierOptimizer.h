/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  BarrierOptimizer.h
 * @brief Optimizer that treat equality-constrained components as manifolds.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/HistoryLMOptimizer.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

struct BarrierParameters {
  gtsam::LevenbergMarquardtParams lm_params;
  size_t num_iterations;
  double initial_mu;       // initial penalty parameter
  double mu_increase_rate; // increase rate of penalty parameter
  std::vector<gtsam::LevenbergMarquardtParams>
      iters_lm_params; // use different lm parameters for different iterations.
  bool verbose = false;

  using shared_ptr = std::shared_ptr<BarrierParameters>;

  BarrierParameters()
      : lm_params(), num_iterations(5), initial_mu(1.0), mu_increase_rate(2.0) {
  }

  BarrierParameters(const gtsam::LevenbergMarquardtParams &_lm_parameters,
                    const size_t &_num_iterations = 5,
                    const double &_initial_mu = 1.0,
                    const double &_mu_increase_rate = 2.0)
      : lm_params(_lm_parameters), num_iterations(_num_iterations),
        initial_mu(_initial_mu), mu_increase_rate(_mu_increase_rate) {}
};

class BarrierOptimizer {
protected:
  BarrierParameters::shared_ptr p_;

public:
  BarrierOptimizer(const BarrierParameters::shared_ptr &parameters)
      : p_(parameters) {}

  gtsam::Values
  optimize(const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::EqualityConstraints &e_constraints,
           const gtdynamics::InequalityConstraints &i_constraints,
           const gtsam::Values &init_values,
           ConstrainedOptResult *intermediate_result = nullptr) const {

    double mu = p_->initial_mu;
    size_t total_iters = 0;
    size_t total_inner_iters = 0;
    gtsam::Values values = init_values;
    for (size_t i = 0; i < p_->num_iterations; i++) {
      if (p_->verbose) {
        std::cout << "====== iteration " << i << " mu = " << mu << " =======\n";
      }
      gtsam::NonlinearFactorGraph merit_graph = graph;
      for (const auto &constraint : e_constraints) {
        merit_graph.add(constraint->createFactor(mu));
      }
      for (const auto &constraint : i_constraints) {
        merit_graph.add(constraint->createBarrierFactor(mu));
      }
      const auto &lm_params = p_->iters_lm_params.size() > 0
                                  ? p_->iters_lm_params.at(i)
                                  : p_->lm_params;
      gtsam::HistoryLMOptimizer optimizer(merit_graph, values, lm_params);
      values = optimizer.optimize();
      if (p_->verbose) {
        std::cout << "\tLM_iters: " << optimizer.getInnerIterations() << "\n";
        std::cout << "\tcost: " << graph.error(values) << "\n";
        std::cout << "\te_violation: "
                  << e_constraints.evaluateViolationL2Norm(values) << "\n";
        std::cout << "\ti_violation: "
                  << i_constraints.evaluateViolationL2Norm(values) << "\n";
      }
      if (intermediate_result != nullptr) {
        const auto &history_states = optimizer.states();
        for (const auto &state : history_states) {
          intermediate_result->intermediate_values.push_back(state.values);
          intermediate_result->num_iters.push_back(total_iters +
                                                   state.iterations);
          intermediate_result->num_inner_iters.push_back(
              total_inner_iters + state.totalNumberInnerIterations);
          intermediate_result->mu_values.push_back(mu);
        }
        total_iters += optimizer.iterations();
        total_inner_iters += optimizer.getInnerIterations();
      }
      mu *= p_->mu_increase_rate;
    }

    return values;
  }
};

} // namespace gtdynamics
