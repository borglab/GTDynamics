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

#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/optimizer/HistoryLMOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

struct BarrierParameters : public ConstrainedOptimizationParameters {
  using Base = ConstrainedOptimizationParameters;
  size_t num_iterations;
  double initial_mu;       // initial penalty parameter
  double mu_increase_rate; // increase rate of penalty parameter

  BarrierParameters()
      : Base(gtsam::LevenbergMarquardtParams()), num_iterations(5),
        initial_mu(1.0), mu_increase_rate(2.0) {}

  BarrierParameters(const gtsam::LevenbergMarquardtParams &_lm_parameters,
                    const size_t &_num_iterations = 5,
                    const double &_initial_mu = 1.0,
                    const double &_mu_increase_rate = 2.0)
      : Base(_lm_parameters), num_iterations(_num_iterations),
        initial_mu(_initial_mu), mu_increase_rate(_mu_increase_rate) {}
};

class BarrierOptimizer {
protected:
  BarrierParameters p_;

public:
  BarrierOptimizer(const BarrierParameters &parameters) : p_(parameters) {}

  gtsam::Values
  optimize(const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::EqualityConstraints &e_constraints,
           const gtdynamics::InequalityConstraints &i_constraints,
           const gtsam::Values &init_values,
           ConstrainedOptResult *intermediate_result = nullptr) const {

    double mu = p_.initial_mu;
    size_t total_iters = 0;
    size_t total_inner_iters = 0;
    gtsam::Values values = init_values;
    for (size_t i = 0; i < p_.num_iterations; i++) {
      gtsam::NonlinearFactorGraph merit_graph = graph;
      for (const auto &constraint : e_constraints) {
        merit_graph.add(constraint->createFactor(mu));
      }
      for (const auto &constraint : i_constraints) {
        merit_graph.add(constraint->createBarrierFactor(mu));
      }
      gtsam::HistoryLMOptimizer optimizer(merit_graph, values,
                                          p_.lm_parameters);
      values = optimizer.optimize();
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
      mu *= p_.mu_increase_rate;
    }

    return values;
  }
};

} // namespace gtdynamics
