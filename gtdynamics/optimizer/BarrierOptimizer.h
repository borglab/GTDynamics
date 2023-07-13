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

#include "gtdynamics/optimizer/EqualityConstraint.h"
#include <gtdynamics/manifold/IneqConstraintManifold.h>
#include <gtdynamics/optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>

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

  ConstrainedOptResult
  optimize(const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::InequalityConstraints &constraints,
           const gtsam::Values &init_values) const {
    ConstrainedOptResult result;

    double mu = p_.initial_mu;
    gtsam::Values values = init_values;
    for (size_t i = 0; i < p_.num_iterations; i++) {
      gtsam::NonlinearFactorGraph merit_graph = graph;
      for (const auto &constraint : constraints) {
        merit_graph.add(constraint->createBarrierFactor(mu));
      }
      gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                   p_.lm_parameters);
      values = optimizer.optimize();
      result.intermediate_values.push_back(values);
      result.num_iters.push_back(optimizer.getInnerIterations());
      result.mu_values.push_back(mu);
      mu *= p_.mu_increase_rate;
    }

    return result;
  }

  ConstrainedOptResult
  optimize(const gtsam::NonlinearFactorGraph &graph,
           const gtdynamics::EqualityConstraints &e_constraints,
           const gtdynamics::InequalityConstraints &i_constraints,
           const gtsam::Values &init_values) const {
    ConstrainedOptResult result;

    double mu = p_.initial_mu;
    gtsam::Values values = init_values;
    for (size_t i = 0; i < p_.num_iterations; i++) {
      gtsam::NonlinearFactorGraph merit_graph = graph;
      for (const auto &constraint : e_constraints) {
        merit_graph.add(constraint->createFactor(mu));
      }
      for (const auto &constraint : i_constraints) {
        merit_graph.add(constraint->createBarrierFactor(mu));
      }
      gtsam::LevenbergMarquardtOptimizer optimizer(merit_graph, values,
                                                   p_.lm_parameters);
      values = optimizer.optimize();
      result.intermediate_values.push_back(values);
      result.num_iters.push_back(optimizer.getInnerIterations());
      result.mu_values.push_back(mu);
      mu *= p_.mu_increase_rate;
    }

    return result;
  }
};

} // namespace gtdynamics
