/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Optimizer.cpp
 * @brief Optimization routines.
 * @author: Frank Dellaert
 */

#include <gtdynamics/constrained_optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/Optimizer.h>
#include <gtdynamics/constrained_optimizer/PenaltyOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;

Values Optimizer::optimize(const NonlinearFactorGraph& graph,
                           const Values& initial_values) const {
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values,
                                               p_.lm_parameters);
  const Values result = optimizer.optimize();
  return result;
}

Values Optimizer::optimize(const gtsam::NonlinearFactorGraph& graph,
                           const gtsam::EqualityConstraints& constraints,
                           const gtsam::Values& initial_values) const {
  if (p_.method == OptimizationParameters::Method::SOFT_CONSTRAINTS) {
    auto merit_graph = graph;
    for (const auto& constraint : constraints) {
      merit_graph.add(constraint->createFactor(1.0));
    }
    return optimize(merit_graph, initial_values);

  } else if (p_.method == OptimizationParameters::Method::PENALTY) {
    auto params = std::make_shared<gtsam::PenaltyParameters>(p_.lm_parameters);
    gtsam::PenaltyOptimizer optimizer(params);
    return optimizer.optimize(graph, constraints, initial_values);

  } else if (p_.method ==
             OptimizationParameters::Method::AUGMENTED_LAGRANGIAN) {
    auto params = std::make_shared<gtsam::AugmentedLagrangianParameters>(
        p_.lm_parameters);
    gtsam::AugmentedLagrangianOptimizer optimizer(params);
    return optimizer.optimize(graph, constraints, initial_values);

  } else {
    throw std::runtime_error("optimization method not recognized.");
  }
}

}  // namespace gtdynamics
