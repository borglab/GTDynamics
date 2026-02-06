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

#include <gtdynamics/optimizer/Optimizer.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;

namespace {

gtsam::NonlinearEqualityConstraints ToGtsamConstraints(
    const gtdynamics::EqualityConstraints& constraints) {
  gtsam::NonlinearEqualityConstraints gtsam_constraints;
  for (const auto& constraint : constraints) {
    auto factor = constraint->createFactor(1.0);
    gtsam_constraints.emplace_shared<gtsam::ZeroCostConstraint>(factor);
  }
  return gtsam_constraints;
}

gtsam::ConstrainedOptProblem ToGtsamProblem(
    const gtsam::NonlinearFactorGraph& graph,
    const gtdynamics::EqualityConstraints& constraints) {
  auto gtsam_constraints = ToGtsamConstraints(constraints);
  return gtsam::ConstrainedOptProblem::EqConstrainedOptProblem(graph,
                                                               gtsam_constraints);
}

}  // namespace

Values Optimizer::optimize(const NonlinearFactorGraph& graph,
                           const Values& initial_values) const {
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values,
                                               p_.lm_parameters);
  const Values result = optimizer.optimize();
  return result;
}

Values Optimizer::optimize(const gtsam::NonlinearFactorGraph& graph,
                           const gtdynamics::EqualityConstraints& constraints,
                           const gtsam::Values& initial_values) const {
  if (p_.method == OptimizationParameters::Method::SOFT_CONSTRAINTS) {
    auto merit_graph = graph;
    for (const auto& constraint : constraints) {
      merit_graph.add(constraint->createFactor(1.0));
    }
    return optimize(merit_graph, initial_values);

  } else if (p_.method == OptimizationParameters::Method::PENALTY) {
    auto params = std::make_shared<gtsam::PenaltyOptimizerParams>();
    params->lmParams = p_.lm_parameters;
    gtsam::PenaltyOptimizer optimizer(ToGtsamProblem(graph, constraints),
                                      initial_values, params);
    return optimizer.optimize();

  } else if (p_.method ==
             OptimizationParameters::Method::AUGMENTED_LAGRANGIAN) {
    auto params = std::make_shared<gtsam::AugmentedLagrangianParams>();
    params->lmParams = p_.lm_parameters;
    gtsam::AugmentedLagrangianOptimizer optimizer(
        ToGtsamProblem(graph, constraints), initial_values, params);
    return optimizer.optimize();

  } else {
    throw std::runtime_error("optimization method not recognized.");
  }
}

}  // namespace gtdynamics
