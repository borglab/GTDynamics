/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  AugmentedLagrangianOptimizer.h
 * @brief Augmented Lagrangian method for constrained optimization.
 * @author Yetong Zhang, Frank Dellaert
 */

#pragma once

#include "gtdynamics/optimizer/ConstrainedOptimizer.h"

namespace gtdynamics {

/// Parameters for Augmented Lagrangian method
struct AugmentedLagrangianParameters
    : public ConstrainedOptimizationParameters {
  using Base = ConstrainedOptimizationParameters;
  size_t num_iterations;

  AugmentedLagrangianParameters()
      : Base(gtsam::LevenbergMarquardtParams()), num_iterations(12) {}

  AugmentedLagrangianParameters(
      const gtsam::LevenbergMarquardtParams& _lm_parameters,
      const size_t& _num_iterations = 12)
      : Base(_lm_parameters), num_iterations(_num_iterations) {}
};

/// Augmented Lagrangian method only considering equality constraints.
class AugmentedLagrangianOptimizer : public ConstrainedOptimizer {
 protected:
  const AugmentedLagrangianParameters p_;

 public:
  /// Default constructor
  AugmentedLagrangianOptimizer() : p_(AugmentedLagrangianParameters()) {}

  /// Construct from parameters.
  AugmentedLagrangianOptimizer(const AugmentedLagrangianParameters& parameters)
      : p_(parameters) {}

  /// Run optimization.
  gtsam::Values optimize(
      const gtsam::NonlinearFactorGraph& graph,
      const EqualityConstraints& constraints,
      const gtsam::Values& initial_values,
      ConstrainedOptResult* intermediate_result = nullptr) const override;
};

}  // namespace gtdynamics
