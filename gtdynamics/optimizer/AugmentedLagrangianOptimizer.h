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
  size_t num_iterations;

  AugmentedLagrangianParameters() : num_iterations(12) {
    lm_parameters = gtsam::LevenbergMarquardtParams();
  }
};

/// Augmented Lagrangian method only considering equality constraints.
class AugmentedLagrangianOptimizer : public ConstrainedOptimizer {
 protected:
  boost::shared_ptr<const AugmentedLagrangianParameters> p_;

 public:
  AugmentedLagrangianOptimizer()
      : p_(boost::make_shared<const AugmentedLagrangianParameters>()) {}

  /* Construct from parameters. */
  AugmentedLagrangianOptimizer(
      const boost::shared_ptr<const AugmentedLagrangianParameters>& parameters)
      : p_(parameters) {}

  /// Run optimization.
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                         const EqualityConstraints& constraints,
                         const gtsam::Values& initial_values,
                         boost::optional<ConstrainedOptResult*> opt_result =
                             boost::none) const override;
};

}  // namespace gtdynamics
