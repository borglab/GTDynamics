/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizer.h
 * @brief Optimizer that treat equality-constrained components as manifolds.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

#include "gtdynamics/manifold/TspaceBasis.h"

namespace gtsam {

/// Parameters for manifold optimizer.
struct ManifoldOptimizerParameters
    : public gtdynamics::ConstrainedOptimizationParameters {
  using Base = gtdynamics::ConstrainedOptimizationParameters;
  ConstraintManifold::Params::shared_ptr
      cc_params;               // Parameter for constraint-connected components
  bool retract_init = true;    // Perform retraction on constructing values for
                               // connected component.
  bool retract_final = false;  // Perform retraction on manifolds after
                               // optimization, used for infeasible methods.
  /// Default Constructor.
  ManifoldOptimizerParameters();
};

/// Base class for manifold optimizer.
class ManifoldOptimizer : public gtdynamics::ConstrainedOptimizer {
 public:
  using shared_ptr = std::shared_ptr<const ManifoldOptimizer>;

 protected:
  const ManifoldOptimizerParameters p_;

 public:
  /// Default constructor.
  ManifoldOptimizer() : p_(ManifoldOptimizerParameters()) {}

  /// Construct from parameters.
  ManifoldOptimizer(const ManifoldOptimizerParameters& parameters)
      : p_(parameters) {}

 protected:
  /** Perform dfs to find the connected component that contains start_key. Will
   * also erase all the keys in the connected component from keys.
   */
  ConnectedComponent::shared_ptr findConnectedComponent(
      const gtdynamics::EqualityConstraints& constraints,
      const gtsam::Key start_key, gtsam::KeySet& keys,
      const gtsam::VariableIndex& var_index) const;

  /// Identify the connected components by constraints.
  std::vector<ConnectedComponent::shared_ptr> identifyConnectedComponents(
      const gtdynamics::EqualityConstraints& constraints) const;
};

}  // namespace gtsam
