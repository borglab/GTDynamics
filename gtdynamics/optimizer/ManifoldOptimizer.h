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

#include <gtdynamics/optimizer/ConstraintManifold.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

namespace gtsam {

/** Function to find the basis keys for constraint manifold. */
typedef KeyVector (*BasisKeyFunc)(const ConnectedComponent::shared_ptr&);

/** Base class for manifold optimizer. */
// TODO(yetong): make ManifoldOptimizer inherit from ConstrainedOptimizer.
class ManifoldOptimizer {
 public:
  using shared_ptr = boost::shared_ptr<const ManifoldOptimizer>;

  /** Parameters for manifold optimizer. */
  struct Params {
    ConstraintManifold::Params::shared_ptr
        cc_params;             // Parameter for constraint-connected components
    bool retract_init = true;  // Perform retraction on constructing values for
                               // connected component.
    using shared_ptr = boost::shared_ptr<Params>;

    /** Default Constructor. */
    Params();
  };

 protected:
  gtsam::NonlinearFactorGraph costs_;            // cost function
  gtdynamics::EqualityConstraints constraints_;  // equality constraints
  Params::shared_ptr params_;
  std::vector<ConnectedComponent::shared_ptr>
      components_;               // All the constraint-connected components
  BasisKeyFunc basis_key_func_;  // ad-hoc function to manually specify the
                                 // basis keys for each constraint manifold

 public:
  /** Default constructor. */
  ManifoldOptimizer() {}

  /** Constructor. */
  ManifoldOptimizer(const gtsam::NonlinearFactorGraph& costs,
                    const gtdynamics::EqualityConstraints& constraints,
                    const Params::shared_ptr& params,
                    boost::optional<BasisKeyFunc> basis_key_func = boost::none)
      : costs_(costs), constraints_(constraints), params_(params) {
    if (basis_key_func) {
      basis_key_func_ = *basis_key_func;
    }
    identifyConnectedComponents();
  }

  /** Run optimization on constraint manifold. */
  virtual const gtsam::Values& optimize() = 0;

  /** Customizable print function. */
  virtual void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

 protected:
  /** Perform dfs to find the connected component that contains start_key. Will
   * also erase all the keys in the connected component from keys.
   */
  ConnectedComponent::shared_ptr dfsFindConnectedComponent(
      const gtsam::Key start_key, gtsam::KeySet& keys,
      const gtsam::VariableIndex& var_index) const;

  /** Identify the connected components by constraints. */
  void identifyConnectedComponents();
};

}  // namespace gtsam
