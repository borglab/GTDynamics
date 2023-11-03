/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConnectedComponent.h
 * @brief Connected componenet by only considering constraint factors.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

// TODO(yetong): change the namespace to gtdynamics

/** Constraint-connected component (CCC) in a constrained optimization problem.
 * The CCC includes the variables as well as the constraints connecting them.
 * Each CCC will be repaced by a manifold variable for manifold optimization. */
class ConnectedComponent {
public:
  const gtdynamics::EqualityConstraints
      constraints_; // constraints in CCC, h(X)=0
  const gtsam::NonlinearFactorGraph
      merit_graph_; // factor graph representing merit function ||h(X)||^2
  const gtsam::KeySet keys_; // variables in CCC
  const gtsam::KeySet unconstrained_keys_; // only used for IE cases
  using shared_ptr = std::shared_ptr<ConnectedComponent>;

  /// Constructor from constraints.
  ConnectedComponent(const gtdynamics::EqualityConstraints &constraints)
      : constraints_(constraints),
        merit_graph_(constraints.meritGraph()),
        keys_(merit_graph_.keys()),
        unconstrained_keys_() {}

  /// Constructor from constraints.
  ConnectedComponent(const gtdynamics::EqualityConstraints &constraints,
                     const gtsam::KeySet unconstrained_keys)
      : constraints_(constraints),
        merit_graph_(constraints.meritGraph()),
        keys_(merit_graph_.keys()),
        unconstrained_keys_(unconstrained_keys) {}
};

} // namespace gtsam
