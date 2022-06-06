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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/** Constraint-connected component (CCC) in a constrained optimization problem.
 * The CCC includes the variables as well as the constraints connecting them.
 * Each CCC will be repaced by a manifold variable for manifold optimization. */
class ConnectedComponent {
 public:
  const gtdynamics::EqualityConstraints
      constraints;  // constraints in CCC, h(X)=0
  const gtsam::NonlinearFactorGraph
      merit_graph;  // factor graph representing merit function ||h(X)||^2
  const gtsam::KeySet keys;  // variables in CCC
  using shared_ptr = boost::shared_ptr<ConnectedComponent>;

  /** Constructor from constraints. */
  ConnectedComponent(const gtdynamics::EqualityConstraints& _constraints)
      : constraints(_constraints),
        merit_graph(construct_merit_graph(_constraints)),
                    keys(merit_graph.keys()) {}

 protected:
  NonlinearFactorGraph construct_merit_graph(
      const gtdynamics::EqualityConstraints& _constraints);
};

}  // namespace gtsam
