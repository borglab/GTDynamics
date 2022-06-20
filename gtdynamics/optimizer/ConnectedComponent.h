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
      constraints_;  // constraints in CCC, h(X)=0
  const gtsam::NonlinearFactorGraph
      merit_graph_;  // factor graph representing merit function ||h(X)||^2
  const gtsam::KeySet keys_;  // variables in CCC
  using shared_ptr = boost::shared_ptr<ConnectedComponent>;

  /** Constructor from constraints. */
  ConnectedComponent(const gtdynamics::EqualityConstraints& constraints)
      : constraints_(constraints),
        merit_graph_(constructMeritGraph(constraints)),
                    keys_(merit_graph_.keys()) {}

 protected:
  NonlinearFactorGraph constructMeritGraph(
      const gtdynamics::EqualityConstraints& constraints);
};

}  // namespace gtsam
