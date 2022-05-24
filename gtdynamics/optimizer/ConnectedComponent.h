/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConnectedComponent.h
 * @brief Variables connected by constraints.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/** Constraint-connected component (CCC) */
class ConnectedComponent {
 public:
  gtdynamics::EqualityConstraints constraints;
  gtsam::NonlinearFactorGraph merit_graph;
  gtsam::KeySet keys;

  using shared_ptr = boost::shared_ptr<ConnectedComponent>;

  /** Constructor from constraints. */
  ConnectedComponent(const gtdynamics::EqualityConstraints& _constraints);
};


}  // namespace gtsam
