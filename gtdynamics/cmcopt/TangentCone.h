/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TangentCone.h
 * @brief Tangent cone formed by linearized active inequality constraints
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/constraints/LinearInequalityConstraint.h>
#include <gtdynamics/optimizer/QPSolver.h>

namespace gtdynamics {
using namespace gtsam;


/** A hyper cone that represents the space defined by A*x >= 0. */
class TangentCone {
public:
  typedef std::shared_ptr<TangentCone> shared_ptr;

protected:
  LinearInequalityConstraints constraints_;

public:
  TangentCone(LinearInequalityConstraints& constraints)
      :constraints_(constraints) {}

  /** Project the vector xi into cone, by solving the convex IQP problem:
   *       argmin x     ||x - xi||^2
   *         s.t.       A * x >=0
   * @return Indices of blocking constraints and the projected vector.
   */
  std::pair<IndexSet, Vector> project(const Vector &xi) const;
};

} // namespace gtdynamics
