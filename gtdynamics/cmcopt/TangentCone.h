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


/// Tangent cone C = {x | A x >= 0} from thesis Eq. (4.16), built from
/// linearized active inequalities.
class TangentCone {
public:
  typedef std::shared_ptr<TangentCone> shared_ptr;

protected:
  LinearInequalityConstraints constraints_;

public:
  TangentCone(LinearInequalityConstraints& constraints)
      :constraints_(constraints) {}

  /// Project xi as in thesis Eq. (4.31), solving argmin_x ||x - xi||^2 subject
  /// to A x >= 0, and return the active rows that block xi.
  std::pair<IndexSet, Vector> project(const Vector &xi) const;
};

} // namespace gtdynamics
