/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file ConstraintManifold.cpp
 * @brief Default ambient recovery for constrained runtime types.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/constrained/CMPoint.h>

namespace gtdynamics {
using namespace gtsam;

const Value& ConstraintManifold::recover(
    const CMPoint& point, Key key,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H) const {
  if (point.manifold().get() != this)
    throw std::invalid_argument(
        "Recovery requires a point of this runtime type");
  if (H)
    *H = point.dim() ? point.eBasis()->recoverJacobian(key)
                     : Matrix::Zero(point.values().at(key).dim(), 0);
  return point.values().at(key);
}

}  // namespace gtdynamics
