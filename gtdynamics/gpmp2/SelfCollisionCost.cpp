/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionCost.cpp
 * @brief Hinge loss self collision cost between two world frame points.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#include <gtdynamics/gpmp2/SelfCollisionCost.h>

namespace gtdynamics {

/* ************************************************************************* */
double hingeLossSelfCollisionCost(const gtsam::Point3 &pA,
                                  const gtsam::Point3 &pB, double epsilon,
                                  gtsam::OptionalJacobian<1, 3> HptA,
                                  gtsam::OptionalJacobian<1, 3> HptB) {
  // Below this separation the direction between the points is numerically
  // meaningless (and NaN at exact overlap), so fall back to a fixed one.
  constexpr double kMinDist = 1e-9;

  const gtsam::Vector3 diff = pA - pB;
  const double dist = diff.norm();

  if (dist > epsilon) {
    if (HptA) *HptA = gtsam::Matrix13::Zero();
    if (HptB) *HptB = gtsam::Matrix13::Zero();
    return 0.0;
  }
  // Closer than epsilon: cost falls as the points separate.
  const gtsam::Matrix13 direction =
      dist < kMinDist ? gtsam::Matrix13(gtsam::Vector3::UnitX().transpose())
                      : gtsam::Matrix13(diff.transpose() / dist);
  if (HptA) *HptA = -direction;
  if (HptB) *HptB = direction;
  return epsilon - dist;
}

}  // namespace gtdynamics
