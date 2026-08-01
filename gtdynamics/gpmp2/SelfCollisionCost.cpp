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

/* ************************************************************************* */
gtsam::Vector selfCollisionError(const gtsam::Vector &q,
                                 const RobotQueryPoints &robot,
                                 const SelfCollisionPairs &pairs,
                                 const gtsam::Vector &radii,
                                 gtsam::Matrix *Hq) {
  const size_t nrPairs = pairs.size();

  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  robot.queryPoints(q, &wPts, Hq ? &ptJacobians : nullptr);
  if (Hq) *Hq = gtsam::Matrix::Zero(nrPairs, robot.dof());

  gtsam::Vector err(nrPairs);
  for (size_t r = 0; r < nrPairs; ++r) {
    const SelfCollisionPair &pair = pairs[r];
    // The standoff folds in both spheres' radii.
    const double eps = pair.epsilon + radii(pair.a) + radii(pair.b);
    if (Hq) {
      gtsam::Matrix13 HptA, HptB;
      err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps,
                                          HptA, HptB);
      Hq->row(r) = HptA * ptJacobians[pair.a] + HptB * ptJacobians[pair.b];
    } else {
      err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps);
    }
  }
  return err;
}

}  // namespace gtdynamics
