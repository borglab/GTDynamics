/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleCost.cpp
 * @brief Hinge loss obstacle cost functions on a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#include <gtdynamics/gpmp2/ObstacleCost.h>

#include <algorithm>
#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
double hingeLossObstacleCost(const gtsam::Point3 &point,
                             const SignedDistanceField &sdf, double epsilon,
                             gtsam::OptionalJacobian<1, 3> Hpt) {
  gtsam::Vector3 fieldGradient;
  double signedDist;
  try {
    signedDist = sdf.getSignedDistance(point, fieldGradient);
  } catch (const SDFQueryOutOfRange &) {
    // Fail closed: a signed distance field is 1-Lipschitz, so the distance off
    // the grid is at least the distance at the nearest grid point minus the
    // offset to it. Evaluating the hinge on that lower bound grows the cost as
    // the point leaves the grid, with a gradient pointing back inside.
    const gtsam::Point3 &lo = sdf.origin();
    const gtsam::Point3 hi =
        lo + gtsam::Point3((sdf.xCount() - 1.0) * sdf.cellSize(),
                           (sdf.yCount() - 1.0) * sdf.cellSize(),
                           (sdf.zCount() - 1.0) * sdf.cellSize());
    const gtsam::Point3 clamped(std::clamp(point.x(), lo.x(), hi.x()),
                                std::clamp(point.y(), lo.y(), hi.y()),
                                std::clamp(point.z(), lo.z(), hi.z()));
    const gtsam::Vector3 offset = point - clamped;
    const double offsetNorm = offset.norm();  // > 0, the point is off the grid

    gtsam::Vector3 clampedGradient;
    const double clampedDist = sdf.getSignedDistance(clamped, clampedGradient);
    signedDist = clampedDist - offsetNorm;
    // A clamped axis no longer moves the field query, only the offset.
    const gtsam::Vector3 mask(point.x() == clamped.x() ? 1.0 : 0.0,
                              point.y() == clamped.y() ? 1.0 : 0.0,
                              point.z() == clamped.z() ? 1.0 : 0.0);
    fieldGradient =
        clampedGradient.cwiseProduct(mask) - offset / offsetNorm;
  }

  if (signedDist > epsilon) {
    if (Hpt) *Hpt = gtsam::Matrix13::Zero();
    return 0.0;
  }
  // Inside the obstacle, or outside it but closer than epsilon.
  if (Hpt) *Hpt = -fieldGradient.transpose();
  return epsilon - signedDist;
}

/* ************************************************************************* */
double hingeLossObstacleCost(const gtsam::Pose3 &wTs,
                             const gtsam::Point3 &point,
                             const SignedDistanceField &sdf, double epsilon,
                             gtsam::OptionalJacobian<1, 6> Hpose,
                             gtsam::OptionalJacobian<1, 3> Hpt) {
  gtsam::Matrix36 HlocalPose;
  gtsam::Matrix3 HlocalPt;
  const gtsam::Point3 sP = wTs.transformTo(point, HlocalPose, HlocalPt);

  gtsam::Matrix13 HerrLocal;
  const double cost = hingeLossObstacleCost(sP, sdf, epsilon, HerrLocal);

  if (Hpose) *Hpose = HerrLocal * HlocalPose;
  if (Hpt) *Hpt = HerrLocal * HlocalPt;
  return cost;
}

/* ************************************************************************* */
gtsam::Vector internal::hingeLossOverPoints(
    const std::vector<gtsam::Point3> &wPts,
    const std::vector<gtsam::Matrix> &ptJacobians,
    const SignedDistanceField &sdf, double epsilon, const gtsam::Vector &radii,
    gtsam::Matrix *Hq) {
  const size_t nrPts = wPts.size();
  if (Hq) *Hq = gtsam::Matrix::Zero(nrPts, ptJacobians.front().cols());

  gtsam::Vector err(nrPts);
  for (size_t i = 0; i < nrPts; ++i) {
    const double eps = epsilon + radii(i);
    if (Hq) {
      gtsam::Matrix13 Hpt;
      err(i) = gtdynamics::hingeLossObstacleCost(wPts[i], sdf, eps, Hpt);
      Hq->row(i) = Hpt * ptJacobians[i];
    } else {
      err(i) = gtdynamics::hingeLossObstacleCost(wPts[i], sdf, eps);
    }
  }
  return err;
}

/* ************************************************************************* */
gtsam::Vector obstacleSDFError(const gtsam::Vector &q,
                               const RobotQueryPoints &robot,
                               const SignedDistanceField &sdf, double epsilon,
                               const gtsam::Vector &radii, gtsam::Matrix *Hq) {
  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  robot.queryPoints(q, &wPts, Hq ? &ptJacobians : nullptr);
  return internal::hingeLossOverPoints(wPts, ptJacobians, sdf, epsilon, radii,
                                       Hq);
}

}  // namespace gtdynamics
