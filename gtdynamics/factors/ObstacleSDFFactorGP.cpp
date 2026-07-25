/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleSDFFactorGP.cpp
 * @brief Obstacle avoidance cost factor at a Gaussian process interpolated
 *        state, using a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#include <gtdynamics/factors/ObstacleSDFFactorGP.h>

#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
gtsam::Vector ObstacleSDFFactorGP::evaluateError(
    const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
    const gtsam::Vector &v2, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  const bool computeJacobians = (H1 || H2 || H3 || H4);
  const size_t nrPts = robot_.nrPoints();

  gtsam::Matrix Hq1, Hv1, Hq2, Hv2;
  const gtsam::Vector q =
      computeJacobians ? interpolator_.interpolatePose(q1, v1, q2, v2, &Hq1,
                                                       &Hv1, &Hq2, &Hv2)
                       : interpolator_.interpolatePose(q1, v1, q2, v2);

  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  robot_.queryPoints(q, &wPts, computeJacobians ? &ptJacobians : nullptr);

  gtsam::Vector err(nrPts);
  gtsam::Matrix errJacobian = gtsam::Matrix::Zero(nrPts, robot_.dof());
  for (size_t i = 0; i < nrPts; ++i) {
    if (computeJacobians) {
      gtsam::Matrix13 Hpt;
      err(i) = hingeLossObstacleCost(wPts[i], *sdf_, epsilon_, Hpt);
      errJacobian.row(i) = Hpt * ptJacobians[i];
    } else {
      err(i) = hingeLossObstacleCost(wPts[i], *sdf_, epsilon_);
    }
  }

  if (computeJacobians) {
    GPLinearInterpolator::updatePoseJacobians(errJacobian, Hq1, Hv1, Hq2, Hv2,
                                              H1, H2, H3, H4);
  }
  return err;
}

}  // namespace gtdynamics
