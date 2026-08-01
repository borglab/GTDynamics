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

namespace gtdynamics {

/* ************************************************************************* */
gtsam::Vector ObstacleSDFFactorGP::evaluateError(
    const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
    const gtsam::Vector &v2, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  const bool computeJacobians = (H1 || H2 || H3 || H4);

  const gtsam::Vector q = interpolator_.interpolatePose(q1, v1, q2, v2);

  gtsam::Matrix Hq;
  const gtsam::Vector err = obstacleSDFError(
      q, *robot_, *sdf_, epsilon_, radii_, computeJacobians ? &Hq : nullptr);

  if (computeJacobians) {
    interpolator_.updatePoseJacobians(Hq, H1, H2, H3, H4);
  }
  return err;
}

}  // namespace gtdynamics
