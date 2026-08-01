/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionSphereFactorGP.cpp
 * @brief Self collision cost factor at a Gaussian process interpolated state.
 * @author Karthik Shaji
 */

#include <gtdynamics/factors/SelfCollisionSphereFactorGP.h>

namespace gtdynamics {

/* ************************************************************************* */
gtsam::Vector SelfCollisionSphereFactorGP::evaluateError(
    const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
    const gtsam::Vector &v2, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  const bool computeJacobians = (H1 || H2 || H3 || H4);

  const gtsam::Vector q = interpolator_.interpolatePose(q1, v1, q2, v2);

  gtsam::Matrix Hq;
  const gtsam::Vector err = selfCollisionError(
      q, *robot_, pairs_, radii_, computeJacobians ? &Hq : nullptr);

  if (computeJacobians) {
    interpolator_.updatePoseJacobians(Hq, H1, H2, H3, H4);
  }
  return err;
}

}  // namespace gtdynamics
