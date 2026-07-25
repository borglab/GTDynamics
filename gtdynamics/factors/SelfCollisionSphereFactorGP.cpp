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

#include <vector>

namespace gtdynamics {

/* ************************************************************************* */
gtsam::Vector SelfCollisionSphereFactorGP::evaluateError(
    const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
    const gtsam::Vector &v2, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  const bool computeJacobians = (H1 || H2 || H3 || H4);
  const size_t nrPairs = pairs_.size();

  gtsam::Matrix Hq1, Hv1, Hq2, Hv2;
  const gtsam::Vector q =
      computeJacobians ? interpolator_.interpolatePose(q1, v1, q2, v2, &Hq1,
                                                       &Hv1, &Hq2, &Hv2)
                       : interpolator_.interpolatePose(q1, v1, q2, v2);

  std::vector<gtsam::Point3> wPts;
  std::vector<gtsam::Matrix> ptJacobians;
  robot_->queryPoints(q, &wPts, computeJacobians ? &ptJacobians : nullptr);

  gtsam::Vector err(nrPairs);
  gtsam::Matrix errJacobian = gtsam::Matrix::Zero(nrPairs, robot_->dof());
  for (size_t r = 0; r < nrPairs; ++r) {
    const SelfCollisionPair &pair = pairs_[r];
    // The standoff folds in both spheres' radii.
    const double eps = pair.epsilon + radii_(pair.a) + radii_(pair.b);
    if (computeJacobians) {
      gtsam::Matrix13 HptA, HptB;
      err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps,
                                          HptA, HptB);
      errJacobian.row(r) =
          HptA * ptJacobians[pair.a] + HptB * ptJacobians[pair.b];
    } else {
      err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps);
    }
  }

  if (computeJacobians) {
    GPLinearInterpolator::updatePoseJacobians(errJacobian, Hq1, Hv1, Hq2, Hv2,
                                              H1, H2, H3, H4);
  }
  return err;
}

}  // namespace gtdynamics
