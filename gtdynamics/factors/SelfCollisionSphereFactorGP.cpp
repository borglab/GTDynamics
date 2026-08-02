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
#include <gtdynamics/gpmp2/collisionValidation.h>

namespace gtdynamics {

/* ************************************************************************* */
SelfCollisionSphereFactorGP::SelfCollisionSphereFactorGP(
    gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2, gtsam::Key vKey2,
    const std::shared_ptr<const RobotQueryPoints> &robot,
    const SelfCollisionPairs &pairs, const gtsam::Vector &radii,
    double costSigma, const gtsam::SharedNoiseModel &QcModel, double deltaT,
    double tau)
    : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), costSigma),
           qKey1, vKey1, qKey2, vKey2),
      radii_(radii),
      pairs_(pairs),
      interpolator_(QcModel, deltaT, tau) {
  // deltaT and tau are checked by the interpolator constructor.
  robot_ = validateAndRestrictSelfCollision(robot, &pairs_, &radii_,
                                            "SelfCollisionSphereFactorGP");
}

/* ************************************************************************* */
SelfCollisionSphereFactorGP::SelfCollisionSphereFactorGP(
    gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2, gtsam::Key vKey2,
    const std::shared_ptr<const RobotQueryPoints> &robot,
    const SelfCollisionPairs &pairs, const gtsam::Vector &radii,
    const gtsam::Vector &sigmas, const gtsam::SharedNoiseModel &QcModel,
    double deltaT, double tau)
    : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), qKey1, vKey1, qKey2,
           vKey2),
      radii_(radii),
      pairs_(pairs),
      interpolator_(QcModel, deltaT, tau) {
  // deltaT and tau are checked by the interpolator constructor.
  robot_ = validateAndRestrictSelfCollision(
      robot, &pairs_, &radii_, "SelfCollisionSphereFactorGP", &sigmas);
}

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
