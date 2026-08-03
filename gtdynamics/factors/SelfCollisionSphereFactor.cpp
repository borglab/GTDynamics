/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionSphereFactor.cpp
 * @brief Self collision cost factor over a set of query point pairs.
 * @author Karthik Shaji - Adapted from gpmp2 by Mustafa Mukadam.
 */

#include <gtdynamics/factors/SelfCollisionSphereFactor.h>
#include <gtdynamics/factors/internal/CollisionFactorUtils.h>

namespace gtdynamics {

/* ************************************************************************* */
SelfCollisionSphereFactor::SelfCollisionSphereFactor(
    gtsam::Key qKey, const std::shared_ptr<const RobotQueryPoints> &robot,
    const SelfCollisionPairs &pairs, const gtsam::Vector &radii,
    double costSigma)
    : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), costSigma), qKey),
      radii_(radii),
      pairs_(pairs) {
  robot_ = internal::validateAndRestrictSelfCollision(
      robot, &pairs_, &radii_, "SelfCollisionSphereFactor");
}

/* ************************************************************************* */
SelfCollisionSphereFactor::SelfCollisionSphereFactor(
    gtsam::Key qKey, const std::shared_ptr<const RobotQueryPoints> &robot,
    const SelfCollisionPairs &pairs, const gtsam::Vector &radii,
    const gtsam::Vector &sigmas)
    : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), qKey),
      radii_(radii),
      pairs_(pairs) {
  robot_ = internal::validateAndRestrictSelfCollision(
      robot, &pairs_, &radii_, "SelfCollisionSphereFactor", &sigmas);
}

/* ************************************************************************* */
gtsam::Vector SelfCollisionSphereFactor::evaluateError(
    const gtsam::Vector &q, gtsam::OptionalMatrixType H1) const {
  return selfCollisionError(q, *robot_, pairs_, radii_, H1);
}

}  // namespace gtdynamics
