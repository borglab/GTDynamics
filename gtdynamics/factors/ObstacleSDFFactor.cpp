/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleSDFFactor.cpp
 * @brief Obstacle avoidance cost factor on a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#include <gtdynamics/factors/ObstacleSDFFactor.h>
#include <gtdynamics/factors/internal/CollisionFactorUtils.h>

namespace gtdynamics {

/* ************************************************************************* */
ObstacleSDFFactor::ObstacleSDFFactor(
    gtsam::Key qKey, const std::shared_ptr<const RobotQueryPoints> &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double costSigma,
    double epsilon)
    : ObstacleSDFFactor(qKey, robot, sdf, costSigma, epsilon,
                        gtsam::Vector::Zero(
                            internal::checkedNrPoints(robot, "ObstacleSDFFactor"))) {}

/* ************************************************************************* */
ObstacleSDFFactor::ObstacleSDFFactor(
    gtsam::Key qKey, const std::shared_ptr<const RobotQueryPoints> &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double costSigma,
    double epsilon, const gtsam::Vector &radii)
    : Base(gtsam::noiseModel::Isotropic::Sigma(
               internal::checkedNrPoints(robot, "ObstacleSDFFactor"), costSigma),
           qKey),
      epsilon_(epsilon),
      radii_(radii),
      robot_(robot),
      sdf_(sdf) {
  internal::validateObstacleSDFFactorArgs(*robot_, sdf_, epsilon_, radii_,
                                          "ObstacleSDFFactor");
}

/* ************************************************************************* */
gtsam::Vector ObstacleSDFFactor::evaluateError(
    const gtsam::Vector &q, gtsam::OptionalMatrixType H1) const {
  return obstacleSDFError(q, *robot_, *sdf_, epsilon_, radii_, H1);
}

}  // namespace gtdynamics
