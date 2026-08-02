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
#include <gtdynamics/gpmp2/collisionValidation.h>

namespace gtdynamics {

/* ************************************************************************* */
ObstacleSDFFactorGP::ObstacleSDFFactorGP(
    gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2, gtsam::Key vKey2,
    const std::shared_ptr<const RobotQueryPoints> &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double costSigma,
    double epsilon, const gtsam::SharedNoiseModel &QcModel, double deltaT,
    double tau)
    : ObstacleSDFFactorGP(qKey1, vKey1, qKey2, vKey2, robot, sdf, costSigma,
                          epsilon,
                          gtsam::Vector::Zero(
                              checkedNrPoints(robot, "ObstacleSDFFactorGP")),
                          QcModel, deltaT, tau) {}

/* ************************************************************************* */
ObstacleSDFFactorGP::ObstacleSDFFactorGP(
    gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2, gtsam::Key vKey2,
    const std::shared_ptr<const RobotQueryPoints> &robot,
    const std::shared_ptr<const SignedDistanceField> &sdf, double costSigma,
    double epsilon, const gtsam::Vector &radii,
    const gtsam::SharedNoiseModel &QcModel, double deltaT, double tau)
    : Base(gtsam::noiseModel::Isotropic::Sigma(
               checkedNrPoints(robot, "ObstacleSDFFactorGP"), costSigma),
           qKey1, vKey1, qKey2, vKey2),
      epsilon_(epsilon),
      radii_(radii),
      robot_(robot),
      sdf_(sdf),
      interpolator_(QcModel, deltaT, tau) {
  // deltaT and tau are checked by the interpolator constructor.
  validateObstacleSDFFactorArgs(*robot_, sdf_, epsilon_, radii_,
                                "ObstacleSDFFactorGP");
}

/* ************************************************************************* */
gtsam::Vector ObstacleSDFFactorGP::evaluateError(
    const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
    const gtsam::Vector &v2, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  return interpolator_.errorAtInterpolatedPose(
      q1, v1, q2, v2,
      [this](const gtsam::Vector &q, gtsam::Matrix *Hq) {
        return obstacleSDFError(q, *robot_, *sdf_, epsilon_, radii_, Hq);
      },
      H1, H2, H3, H4);
}

}  // namespace gtdynamics
