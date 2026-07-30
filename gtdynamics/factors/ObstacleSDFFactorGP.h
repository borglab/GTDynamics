/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleSDFFactorGP.h
 * @brief Obstacle avoidance cost factor at a Gaussian process interpolated
 *        state, using a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/factors/ObstacleSDFFactor.h>
#include <gtdynamics/gpmp2/GPLinearInterpolator.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Obstacle avoidance cost evaluated at a state interpolated between two support
 * states, so that collisions can be checked between the states of a trajectory
 * without adding variables for them. The interpolation is only the posterior
 * mean of the Gaussian process prior if a GPLinearPrior with the same QcModel
 * and deltaT connects the same two support states in the graph.
 */
class GTSAM_EXPORT ObstacleSDFFactorGP
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
 private:
  using This = ObstacleSDFFactorGP;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector>;

  double epsilon_;
  gtsam::Vector radii_;  ///< one radius per query point, zero if unspecified
  std::shared_ptr<const RobotQueryPoints> robot_;
  std::shared_ptr<const SignedDistanceField> sdf_;
  GPLinearInterpolator interpolator_;

 public:
  /**
   * Constructor with a single standoff for every query point.
   * @param qKey1 key of the joint angles of the first support state
   * @param vKey1 key of the joint velocities of the first support state
   * @param qKey2 key of the joint angles of the second support state
   * @param vKey2 key of the joint velocities of the second support state
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per query point
   * @param epsilon standoff distance kept from every obstacle
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  ObstacleSDFFactorGP(gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2,
                      gtsam::Key vKey2,
                      const std::shared_ptr<const RobotQueryPoints> &robot,
                      const std::shared_ptr<const SignedDistanceField> &sdf,
                      double costSigma, double epsilon,
                      const gtsam::SharedNoiseModel &QcModel, double deltaT,
                      double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(
                 checkedNrPoints(robot, "ObstacleSDFFactorGP"), costSigma),
             qKey1, vKey1, qKey2, vKey2),
        epsilon_(epsilon),
        radii_(gtsam::Vector::Zero(robot->nrPoints())),
        robot_(robot),
        sdf_(sdf),
        interpolator_(QcModel, deltaT, tau) {
    // deltaT and tau are checked by the interpolator constructor.
    validateObstacleSDFFactorArgs(*robot_, sdf_, epsilon_, radii_,
                                  "ObstacleSDFFactorGP");
  }

  /**
   * Constructor with a radius per query point, added to the shared epsilon.
   * @param qKey1 key of the joint angles of the first support state
   * @param vKey1 key of the joint velocities of the first support state
   * @param qKey2 key of the joint angles of the second support state
   * @param vKey2 key of the joint velocities of the second support state
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per query point
   * @param epsilon standoff distance added to every radius
   * @param radii radius of each query point, one per point of the model
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  ObstacleSDFFactorGP(gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2,
                      gtsam::Key vKey2,
                      const std::shared_ptr<const RobotQueryPoints> &robot,
                      const std::shared_ptr<const SignedDistanceField> &sdf,
                      double costSigma, double epsilon,
                      const gtsam::Vector &radii,
                      const gtsam::SharedNoiseModel &QcModel, double deltaT,
                      double tau)
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

  ~ObstacleSDFFactorGP() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the hinge loss at the interpolated state, and its Jacobians.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
      const gtsam::Vector &v2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override;

  /// Return the standoff distance.
  double epsilon() const { return epsilon_; }

  /// Return the per query point radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ObstacleSDFFactorGP with " << robot_->nrPoints()
              << " query points" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class ObstacleSDFFactorGP

}  // namespace gtdynamics
