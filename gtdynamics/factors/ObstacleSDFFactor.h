/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleSDFFactor.h
 * @brief Obstacle avoidance cost factor on a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/factors/internal/CollisionFactorUtils.h>
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
#include <string>

namespace gtdynamics {

/**
 * Unary factor that acts to prevent collision of a robot with obstacles, by
 * keeping every query point clear of a given signed distance field. 
 * The connected variable is q (stacked set of joint angles), as ordered in the 
 * RobotQueryPoints model. 
 * The signed distance field has positive values outside obstacles, negative values 
 * inside obstacles, and can be further offset by a standoff distance (epsilon).
 */
class ObstacleSDFFactor : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = ObstacleSDFFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  double epsilon_;
  gtsam::Vector radii_;  ///< per query point radius, zero if unspecified
  std::shared_ptr<const RobotQueryPoints> robot_;
  std::shared_ptr<const SignedDistanceField> sdf_;

 public:
  /**
   * Constructor with a single standoff for every query point.
   * @param qKey key of the stacked joint angle vector
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per query point
   * @param epsilon standoff distance kept from every obstacle
   */
  ObstacleSDFFactor(gtsam::Key qKey,
                    const std::shared_ptr<const RobotQueryPoints> &robot,
                    const std::shared_ptr<const SignedDistanceField> &sdf,
                    double costSigma, double epsilon)
      : ObstacleSDFFactor(qKey, robot, sdf, costSigma, epsilon,
                          gtsam::Vector::Zero(internal::checkedNrPoints(
                              robot, "ObstacleSDFFactor"))) {}

  /**
   * Constructor with a radius per query point, added to the shared epsilon.
   * @param qKey key of the stacked joint angle vector
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per query point
   * @param epsilon standoff distance added to every radius
   * @param radii radius of each query point, one per point of the model
   */
  ObstacleSDFFactor(gtsam::Key qKey,
                    const std::shared_ptr<const RobotQueryPoints> &robot,
                    const std::shared_ptr<const SignedDistanceField> &sdf,
                    double costSigma, double epsilon,
                    const gtsam::Vector &radii)
      : Base(gtsam::noiseModel::Isotropic::Sigma(
                 internal::checkedNrPoints(robot, "ObstacleSDFFactor"),
                 costSigma),
             qKey),
        epsilon_(epsilon),
        radii_(radii),
        robot_(robot),
        sdf_(sdf) {
    internal::validateObstacleSDFFactorArgs(*robot_, sdf_, epsilon_, radii_,
                                            "ObstacleSDFFactor");
  }

  ~ObstacleSDFFactor() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the hinge loss at every query point, and its Jacobian.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q,
      gtsam::OptionalMatrixType H1 = nullptr) const override {
    return obstacleSDFError(q, *robot_, *sdf_, epsilon_, radii_, H1);
  }

  /// Return the shared standoff distance.
  double epsilon() const { return epsilon_; }

  /// Return the per query point radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ObstacleSDFFactor with " << robot_->nrPoints()
              << " query points" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class ObstacleSDFFactor

}  // namespace gtdynamics
