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

#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

  /// Reject a null field, a negative standoff, or bad radii.
  void validate() const;

  /// nrPoints of a model that must not be null, for the initializer list.
  static size_t checkedNrPoints(
      const std::shared_ptr<const RobotQueryPoints> &robot) {
    if (!robot) {
      throw std::invalid_argument("ObstacleSDFFactor: robot must not be null.");
    }
    return robot->nrPoints();
  }

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
      : Base(gtsam::noiseModel::Isotropic::Sigma(checkedNrPoints(robot),
                                                 costSigma),
             qKey),
        epsilon_(epsilon),
        radii_(gtsam::Vector::Zero(robot->nrPoints())),
        robot_(robot),
        sdf_(sdf) {
    validate();
  }

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
      : Base(gtsam::noiseModel::Isotropic::Sigma(checkedNrPoints(robot),
                                                 costSigma),
             qKey),
        epsilon_(epsilon),
        radii_(radii),
        robot_(robot),
        sdf_(sdf) {
    validate();
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
      gtsam::OptionalMatrixType H1 = nullptr) const override;

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
