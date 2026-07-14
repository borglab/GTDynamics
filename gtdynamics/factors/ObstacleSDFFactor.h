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

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Unary factor keeping every query point clear of the obstacles in a world
 * frame signed distance field, each point standing off by its radius + epsilon.
 */
class ObstacleSDFFactor : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = ObstacleSDFFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  double epsilon_;
  gtsam::Vector radii_;  ///< per query point radius, zero if unspecified
  RobotQueryPoints robot_;
  std::shared_ptr<const SignedDistanceField> sdf_;

 public:
  /**
   * Constructor with a single standoff for every query point.
   * @param q_key key of the stacked joint angle vector
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param cost_sigma cost function sigma, one per query point
   * @param epsilon standoff distance kept from every obstacle
   */
  ObstacleSDFFactor(gtsam::Key q_key, const RobotQueryPoints &robot,
                    const std::shared_ptr<const SignedDistanceField> &sdf,
                    double cost_sigma, double epsilon)
      : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nrPoints(), cost_sigma),
             q_key),
        epsilon_(epsilon),
        radii_(gtsam::Vector::Zero(robot.nrPoints())),
        robot_(robot),
        sdf_(sdf) {}

  /**
   * Constructor with a radius per query point, added to the shared epsilon.
   * @param q_key key of the stacked joint angle vector
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param cost_sigma cost function sigma, one per query point
   * @param epsilon standoff distance added to every radius
   * @param radii radius of each query point, one per point of the model
   */
  ObstacleSDFFactor(gtsam::Key q_key, const RobotQueryPoints &robot,
                    const std::shared_ptr<const SignedDistanceField> &sdf,
                    double cost_sigma, double epsilon,
                    const gtsam::Vector &radii)
      : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nrPoints(), cost_sigma),
             q_key),
        epsilon_(epsilon),
        radii_(radii),
        robot_(robot),
        sdf_(sdf) {
    if (static_cast<size_t>(radii.size()) != robot.nrPoints()) {
      throw std::invalid_argument(
          "ObstacleSDFFactor: radii must have one entry per query point.");
    }
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
    const size_t m = robot_.nrPoints();
    gtsam::Vector err(m);

    std::vector<gtsam::Point3> wPs;
    std::vector<gtsam::Matrix> Jps;
    robot_.queryPoints(q, &wPs, H1 ? &Jps : nullptr);
    if (H1) *H1 = gtsam::Matrix::Zero(m, robot_.dof());

    for (size_t i = 0; i < m; ++i) {
      const double eps = epsilon_ + radii_(i);
      if (H1) {
        gtsam::Matrix13 Herr_point;
        err(i) = hingeLossObstacleCost(wPs[i], *sdf_, eps, Herr_point);
        H1->row(i) = Herr_point * Jps[i];
      } else {
        err(i) = hingeLossObstacleCost(wPs[i], *sdf_, eps);
      }
    }
    return err;
  }

  /// Return the shared standoff distance.
  double epsilon() const { return epsilon_; }

  /// Return the per query point radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ObstacleSDFFactor with " << robot_.nrPoints()
              << " query points" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class ObstacleSDFFactor

}  // namespace gtdynamics
