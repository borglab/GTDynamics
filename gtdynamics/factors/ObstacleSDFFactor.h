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
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Unary factor keeping every query point on the robot at least epsilon away
 * from the obstacles encoded in a world frame signed distance field. The error
 * has one row per query point, each the hinge loss epsilon - d(wP).
 */
class ObstacleSDFFactor : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = ObstacleSDFFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  double epsilon_;
  RobotQueryPoints robot_;
  std::shared_ptr<const SignedDistanceField> sdf_;

 public:
  /**
   * Constructor.
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
        robot_(robot),
        sdf_(sdf) {}

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
      if (H1) {
        gtsam::Matrix13 Herr_point;
        err(i) = hingeLossObstacleCost(wPs[i], *sdf_, epsilon_, Herr_point);
        H1->row(i) = Herr_point * Jps[i];
      } else {
        err(i) = hingeLossObstacleCost(wPs[i], *sdf_, epsilon_);
      }
    }
    return err;
  }

  /// Return the standoff distance.
  double epsilon() const { return epsilon_; }

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
