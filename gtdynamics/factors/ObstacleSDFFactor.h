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

  /// Reject a null field, a negative standoff, or bad radii.
  void validate() const {
    if (!sdf_) {
      throw std::invalid_argument("ObstacleSDFFactor: sdf must not be null.");
    }
    if (epsilon_ < 0.0) {
      throw std::invalid_argument("ObstacleSDFFactor: epsilon must be >= 0.");
    }
    if (static_cast<size_t>(radii_.size()) != robot_.nrPoints()) {
      throw std::invalid_argument(
          "ObstacleSDFFactor: radii must have one entry per query point.");
    }
    if ((radii_.array() < 0.0).any()) {
      throw std::invalid_argument("ObstacleSDFFactor: radii must be >= 0.");
    }
    // Overlapping spheres on a link are fine, but the same point registered
    // twice with different radii is a contradiction. Group by link so only
    // same-link points are compared, not every pair.
    const auto &pts = robot_.points();
    std::map<uint8_t, std::vector<size_t>> by_link;
    for (size_t i = 0; i < pts.size(); ++i) {
      by_link[pts[i].link->id()].push_back(i);
    }
    for (const auto &group : by_link) {
      const std::vector<size_t> &idx = group.second;
      for (size_t a = 0; a < idx.size(); ++a) {
        for (size_t b = a + 1; b < idx.size(); ++b) {
          if ((pts[idx[a]].point - pts[idx[b]].point).norm() < 1e-9 &&
              std::fabs(radii_(idx[a]) - radii_(idx[b])) > 1e-9) {
            throw std::invalid_argument(
                "ObstacleSDFFactor: two points at the same location have "
                "conflicting radii.");
          }
        }
      }
    }
  }

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
        sdf_(sdf) {
    validate();
  }

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
