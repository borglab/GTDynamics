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

#include <gtdynamics/gpmp2/GPLinearInterpolator.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * Obstacle avoidance cost evaluated at a state interpolated between two support
 * states, so that collisions can be checked between the states of a trajectory
 * without adding variables for them. The interpolation is only the posterior
 * mean of the Gaussian process prior if a GPLinearPrior with the same Qc_model
 * and delta_t connects the same two support states in the graph.
 */
class ObstacleSDFFactorGP
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
 private:
  using This = ObstacleSDFFactorGP;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector>;

  double epsilon_;
  RobotQueryPoints robot_;
  std::shared_ptr<const SignedDistanceField> sdf_;
  GPLinearInterpolator GPbase_;

 public:
  /**
   * Constructor.
   * @param q_key1 key of the joint angles of the first support state
   * @param v_key1 key of the joint velocities of the first support state
   * @param q_key2 key of the joint angles of the second support state
   * @param v_key2 key of the joint velocities of the second support state
   * @param robot query point model of the robot
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param cost_sigma cost function sigma, one per query point
   * @param epsilon standoff distance kept from every obstacle
   * @param Qc_model Gaussian noise model whose covariance is Qc
   * @param delta_t time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  ObstacleSDFFactorGP(gtsam::Key q_key1, gtsam::Key v_key1, gtsam::Key q_key2,
                      gtsam::Key v_key2, const RobotQueryPoints &robot,
                      const std::shared_ptr<const SignedDistanceField> &sdf,
                      double cost_sigma, double epsilon,
                      const gtsam::SharedNoiseModel &Qc_model, double delta_t,
                      double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nrPoints(), cost_sigma),
             q_key1, v_key1, q_key2, v_key2),
        epsilon_(epsilon),
        robot_(robot),
        sdf_(sdf),
        GPbase_(Qc_model, delta_t, tau) {}

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
      gtsam::OptionalMatrixType H4 = nullptr) const override {
    const bool use_H = (H1 || H2 || H3 || H4);
    const size_t m = robot_.nrPoints();

    gtsam::Matrix Jq_q1, Jq_v1, Jq_q2, Jq_v2;
    const gtsam::Vector q =
        use_H ? GPbase_.interpolatePose(q1, v1, q2, v2, &Jq_q1, &Jq_v1, &Jq_q2,
                                        &Jq_v2)
              : GPbase_.interpolatePose(q1, v1, q2, v2);

    std::vector<gtsam::Point3> wPs;
    std::vector<gtsam::Matrix> Jps;
    robot_.queryPoints(q, &wPs, use_H ? &Jps : nullptr);

    gtsam::Vector err(m);
    gtsam::Matrix Jerr_q = gtsam::Matrix::Zero(m, robot_.dof());
    for (size_t i = 0; i < m; ++i) {
      if (use_H) {
        gtsam::Matrix13 Herr_point;
        err(i) = hingeLossObstacleCost(wPs[i], *sdf_, epsilon_, Herr_point);
        Jerr_q.row(i) = Herr_point * Jps[i];
      } else {
        err(i) = hingeLossObstacleCost(wPs[i], *sdf_, epsilon_);
      }
    }

    if (use_H) {
      GPLinearInterpolator::updatePoseJacobians(Jerr_q, Jq_q1, Jq_v1, Jq_q2,
                                                Jq_v2, H1, H2, H3, H4);
    }
    return err;
  }

  /// Return the standoff distance.
  double epsilon() const { return epsilon_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ObstacleSDFFactorGP with " << robot_.nrPoints()
              << " query points" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class ObstacleSDFFactorGP

}  // namespace gtdynamics
