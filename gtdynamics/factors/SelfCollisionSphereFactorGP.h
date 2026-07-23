/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionSphereFactorGP.h
 * @brief Self collision cost factor at a Gaussian process interpolated state.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/factors/SelfCollisionSphereFactor.h>
#include <gtdynamics/gpmp2/GPLinearInterpolator.h>
#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
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
 * Self collision cost evaluated at a state interpolated between two support
 * states, so that self collisions can be checked between the states of a
 * trajectory without adding variables for them. The pairs follow the same
 * semantics and restrictions as SelfCollisionSphereFactor. The interpolation is
 * only the posterior mean of the Gaussian process prior if a GPLinearPrior with
 * the same Qc_model and delta_t connects the same two support states in the
 * graph.
 */
class SelfCollisionSphereFactorGP
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
 private:
  using This = SelfCollisionSphereFactorGP;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector>;

  RobotQueryPoints robot_;
  gtsam::Vector radii_;  ///< one radius per query point, zero if unspecified
  std::vector<SelfCollisionPair> pairs_;
  GPLinearInterpolator GPbase_;

  /// Reject inconsistent indices, radii or standoffs.
  void validate() const {
    validateSelfCollisionPairs(robot_, pairs_, radii_,
                               "SelfCollisionSphereFactorGP");
  }

 public:
  /**
   * Constructor with a single sigma across every pair.
   * @param q_key1 key of the joint angles of the first support state
   * @param v_key1 key of the joint velocities of the first support state
   * @param q_key2 key of the joint angles of the second support state
   * @param v_key2 key of the joint velocities of the second support state
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param cost_sigma cost function sigma, shared by every pair
   * @param Qc_model Gaussian noise model whose covariance is Qc
   * @param delta_t time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  SelfCollisionSphereFactorGP(gtsam::Key q_key1, gtsam::Key v_key1,
                              gtsam::Key q_key2, gtsam::Key v_key2,
                              const RobotQueryPoints &robot,
                              const std::vector<SelfCollisionPair> &pairs,
                              const gtsam::Vector &radii, double cost_sigma,
                              const gtsam::SharedNoiseModel &Qc_model,
                              double delta_t, double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), cost_sigma),
             q_key1, v_key1, q_key2, v_key2),
        robot_(robot),
        radii_(radii),
        pairs_(pairs),
        GPbase_(Qc_model, delta_t, tau) {
    // delta_t and tau are checked by the interpolator constructor.
    validate();
  }

  /**
   * Constructor with a sigma per pair.
   * @param q_key1 key of the joint angles of the first support state
   * @param v_key1 key of the joint velocities of the first support state
   * @param q_key2 key of the joint angles of the second support state
   * @param v_key2 key of the joint velocities of the second support state
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param sigmas cost function sigma of each pair, one per pair
   * @param Qc_model Gaussian noise model whose covariance is Qc
   * @param delta_t time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  SelfCollisionSphereFactorGP(gtsam::Key q_key1, gtsam::Key v_key1,
                              gtsam::Key q_key2, gtsam::Key v_key2,
                              const RobotQueryPoints &robot,
                              const std::vector<SelfCollisionPair> &pairs,
                              const gtsam::Vector &radii,
                              const gtsam::Vector &sigmas,
                              const gtsam::SharedNoiseModel &Qc_model,
                              double delta_t, double tau)
      : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), q_key1, v_key1,
             q_key2, v_key2),
        robot_(robot),
        radii_(radii),
        pairs_(pairs),
        GPbase_(Qc_model, delta_t, tau) {
    if (static_cast<size_t>(sigmas.size()) != pairs.size()) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactorGP: sigmas must have one entry per pair.");
    }
    validate();
  }

  ~SelfCollisionSphereFactorGP() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Return the number of collision pairs.
  size_t nrPairs() const { return pairs_.size(); }

  /// Evaluate the hinge loss of every pair at the interpolated state, and its
  /// Jacobians with respect to the support states.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
      const gtsam::Vector &v2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override {
    const bool use_H = (H1 || H2 || H3 || H4);
    const size_t n = pairs_.size();

    gtsam::Matrix Jq_q1, Jq_v1, Jq_q2, Jq_v2;
    const gtsam::Vector q =
        use_H ? GPbase_.interpolatePose(q1, v1, q2, v2, &Jq_q1, &Jq_v1, &Jq_q2,
                                        &Jq_v2)
              : GPbase_.interpolatePose(q1, v1, q2, v2);

    std::vector<gtsam::Point3> wPs;
    std::vector<gtsam::Matrix> Jps;
    robot_.queryPoints(q, &wPs, use_H ? &Jps : nullptr);

    gtsam::Vector err(n);
    gtsam::Matrix Jerr_q = gtsam::Matrix::Zero(n, robot_.dof());
    for (size_t r = 0; r < n; ++r) {
      const SelfCollisionPair &p = pairs_[r];
      // The standoff folds in both spheres' radii.
      const double eps = p.epsilon + radii_(p.a) + radii_(p.b);
      if (use_H) {
        gtsam::Matrix13 H_pA, H_pB;
        err(r) =
            hingeLossSelfCollisionCost(wPs[p.a], wPs[p.b], eps, H_pA, H_pB);
        Jerr_q.row(r) = H_pA * Jps[p.a] + H_pB * Jps[p.b];
      } else {
        err(r) = hingeLossSelfCollisionCost(wPs[p.a], wPs[p.b], eps);
      }
    }

    if (use_H) {
      GPLinearInterpolator::updatePoseJacobians(Jerr_q, Jq_q1, Jq_v1, Jq_q2,
                                                Jq_v2, H1, H2, H3, H4);
    }
    return err;
  }

  /// Return the per query point radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "SelfCollisionSphereFactorGP with " << pairs_.size()
              << " pairs" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class SelfCollisionSphereFactorGP

}  // namespace gtdynamics
