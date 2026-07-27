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
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SelfCollisionCost.h>
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
 * the same QcModel and deltaT connects the same two support states in the
 * graph.
 */
class GTSAM_EXPORT SelfCollisionSphereFactorGP
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
 private:
  using This = SelfCollisionSphereFactorGP;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector>;

  std::shared_ptr<const RobotQueryPoints> robot_;
  gtsam::Vector radii_;  ///< one radius per query point, zero if unspecified
  SelfCollisionPairs pairs_;
  GPLinearInterpolator interpolator_;

  /// Reject a null model and inconsistent indices, radii or standoffs.
  void validate() const {
    if (!robot_) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactorGP: robot must not be null.");
    }
    validateSelfCollisionPairs(*robot_, pairs_, radii_,
                               "SelfCollisionSphereFactorGP");
  }

 public:
  /**
   * Constructor with a single sigma across every pair.
   * @param qKey1 key of the joint angles of the first support state
   * @param vKey1 key of the joint velocities of the first support state
   * @param qKey2 key of the joint angles of the second support state
   * @param vKey2 key of the joint velocities of the second support state
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param costSigma cost function sigma, shared by every pair
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  SelfCollisionSphereFactorGP(gtsam::Key qKey1, gtsam::Key vKey1,
                              gtsam::Key qKey2, gtsam::Key vKey2,
                              const std::shared_ptr<const RobotQueryPoints> &robot,
                              const SelfCollisionPairs &pairs,
                              const gtsam::Vector &radii, double costSigma,
                              const gtsam::SharedNoiseModel &QcModel,
                              double deltaT, double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), costSigma),
             qKey1, vKey1, qKey2, vKey2),
        robot_(robot),
        radii_(radii),
        pairs_(pairs),
        interpolator_(QcModel, deltaT, tau) {
    // deltaT and tau are checked by the interpolator constructor.
    validate();
  }

  /**
   * Constructor with a sigma per pair.
   * @param qKey1 key of the joint angles of the first support state
   * @param vKey1 key of the joint velocities of the first support state
   * @param qKey2 key of the joint angles of the second support state
   * @param vKey2 key of the joint velocities of the second support state
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param sigmas cost function sigma of each pair, one per pair
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  SelfCollisionSphereFactorGP(gtsam::Key qKey1, gtsam::Key vKey1,
                              gtsam::Key qKey2, gtsam::Key vKey2,
                              const std::shared_ptr<const RobotQueryPoints> &robot,
                              const SelfCollisionPairs &pairs,
                              const gtsam::Vector &radii,
                              const gtsam::Vector &sigmas,
                              const gtsam::SharedNoiseModel &QcModel,
                              double deltaT, double tau)
      : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), qKey1, vKey1,
             qKey2, vKey2),
        robot_(robot),
        radii_(radii),
        pairs_(pairs),
        interpolator_(QcModel, deltaT, tau) {
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
      gtsam::OptionalMatrixType H4 = nullptr) const override;

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
