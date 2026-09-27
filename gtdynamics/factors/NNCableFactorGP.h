/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NNCableFactorGP.h
 * @brief Obstacle avoidance cost factor for a neural-network-predicted cable
 *        at a Gaussian process interpolated state.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/factors/internal/CollisionFactorUtils.h>
#include <gtdynamics/gpmp2/GPLinearInterpolator.h>
#include <gtdynamics/gpmp2/RobotNNCableModel.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <memory>
#include <string>

namespace gtdynamics {

/**
 * Cable obstacle avoidance cost evaluated at a state interpolated between two
 * support states, so that the predicted cable can be checked between the
 * states of a trajectory without adding variables for them. The interpolation
 * is only the posterior mean of the Gaussian process prior if a GPLinearPrior
 * with the same QcModel and deltaT connects the same two support states in
 * the graph.
 */
class NNCableFactorGP
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
 private:
  using This = NNCableFactorGP;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector>;

  double epsilon_;
  gtsam::Vector radii_;  ///< per sample standoff radius, e.g. the cable radius
  std::shared_ptr<const RobotNNCableModel> cable_;
  std::shared_ptr<const SignedDistanceField> sdf_;
  GPLinearInterpolator interpolator_;

 public:
  /**
   * Constructor with a single cable radius for every sample.
   * @param qKey1 key of the joint angles of the first support state
   * @param vKey1 key of the joint velocities of the first support state
   * @param qKey2 key of the joint angles of the second support state
   * @param vKey2 key of the joint velocities of the second support state
   * @param cable cable shape model
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per sample
   * @param epsilon standoff distance kept from every obstacle
   * @param cableRadius radius of the cable, added to epsilon at every sample
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  NNCableFactorGP(gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2,
                  gtsam::Key vKey2,
                  const std::shared_ptr<const RobotNNCableModel> &cable,
                  const std::shared_ptr<const SignedDistanceField> &sdf,
                  double costSigma, double epsilon, double cableRadius,
                  const gtsam::SharedNoiseModel &QcModel, double deltaT,
                  double tau)
      : NNCableFactorGP(qKey1, vKey1, qKey2, vKey2, cable, sdf, costSigma,
                        epsilon,
                        gtsam::Vector::Constant(
                            internal::checkedNumSamples(cable,
                                                        "NNCableFactorGP"),
                            cableRadius),
                        QcModel, deltaT, tau) {}

  /**
   * Constructor with a radius per sample, added to the shared epsilon.
   * @param qKey1 key of the joint angles of the first support state
   * @param vKey1 key of the joint velocities of the first support state
   * @param qKey2 key of the joint angles of the second support state
   * @param vKey2 key of the joint velocities of the second support state
   * @param cable cable shape model
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per sample
   * @param epsilon standoff distance added to every radius
   * @param radii standoff radius of each sample, one per sample
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  NNCableFactorGP(gtsam::Key qKey1, gtsam::Key vKey1, gtsam::Key qKey2,
                  gtsam::Key vKey2,
                  const std::shared_ptr<const RobotNNCableModel> &cable,
                  const std::shared_ptr<const SignedDistanceField> &sdf,
                  double costSigma, double epsilon, const gtsam::Vector &radii,
                  const gtsam::SharedNoiseModel &QcModel, double deltaT,
                  double tau)
      : Base(gtsam::noiseModel::Isotropic::Sigma(
                 internal::checkedNumSamples(cable, "NNCableFactorGP"),
                 costSigma),
             qKey1, vKey1, qKey2, vKey2),
        epsilon_(epsilon),
        radii_(radii),
        cable_(cable),
        sdf_(sdf),
        interpolator_(QcModel, deltaT, tau) {
    // deltaT and tau are checked by the interpolator constructor.
    internal::validateNNCableFactorArgs(*cable_, sdf_, epsilon_, radii_,
                                        "NNCableFactorGP");
  }

  ~NNCableFactorGP() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the hinge loss at every cable sample at the interpolated state,
  /// and its Jacobians with respect to the support states.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q1, const gtsam::Vector &v1, const gtsam::Vector &q2,
      const gtsam::Vector &v2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override {
    return interpolator_.errorAtInterpolatedPose(
        q1, v1, q2, v2,
        [this](const gtsam::Vector &q, gtsam::Matrix *Hq) {
          return nnCableSDFError(q, *cable_, *sdf_, epsilon_, radii_, Hq);
        },
        H1, H2, H3, H4);
  }

  /// Return the shared standoff distance.
  double epsilon() const { return epsilon_; }

  /// Return the per sample radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "NNCableFactorGP with " << cable_->numSamples()
              << " cable samples" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class NNCableFactorGP

}  // namespace gtdynamics
