/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NNCableFactor.h
 * @brief Obstacle avoidance cost factor for a neural-network-predicted cable.
 * @author Karthik Shaji
 */

#pragma once

#include <gtdynamics/factors/internal/CollisionFactorUtils.h>
#include <gtdynamics/gpmp2/NNCableSpline.h>
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
 * Unary factor that keeps a neural-network-predicted cable clear of a signed
 * distance field, by applying a hinge loss at points sampled along the
 * predicted cable curve. The connected variable is q (stacked joint angles),
 * as ordered in the NNCableSpline model.
 */
class NNCableFactor : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = NNCableFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  double epsilon_;
  gtsam::Vector radii_;  ///< per sample standoff radius, e.g. the cable radius
  std::shared_ptr<const NNCableSpline> cable_;
  std::shared_ptr<const SignedDistanceField> sdf_;

 public:
  /**
   * Constructor with a single cable radius for every sample.
   * @param qKey key of the stacked joint angle vector
   * @param cable cable shape model
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per sample
   * @param epsilon standoff distance kept from every obstacle
   * @param cableRadius radius of the cable, added to epsilon at every sample
   */
  NNCableFactor(gtsam::Key qKey,
                const std::shared_ptr<const NNCableSpline> &cable,
                const std::shared_ptr<const SignedDistanceField> &sdf,
                double costSigma, double epsilon, double cableRadius = 0.0)
      : NNCableFactor(qKey, cable, sdf, costSigma, epsilon,
                      gtsam::Vector::Constant(
                          internal::checkedNumSamples(cable, "NNCableFactor"),
                          cableRadius)) {}

  /**
   * Constructor with a radius per sample, added to the shared epsilon.
   * @param qKey key of the stacked joint angle vector
   * @param cable cable shape model
   * @param sdf signed distance field of the obstacles, in the world frame
   * @param costSigma cost function sigma, one per sample
   * @param epsilon standoff distance added to every radius
   * @param radii standoff radius of each sample, one per sample
   */
  NNCableFactor(gtsam::Key qKey,
                const std::shared_ptr<const NNCableSpline> &cable,
                const std::shared_ptr<const SignedDistanceField> &sdf,
                double costSigma, double epsilon, const gtsam::Vector &radii)
      : Base(gtsam::noiseModel::Isotropic::Sigma(
                 internal::checkedNumSamples(cable, "NNCableFactor"),
                 costSigma),
             qKey),
        epsilon_(epsilon),
        radii_(radii),
        cable_(cable),
        sdf_(sdf) {
    internal::validateNNCableFactorArgs(*cable_, sdf_, epsilon_, radii_,
                                        "NNCableFactor");
  }

  ~NNCableFactor() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the hinge loss at every cable sample, and its Jacobian.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q,
      gtsam::OptionalMatrixType H1 = nullptr) const override {
    return nnCableSDFError(q, *cable_, *sdf_, epsilon_, radii_, H1);
  }

  /// Return the shared standoff distance.
  double epsilon() const { return epsilon_; }

  /// Return the per sample radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "NNCableFactor with " << cable_->numSamples()
              << " cable samples" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class NNCableFactor

}  // namespace gtdynamics
