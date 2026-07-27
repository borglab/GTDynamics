/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionSphereFactor.h
 * @brief Self collision cost factor over a set of query point pairs.
 * @author Karthik Shaji - Adapted from gpmp2 by Mustafa Mukadam.
 */

#pragma once

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

/// One self collision check between two query points, each inflated to a sphere
/// by its radius and kept epsilon apart on top of that.
struct SelfCollisionPair {
  size_t a;        ///< first query point index
  size_t b;        ///< second query point index
  double epsilon;  ///< standoff between the two, added to their radii

  SelfCollisionPair() : a(0), b(0), epsilon(0.0) {}
  SelfCollisionPair(size_t a, size_t b, double epsilon)
      : a(a), b(b), epsilon(epsilon) {}
};

using SelfCollisionPairs = std::vector<SelfCollisionPair>;

/**
 * Reject inconsistent indices, radii or standoffs, shared by every self
 * collision factor. factorName prefixes the error messages.
 */
GTSAM_EXPORT void validateSelfCollisionPairs(const RobotQueryPoints &robot,
                                             const SelfCollisionPairs &pairs,
                                             const gtsam::Vector &radii,
                                             const std::string &factorName);

/**
 * Restrict robot to just the points pairs references, and remap pairs/radii
 * from robot's point indices into the restricted model's own compact ones.
 * Shared by every self collision factor so evaluation only ever queries the
 * points a factor actually checks, not every point of a larger shared model.
 * Mutates *pairs and *radii in place; robot itself is left untouched.
 */
GTSAM_EXPORT std::shared_ptr<const RobotQueryPoints> restrictToReferencedPoints(
    const std::shared_ptr<const RobotQueryPoints> &robot,
    SelfCollisionPairs *pairs, gtsam::Vector *radii);

/**
 * Unary factor keeping the robot clear of itself over a set of query point
 * pairs, one hinge loss row per pair. Both points of every pair move with q, so
 * build the RobotQueryPoints with the union of every joint involved and a
 * common base, so a cross-arm pair couples all their DOFs in one row.
 *
 * Same-link pairs are rejected; the caller must also avoid adjacent-link pairs
 * (constant separation). Coincident points fall back to a fixed separation
 * direction rather than a non-finite gradient.
 */
class GTSAM_EXPORT SelfCollisionSphereFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = SelfCollisionSphereFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  std::shared_ptr<const RobotQueryPoints> robot_;
  gtsam::Vector radii_;  ///< one radius per query point, zero if unspecified
  SelfCollisionPairs pairs_;

  /// Reject a null model or inconsistent indices/radii/standoffs (checked
  /// against robot's own point indices), then restrict robot_ to just the
  /// points pairs_ references and remap pairs_/radii_ to match.
  void validateAndRestrict(const std::shared_ptr<const RobotQueryPoints> &robot) {
    if (!robot) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactor: robot must not be null.");
    }
    validateSelfCollisionPairs(*robot, pairs_, radii_,
                               "SelfCollisionSphereFactor");
    robot_ = restrictToReferencedPoints(robot, &pairs_, &radii_);
  }

 public:
  /**
   * Constructor with a single sigma across every pair.
   * @param qKey key of the stacked joint angle vector
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param costSigma cost function sigma, shared by every pair
   */
  SelfCollisionSphereFactor(gtsam::Key qKey,
                      const std::shared_ptr<const RobotQueryPoints> &robot,
                      const SelfCollisionPairs &pairs,
                      const gtsam::Vector &radii, double costSigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), costSigma),
             qKey),
        radii_(radii),
        pairs_(pairs) {
    validateAndRestrict(robot);
  }

  /**
   * Constructor with a sigma per pair.
   * @param qKey key of the stacked joint angle vector
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param sigmas cost function sigma of each pair, one per pair
   */
  SelfCollisionSphereFactor(gtsam::Key qKey,
                      const std::shared_ptr<const RobotQueryPoints> &robot,
                      const SelfCollisionPairs &pairs,
                      const gtsam::Vector &radii, const gtsam::Vector &sigmas)
      : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), qKey),
        radii_(radii),
        pairs_(pairs) {
    if (static_cast<size_t>(sigmas.size()) != pairs.size()) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactor: sigmas must have one entry per pair.");
    }
    validateAndRestrict(robot);
  }

  ~SelfCollisionSphereFactor() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Return the number of collision pairs.
  size_t nrPairs() const { return pairs_.size(); }

  /// Evaluate the hinge loss of every pair, and its Jacobian.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q,
      gtsam::OptionalMatrixType H1 = nullptr) const override;

  /// Return the per query point radii.
  const gtsam::Vector &radii() const { return radii_; }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "SelfCollisionSphereFactor with " << pairs_.size()
              << " pairs" << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class SelfCollisionSphereFactor

}  // namespace gtdynamics
