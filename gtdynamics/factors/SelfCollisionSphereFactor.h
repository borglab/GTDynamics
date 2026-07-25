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
inline void validateSelfCollisionPairs(const RobotQueryPoints &robot,
                                       const SelfCollisionPairs &pairs,
                                       const gtsam::Vector &radii,
                                       const std::string &factorName) {
  if (static_cast<size_t>(radii.size()) != robot.nrPoints()) {
    throw std::invalid_argument(
        factorName + ": radii must have one entry per point.");
  }
  if ((radii.array() < 0.0).any()) {
    throw std::invalid_argument(factorName + ": radii must be >= 0.");
  }
  for (const auto &pair : pairs) {
    if (pair.epsilon < 0.0) {
      throw std::invalid_argument(factorName +
                                  ": a pair epsilon must be >= 0.");
    }
    if (pair.a >= robot.nrPoints() || pair.b >= robot.nrPoints()) {
      throw std::invalid_argument(factorName +
                                  ": pair point index out of range.");
    }
    if (pair.a == pair.b) {
      throw std::invalid_argument(factorName +
                                  ": a pair must use two distinct points.");
    }
    // Points on one rigid link keep a constant separation, so the hinge has
    // no gradient; reject such a pair rather than fire it permanently.
    if (robot.points()[pair.a].link->id() ==
        robot.points()[pair.b].link->id()) {
      throw std::invalid_argument(
          factorName + ": a pair must use points on different links.");
    }
  }
}

/**
 * Unary factor keeping the robot clear of itself over a set of query point
 * pairs, one hinge loss row per pair. Both points of every pair move with q, so
 * build the RobotQueryPoints with the union of every joint involved and a
 * common base, so a cross-arm pair couples all their DOFs in one row.
 *
 * Same-link pairs are rejected; the caller must also avoid adjacent-link pairs
 * (constant separation) and points that can coincide (non-finite gradient).
 */
class SelfCollisionSphereFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = SelfCollisionSphereFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  RobotQueryPoints robot_;
  gtsam::Vector radii_;  ///< one radius per query point, zero if unspecified
  std::vector<SelfCollisionPair> pairs_;

  /// Reject inconsistent indices, radii or standoffs.
  void validate() const {
    validateSelfCollisionPairs(robot_, pairs_, radii_,
                               "SelfCollisionSphereFactor");
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
  SelfCollisionSphereFactor(gtsam::Key qKey, const RobotQueryPoints &robot,
                      const std::vector<SelfCollisionPair> &pairs,
                      const gtsam::Vector &radii, double costSigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), costSigma),
             qKey),
        robot_(robot),
        radii_(radii),
        pairs_(pairs) {
    validate();
  }

  /**
   * Constructor with a sigma per pair.
   * @param qKey key of the stacked joint angle vector
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param sigmas cost function sigma of each pair, one per pair
   */
  SelfCollisionSphereFactor(gtsam::Key qKey, const RobotQueryPoints &robot,
                      const std::vector<SelfCollisionPair> &pairs,
                      const gtsam::Vector &radii, const gtsam::Vector &sigmas)
      : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), qKey),
        robot_(robot),
        radii_(radii),
        pairs_(pairs) {
    if (static_cast<size_t>(sigmas.size()) != pairs.size()) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactor: sigmas must have one entry per pair.");
    }
    validate();
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
      gtsam::OptionalMatrixType H1 = nullptr) const override {
    const size_t nrPairs = pairs_.size();
    gtsam::Vector err(nrPairs);

    std::vector<gtsam::Point3> wPts;
    std::vector<gtsam::Matrix> ptJacobians;
    robot_.queryPoints(q, &wPts, H1 ? &ptJacobians : nullptr);
    if (H1) *H1 = gtsam::Matrix::Zero(nrPairs, robot_.dof());

    for (size_t r = 0; r < nrPairs; ++r) {
      const SelfCollisionPair &pair = pairs_[r];
      // The standoff folds in both spheres' radii.
      const double eps = pair.epsilon + radii_(pair.a) + radii_(pair.b);
      if (H1) {
        gtsam::Matrix13 HptA, HptB;
        err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps,
                                            HptA, HptB);
        H1->row(r) = HptA * ptJacobians[pair.a] + HptB * ptJacobians[pair.b];
      } else {
        err(r) = hingeLossSelfCollisionCost(wPts[pair.a], wPts[pair.b], eps);
      }
    }
    return err;
  }

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
