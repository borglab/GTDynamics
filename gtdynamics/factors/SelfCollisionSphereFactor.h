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
 * Unary factor keeping the robot clear of itself over a set of query point
 * pairs, one hinge loss row per pair. Both points of every pair move with q, so
 * build the RobotQueryPoints with the union of every joint involved and a
 * common base, so a cross-arm pair couples all their DOFs in one row.
 *
 * The caller registers only meaningful pairs: points on adjacent links sit at a
 * near constant separation and would fire permanently, and two points that can
 * coincide give a non-finite distance gradient. The factor excludes neither.
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
    if (static_cast<size_t>(radii_.size()) != robot_.nrPoints()) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactor: radii must have one entry per point.");
    }
    if ((radii_.array() < 0.0).any()) {
      throw std::invalid_argument(
          "SelfCollisionSphereFactor: radii must be >= 0.");
    }
    for (const auto &p : pairs_) {
      if (p.epsilon < 0.0) {
        throw std::invalid_argument(
            "SelfCollisionSphereFactor: a pair epsilon must be >= 0.");
      }
      if (p.a >= robot_.nrPoints() || p.b >= robot_.nrPoints()) {
        throw std::invalid_argument(
            "SelfCollisionSphereFactor: pair point index out of range.");
      }
      if (p.a == p.b) {
        throw std::invalid_argument(
            "SelfCollisionSphereFactor: a pair must use two distinct points.");
      }
    }
  }

 public:
  /**
   * Constructor with a single sigma across every pair.
   * @param q_key key of the stacked joint angle vector
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param cost_sigma cost function sigma, shared by every pair
   */
  SelfCollisionSphereFactor(gtsam::Key q_key, const RobotQueryPoints &robot,
                      const std::vector<SelfCollisionPair> &pairs,
                      const gtsam::Vector &radii, double cost_sigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(pairs.size(), cost_sigma),
             q_key),
        robot_(robot),
        radii_(radii),
        pairs_(pairs) {
    validate();
  }

  /**
   * Constructor with a sigma per pair.
   * @param q_key key of the stacked joint angle vector
   * @param robot query point model, spanning every joint involved
   * @param pairs query point pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param sigmas cost function sigma of each pair, one per pair
   */
  SelfCollisionSphereFactor(gtsam::Key q_key, const RobotQueryPoints &robot,
                      const std::vector<SelfCollisionPair> &pairs,
                      const gtsam::Vector &radii, const gtsam::Vector &sigmas)
      : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), q_key),
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
    const size_t n = pairs_.size();
    gtsam::Vector err(n);

    std::vector<gtsam::Point3> wPs;
    std::vector<gtsam::Matrix> Jps;
    robot_.queryPoints(q, &wPs, H1 ? &Jps : nullptr);
    if (H1) *H1 = gtsam::Matrix::Zero(n, robot_.dof());

    for (size_t r = 0; r < n; ++r) {
      const SelfCollisionPair &p = pairs_[r];
      // The standoff folds in both spheres' radii.
      const double eps = p.epsilon + radii_(p.a) + radii_(p.b);
      if (H1) {
        gtsam::Matrix13 H_pA, H_pB;
        err(r) =
            hingeLossSelfCollisionCost(wPs[p.a], wPs[p.b], eps, H_pA, H_pB);
        H1->row(r) = H_pA * Jps[p.a] + H_pB * Jps[p.b];
      } else {
        err(r) = hingeLossSelfCollisionCost(wPs[p.a], wPs[p.b], eps);
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
