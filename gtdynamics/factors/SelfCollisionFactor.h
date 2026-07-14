/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionFactor.h
 * @brief Self collision cost factor over a set of collision pairs.
 * @author Karthik Shaji - Adapted from gpmp2 by Mustafa Mukadam.
 */

#pragma once

#include <gtdynamics/gpmp2/ObstacleCost.h>
#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtdynamics {

/// One self collision check: a query point vs another query point or a link's
/// field. One side is always a point. Build via PointPair / PointSDF.
struct SelfCollisionPair {
  size_t a;             ///< query point index, the point side
  bool is_sdf;          ///< false: point vs point; true: point vs a link field
  size_t b;             ///< point vs point: the other query point index
  LinkSharedPtr link_b;                              ///< point vs field: link B
  std::shared_ptr<const SignedDistanceField> sdf_b;  ///< field in link B's frame
  double epsilon;       ///< standoff between the two sides, added to their radii

  /// Point vs point pair.
  static SelfCollisionPair PointPair(size_t a, size_t b, double epsilon) {
    return SelfCollisionPair{a, false, b, nullptr, nullptr, epsilon};
  }

  /// Point vs link field pair.
  static SelfCollisionPair PointSDF(
      size_t a, const LinkSharedPtr &link_b,
      const std::shared_ptr<const SignedDistanceField> &sdf_b, double epsilon) {
    return SelfCollisionPair{a, true, 0, link_b, sdf_b, epsilon};
  }
};

using SelfCollisionPairs = std::vector<SelfCollisionPair>;

/**
 * Unary factor keeping the robot clear of itself over a set of collision pairs,
 * one hinge loss row per pair. Both sides of every pair move with q, so build
 * the RobotQueryPoints with the union of every joint involved and a common base
 * so a cross-arm pair couples all their DOFs in one row.
 *
 * The caller registers only meaningful pairs: adjacent links sit at near constant
 * separation and would fire permanently, and two points that can coincide give a
 * non-finite distance gradient. The factor does not auto exclude either.
 */
class SelfCollisionFactor : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = SelfCollisionFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  RobotQueryPoints robot_;
  gtsam::Vector radii_;  ///< one radius per query point, zero if unspecified
  std::vector<SelfCollisionPair> pairs_;

  /// Reject pairs whose indices, radii or fields are inconsistent.
  void validate() const {
    if (static_cast<size_t>(radii_.size()) != robot_.nrPoints()) {
      throw std::invalid_argument(
          "SelfCollisionFactor: radii must have one entry per query point.");
    }
    for (const auto &p : pairs_) {
      if (p.a >= robot_.nrPoints()) {
        throw std::invalid_argument(
            "SelfCollisionFactor: pair point index out of range.");
      }
      if (p.is_sdf) {
        if (!p.link_b || !p.sdf_b) {
          throw std::invalid_argument(
              "SelfCollisionFactor: a point vs field pair needs a link and a "
              "field.");
        }
      } else if (p.b >= robot_.nrPoints()) {
        throw std::invalid_argument(
            "SelfCollisionFactor: pair point index out of range.");
      }
    }
  }

 public:
  /**
   * Constructor with a single sigma across every pair.
   * @param q_key key of the stacked joint angle vector
   * @param robot query point model, spanning every joint involved
   * @param pairs collision pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param cost_sigma cost function sigma, shared by every pair
   */
  SelfCollisionFactor(gtsam::Key q_key, const RobotQueryPoints &robot,
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
   * @param pairs collision pairs to check
   * @param radii radius of each query point, one per point of the model
   * @param sigmas cost function sigma of each pair, one per pair
   */
  SelfCollisionFactor(gtsam::Key q_key, const RobotQueryPoints &robot,
                      const std::vector<SelfCollisionPair> &pairs,
                      const gtsam::Vector &radii, const gtsam::Vector &sigmas)
      : Base(gtsam::noiseModel::Diagonal::Sigmas(sigmas), q_key),
        robot_(robot),
        radii_(radii),
        pairs_(pairs) {
    if (static_cast<size_t>(sigmas.size()) != pairs.size()) {
      throw std::invalid_argument(
          "SelfCollisionFactor: sigmas must have one entry per pair.");
    }
    validate();
  }

  ~SelfCollisionFactor() override {}

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
    const size_t dof = robot_.dof();
    const size_t n = pairs_.size();
    gtsam::Vector err(n);

    // One FK pass supplies both the query points and the link poses.
    std::map<uint8_t, gtsam::Pose3> wTl;
    std::map<uint8_t, gtsam::Matrix> Jl;
    robot_.forwardKinematics(q, &wTl, H1 ? &Jl : nullptr);

    // Transform every query point to the world, keeping the pose map for the
    // field sides (queryPoints would discard it).
    const auto &pts = robot_.points();
    std::vector<gtsam::Point3> wPs(robot_.nrPoints());
    std::vector<gtsam::Matrix> Jps;
    if (H1) Jps.resize(robot_.nrPoints());
    for (size_t i = 0; i < robot_.nrPoints(); ++i) {
      const uint8_t id = pts[i].link->id();
      auto it = wTl.find(id);
      if (it == wTl.end()) {
        throw std::runtime_error(
            "SelfCollisionFactor: query point on a link not reachable from the "
            "base.");
      }
      if (H1) {
        gtsam::Matrix36 H_pose;
        wPs[i] = it->second.transformFrom(pts[i].point, H_pose);
        Jps[i] = H_pose * Jl.at(id);
      } else {
        wPs[i] = it->second.transformFrom(pts[i].point);
      }
    }

    if (H1) *H1 = gtsam::Matrix::Zero(n, dof);

    for (size_t r = 0; r < n; ++r) {
      const SelfCollisionPair &p = pairs_[r];
      if (!p.is_sdf) {
        // Point vs point: standoff folds in both radii.
        const double eps = p.epsilon + radii_(p.a) + radii_(p.b);
        if (H1) {
          gtsam::Matrix13 H_pA, H_pB;
          err(r) = hingeLossSelfCollisionCost(wPs[p.a], wPs[p.b], eps, H_pA,
                                              H_pB);
          H1->row(r) = H_pA * Jps[p.a] + H_pB * Jps[p.b];
        } else {
          err(r) = hingeLossSelfCollisionCost(wPs[p.a], wPs[p.b], eps);
        }
      } else {
        // Point vs field: the field encodes link_b's shape, so only the point
        // side carries a radius.
        const uint8_t bid = p.link_b->id();
        auto itB = wTl.find(bid);
        if (itB == wTl.end()) {
          throw std::runtime_error(
              "SelfCollisionFactor: field link not reachable from the base.");
        }
        const double eps = p.epsilon + radii_(p.a);
        if (H1) {
          gtsam::Matrix16 H_pose;
          gtsam::Matrix13 H_point;
          err(r) = hingeLossObstacleCost(itB->second, wPs[p.a], *p.sdf_b, eps,
                                         H_pose, H_point);
          // Both the point and link_b's frame move with q.
          H1->row(r) = H_point * Jps[p.a] + H_pose * Jl.at(bid);
        } else {
          err(r) = hingeLossObstacleCost(itB->second, wPs[p.a], *p.sdf_b, eps);
        }
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
    std::cout << s << "SelfCollisionFactor with " << pairs_.size() << " pairs"
              << std::endl;
    Base::print("", keyFormatter);
  }
};  // \class SelfCollisionFactor

}  // namespace gtdynamics
