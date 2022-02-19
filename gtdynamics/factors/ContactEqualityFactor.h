/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactEqualityFactor.h
 * @brief Factor to constrain temporal contact points to be equal.
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/utils/PointOnLink.h"
#include "gtdynamics/utils/values.h"

namespace gtdynamics {

/**
 * ContactEqualityFactor is a two-way nonlinear factor which computes difference
 * between contact points on the same link at two different time steps.
 */
class ContactEqualityFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  using This = ContactEqualityFactor;
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  /// The point on the link at which to enforce equality.
  PointOnLink point_on_link_;

  /**
   * Flag to enforce the factor. If false, then factor returns zero error.
   * Used primarily for hybrid elimination.
   */
  bool enforce_equality_;

 public:
  // shorthand for a smart pointer to a factor
  using shared_ptr = typename boost::shared_ptr<ContactEqualityFactor>;

  /** default constructor - only use for serialization */
  ContactEqualityFactor(){};

  /**
   * Constructor.
   *
   * @param point_on_link    Point on the link at which to enforce
   * constraint.
   * @param model            The noise model for this factor.
   * @param k1               First time index.
   * @param k2               Subsequent time index.
   * @param enforce_equality Flag indicating if the equality should be
   * enforced.
   */
  ContactEqualityFactor(const PointOnLink &point_on_link,
                        const gtsam::SharedNoiseModel &model, size_t k1,
                        size_t k2, bool enforce_equality = true)
      : Base(model, PoseKey(point_on_link.link->id(), k1),
             PoseKey(point_on_link.link->id(), k2)),
        point_on_link_(point_on_link),
        enforce_equality_(enforce_equality) {}

  ~ContactEqualityFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Generic method to compute difference between contact points and provide
  /// jacobians.
  gtsam::Vector3 contactPointsDifference(
      const gtsam::Pose3 &wT1, const gtsam::Pose3 &wT2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const {
    gtsam::Point3 p1_w = wT1.transformFrom(point_on_link_.point, H1);
    gtsam::Point3 p2_w = wT2.transformFrom(point_on_link_.point, H2);

    if (H1) *H1 = -(*H1);
    return p2_w - p1_w;
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wT1, const gtsam::Pose3 &wT2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
    if (enforce_equality_) {
      return contactPointsDifference(wT1, wT2, H1, H2);
    }
    // If equality is not enforced then set error to zero.
    return gtsam::Vector::Zero(3);
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ContactEqualityFactor(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ")\n";
    this->noiseModel_->print("  noise model: ");
  }

  bool equals(const gtsam::NonlinearFactor &other,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&other);
    return e != nullptr && Base::equals(*e, tol) &&
           point_on_link_.equals(e->point_on_link_) &&
           enforce_equality_ == e->enforce_equality_;
  }
};

}  // namespace gtdynamics

namespace gtsam {

template <>
struct traits<gtdynamics::ContactEqualityFactor>
    : public Testable<gtdynamics::ContactEqualityFactor> {};

}  // namespace gtsam
