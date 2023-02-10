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

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

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

 public:
  // shorthand for a smart pointer to a factor
  using shared_ptr = std::shared_ptr<ContactEqualityFactor>;

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
   */
  ContactEqualityFactor(const PointOnLink &point_on_link,
                        const gtsam::SharedNoiseModel &model, size_t k1,
                        size_t k2)
      : Base(model, PoseKey(point_on_link.link->id(), k1),
             PoseKey(point_on_link.link->id(), k2)),
        point_on_link_(point_on_link) {}

  ~ContactEqualityFactor() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Generic method to compute difference between contact points and provide
  /// jacobians.
  gtsam::Vector3 contactPointsDifference(const gtsam::Pose3 &wT1,
                                         const gtsam::Pose3 &wT2,
                                         gtsam::OptionalMatrixType H1,
                                         gtsam::OptionalMatrixType H2) const {
    gtsam::Point3 p1_w = wT1.transformFrom(point_on_link_.point, H1);
    gtsam::Point3 p2_w = wT2.transformFrom(point_on_link_.point, H2);

    if (H1) *H1 = -(*H1);
    return p2_w - p1_w;
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wT1, const gtsam::Pose3 &wT2,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr) const override {
    return contactPointsDifference(wT1, wT2, H1, H2);
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
           point_on_link_.equals(e->point_on_link_);
  }
};

}  // namespace gtdynamics

namespace gtsam {

template <>
struct traits<gtdynamics::ContactEqualityFactor>
    : public Testable<gtdynamics::ContactEqualityFactor> {};

}  // namespace gtsam
