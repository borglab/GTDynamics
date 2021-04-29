/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactEqualityFactor.h
 * @brief Factor to constrain contact point to be equal.
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <string>

#include "gtdynamics/universal_robot/Robot.h"
#include "gtdynamics/utils/ContactPoint.h"
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

  PointOnLink point_on_link_;

 public:
  /**
   * Constructor.
   *
   * @param point_on_link   Point on the link at which to enforce constraint.
   * @param model           The noise model for this factor.
   * @param k1              First time index.
   * @param k2              Next time index.
   */
  ContactEqualityFactor(PointOnLink point_on_link,
                        const gtsam::SharedNoiseModel &model, size_t k1,
                        size_t k2)
      : Base(model, internal::PoseKey(point_on_link.link->id(), k1),
             internal::PoseKey(point_on_link.link->id(), k2)),
        point_on_link_(point_on_link) {}

  virtual ~ContactEqualityFactor() {}

  gtsam::Vector3 constrainPointOnLink(
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
    return constrainPointOnLink(wT1, wT2, H1, H2);
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "ContactEqualityFactor(" << keyFormatter(this->key1())
              << "," << keyFormatter(this->key2()) << ")\n";
    this->noiseModel_->print("  noise model: ");
  }
};

}  // namespace gtdynamics
