/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactPointFactor.h
 * @brief Factor to enforce point on link is in contact with environment point.
 * @author: Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

#include "gtdynamics/utils/ContactPoint.h"

namespace gtdynamics {

/**
 * ContactPointFactor is a two-way nonlinear factor which enforces a
 * point on the link to be equal to the point of contact in the environment.
 */
class ContactPointFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 private:
  using This = ContactPointFactor;
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>;

  gtsam::Point3 lPc_;  // The contact point in the link's COM frame.

 public:
  /**
   * Constructor.
   *
   * @param link_pose_key Key for the CoM pose of the link in contact.
   * @param point_key Key for the contact point in the environment.
   * @param cost_model Noise model associated with this factor.
   * @param lPc Static transform from point of contact to link CoM.
   */
  ContactPointFactor(gtsam::Key link_pose_key, gtsam::Key point_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const gtsam::Point3 &lPc)
      : Base(cost_model, link_pose_key, point_key), lPc_(lPc) {}

  /**
   * Convenience constructor which uses PointOnLink.
   *
   * @param point_on_link PointOnLink object which encapsulates the link and its
   * contact point.
   * @param point_key Key for the contact point in the environment.
   * @param cost_model Noise model associated with this factor.
   * @param t The time step at which to add the factor (default t=0).
   */
  ContactPointFactor(const PointOnLink &point_on_link, gtsam::Key point_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     size_t t = 0)
      : Base(cost_model,
             gtdynamics::internal::PoseKey(point_on_link.link->id(), t),
             point_key),
        lPc_(point_on_link.point) {}

  virtual ~ContactPointFactor() {}

  /**
   * Evaluate error.
   * @param wTl The link's COM pose in the world frame.
   * @param wPc The environment contact point in the world frame.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTl, const gtsam::Point3 &wPc,
      boost::optional<gtsam::Matrix &> H_pose = boost::none,
      boost::optional<gtsam::Matrix &> H_point = boost::none) const override {
    gtsam::Vector error = gtsam::traits<gtsam::Point3>::Local(
        wTl.transformFrom(lPc_, H_pose), wPc);
    if (H_pose) *H_pose = -gtsam::Matrix3::Identity() * (*H_pose);
    if (H_point) *H_point = gtsam::Matrix3::Identity();
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "ContactPointFactor"
              << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NonlinearEquality2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics