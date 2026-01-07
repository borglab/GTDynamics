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

#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <string>

namespace gtdynamics {

/**
 * ContactPointFactor is a two-way nonlinear factor which constrains a link pose
 * and a point of contact P in the world/spatial frame by enforcing a point on
 * the link to be equal (and thus, in contact) to P.
 *
 * This factor differs from PointGoalFactor since it also estimates the point of
 * contact.
 */
class ContactPointFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3> {
 private:
  using This = ContactPointFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;

  // The contact point in the link's CoM frame.
  gtsam::Point3 contact_in_com_;

 public:
  /**
   * Constructor.
   *
   * @param link_pose_key Key for the CoM pose of the link in contact.
   * @param point_key Key for the contact point in the environment.
   * @param cost_model Noise model associated with this factor.
   * @param contact_in_com Static transform from point of contact to link CoM.
   */
  ContactPointFactor(gtsam::Key link_pose_key, gtsam::Key point_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const gtsam::Point3 &contact_in_com)
      : Base(cost_model, link_pose_key, point_key),
        contact_in_com_(contact_in_com) {}

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
      : ContactPointFactor(gtdynamics::PoseKey(point_on_link.link->id(), t),
                           point_key, cost_model, point_on_link.point) {}

  virtual ~ContactPointFactor() {}

  /**
   * Evaluate error.
   * @param wTl The link's COM pose in the world frame.
   * @param wPc The environment contact point in the world frame.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTl, const gtsam::Point3 &wPc,
      gtsam::OptionalMatrixType H_pose = nullptr,
      gtsam::OptionalMatrixType H_point = nullptr) const override {
    gtsam::Vector error = wPc - wTl.transformFrom(contact_in_com_, H_pose);
    if (H_pose) *H_pose = -gtsam::Matrix3::Identity() * (*H_pose);
    if (H_point) *H_point = gtsam::Matrix3::Identity();
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
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
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

/**
 * ContactPoseFactor is a two-way nonlinear factor which constrains a link pose
 * and a reference frame defined at a point of contact P in the world/spatial
 * frame.
 *
 * This factor is useful for implementing flat foot constraints.
 */
class ContactPoseFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> {
 private:
  using This = ContactPoseFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;

  // The contact point reference frame in the link's CoM frame.
  gtsam::Pose3 comTcontact_;

 public:
  /**
   * Constructor.
   *
   * @param link_pose_key Key for the CoM pose of the link in contact.
   * @param contact_pose_key Key for the contact point pose in the world/spatial
   * frame.
   * @param cost_model Noise model associated with this factor.
   * @param contact_in_com Static transform from point of contact to link CoM.
   */
  ContactPoseFactor(gtsam::Key link_pose_key, gtsam::Key contact_pose_key,
                    const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    const gtsam::Pose3 &comTcontact)
      : Base(cost_model, link_pose_key, contact_pose_key),
        comTcontact_(comTcontact) {}

  /**
   * Convenience constructor which uses PointOnLink.
   *
   * @param point_on_link PointOnLink object which encapsulates the link and its
   * contact point.
   * @param contact_pose_key Key for the contact point pose in the world/spatial
   * frame.
   * @param cost_model Noise model associated with this factor.
   * @param t The time step at which to add the factor (default t=0).
   */
  ContactPoseFactor(const PointOnLink &point_on_link,
                    gtsam::Key contact_pose_key,
                    const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    size_t t = 0)
      : ContactPoseFactor(
            gtdynamics::PoseKey(point_on_link.link->id(), t), contact_pose_key,
            cost_model,
            // Contact reference frame has same rotation as the link CoM
            gtsam::Pose3(gtsam::Rot3(), point_on_link.point)) {}

  virtual ~ContactPoseFactor() {}

  /**
   * Evaluate error.
   * @param wTl The link's COM pose in the world frame.
   * @param wTcontact The environment contact point frame in the world frame.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTl, const gtsam::Pose3 &wTcontact,
      gtsam::OptionalMatrixType H_link = nullptr,
      gtsam::OptionalMatrixType H_contact = nullptr) const override {
    gtsam::Pose3 measured_wTcontact = wTl.compose(comTcontact_, H_link);
    gtsam::Matrix6 H1, H2;
    gtsam::Vector error =
        measured_wTcontact.localCoordinates(wTcontact, H1, H2);
    if (H_link) *H_link = H1 * (*H_link);
    if (H_contact) *H_contact = H2;
    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "ContactPoseFactor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

/** Contact Point Factor with a static contact point. */
class FixedContactPointFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 private:
  using This = FixedContactPointFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

  // The contact point in the link's CoM frame.
  gtsam::Point3 contact_in_world_;
  gtsam::Point3 contact_in_com_;

 public:
  /**
   * Constructor.
   *
   * @param link_pose_key Key for the CoM pose of the link in contact.
   * @param cost_model Noise model associated with this factor.
   * @param contacet_in_world Static contact point expressed in world frame.
   * @param contact_in_com Static transform from point of contact to link CoM.
   */
  FixedContactPointFactor(gtsam::Key link_pose_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const gtsam::Point3 &contact_in_world,
                          const gtsam::Point3 &contact_in_com)
      : Base(cost_model, link_pose_key),
        contact_in_world_(contact_in_world),
        contact_in_com_(contact_in_com) {}

  virtual ~FixedContactPointFactor() {}

  /**
   * Evaluate error.
   * @param wTl The link's COM pose in the world frame.
   * @param wPc The environment contact point in the world frame.
   */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &wTl,
      gtsam::OptionalMatrixType H_pose = nullptr) const override {
    gtsam::Vector error =
        contact_in_world_ - wTl.transformFrom(contact_in_com_, H_pose);
    if (H_pose) *H_pose = -gtsam::Matrix3::Identity() * (*H_pose);
    return error;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "FixedContactPointFactor"
              << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics
