/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactKinematicsTwistFactor.h
 * @brief Factor to enforce zero linear velocity at the contact point.
 * @author: Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/**
 * ContactKinematicsTwistFactor is unary nonlinear factor which enforces zero
 * linear velocity at the contact point for a link.
 */
class ContactKinematicsTwistFactor
    : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 private:
  using This = ContactKinematicsTwistFactor;
  using Base = gtsam::NoiseModelFactor1<gtsam::Vector6>;

  gtsam::Pose3 cTcom_;

 public:
  /**
   * Contact kinematics factor for zero (linear) velocity at contact.
   * @param twist_key LabeledKey corresponding to this link's twist.
   * @param cost_model Noise ,odel for this factor.
   * @param cTcom Contact frame expressed in CoM frame.
   */
  ContactKinematicsTwistFactor(
      gtsam::Key twist_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &cTcom)
      : Base(cost_model, twist_key), cTcom_(cTcom) {}
  virtual ~ContactKinematicsTwistFactor() {}

 public:
  /**
   * Evaluate contact point linear velocity errors.
   * @param twist twist on this link
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twist,
      boost::optional<gtsam::Matrix &> H_twist = boost::none) const override {
    gtsam::Matrix36 H_vel;
    H_vel << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

    // Transform the twist from the link COM frame to the contact frame.
    gtsam::Vector3 error = H_vel * cTcom_.AdjointMap() * twist;

    if (H_twist) *H_twist = H_vel * cTcom_.AdjointMap();

    return error;
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  //< print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "wrench equivalence factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gtdynamics
