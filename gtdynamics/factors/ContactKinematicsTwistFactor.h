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

#include <gtdynamics/utils/utils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/expressions.h>

#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * ContactKinematicsTwistConstraint is a 3-dimensional constraint which enforces
 * zero linear velocity at the contact point for a link.
 */
inline gtsam::Vector3_ ContactKinematicsTwistConstraint(
    gtsam::Key twist_key, const gtsam::Pose3 &cTcom) {
  gtsam::Matrix36 H_vel;
  H_vel << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  gtsam::Matrix36 H = H_vel * cTcom.AdjointMap();
  gtsam::Vector6_ twist(twist_key);
  const std::function<gtsam::Vector3(gtsam::Vector6)> f =
      [H](const gtsam::Vector6 &V) { return H * V; };
  gtsam::Vector3_ error = gtsam::linearExpression(f, twist, H);
  return error;
}

/**
 * ContactKinematicsTwistFactor is unary nonlinear factor which enforces zero
 * linear velocity at the contact point for a link.
 */
class ContactKinematicsTwistFactor
    : public gtsam::ExpressionFactor<gtsam::Vector3> {
 private:
  using This = ContactKinematicsTwistFactor;
  using Base = gtsam::ExpressionFactor<gtsam::Vector3>;

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
      : Base(cost_model, gtsam::Vector3::Zero(),
             ContactKinematicsTwistConstraint(twist_key, cTcom)) {}
  virtual ~ContactKinematicsTwistFactor() {}

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
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
