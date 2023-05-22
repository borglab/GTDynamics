/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactDynamicsMomentFactor.h
 * @brief Factor to enforce moment at the contact point.
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
 * ContactDynamicsMomentConstraint is a 3-dimensional constraint which enforces
 * zero moment at the contact point for the link.
 */
inline gtsam::Vector3_ ContactDynamicsMomentConstraint(
    gtsam::Key contact_wrench_key, const gtsam::Pose3 &cTcom) {
  gtsam::Matrix36 H_contact_wrench;
  H_contact_wrench << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  gtsam::Matrix36 H =
      H_contact_wrench * cTcom.inverse().AdjointMap().transpose();
  gtsam::Vector6_ contact_wrench(contact_wrench_key);
  const std::function<gtsam::Vector3(gtsam::Vector6)> f =
      [H](const gtsam::Vector6 &F) { return H * F; };
  gtsam::Vector3_ error = gtsam::linearExpression(f, contact_wrench, H);
  return error;
}

/**
 * ContactDynamicsMomentConstraint is a 3-dimensional constraint which enforces
 * zero moment at the contact point for the link. This is an alternative
 * interface for expressions as inputs
 */
inline gtsam::Vector3_ ContactDynamicsMomentConstraint(
    gtsam::Vector6_ contact_wrench, const gtsam::Pose3 &cTcom) {
  gtsam::Matrix36 H_contact_wrench;
  H_contact_wrench << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  gtsam::Matrix36 H =
      H_contact_wrench * cTcom.inverse().AdjointMap().transpose();
  const std::function<gtsam::Vector3(gtsam::Vector6)> f =
      [H](const gtsam::Vector6 &F) { return H * F; };
  gtsam::Vector3_ error = gtsam::linearExpression(f, contact_wrench, H);
  return error;
}

/**
 * ContactDynamicsMomentFactor is unary nonlinear factor which enforces zero
 * moment at the contact point for the link.
 */
class ContactDynamicsMomentFactor
    : public gtsam::ExpressionFactor<gtsam::Vector3> {
 private:
  using This = ContactDynamicsMomentFactor;
  using Base = gtsam::ExpressionFactor<gtsam::Vector3>;

  gtsam::Pose3 cTcom_;
  gtsam::Matrix36 H_contact_wrench_;

 public:
  /**
   * Contact dynamics factor for zero moment at contact.
   *
   * @param contact_wrench_key Key corresponding to this link's contact wrench.
   * @param cost_model Noise model for this factor.
   * @param cTcom Contact frame expressed in com frame.
   */
  ContactDynamicsMomentFactor(
      gtsam::Key contact_wrench_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &cTcom)
      : Base(cost_model, gtsam::Vector3::Zero(),
             ContactDynamicsMomentConstraint(contact_wrench_key, cTcom)) {}

  virtual ~ContactDynamicsMomentFactor() {}

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "Contact Dynamics Moment Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics
