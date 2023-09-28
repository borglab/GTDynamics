/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactKinematicsAccelFactor.h
 * @brief Factor to enforce zero linear acceleration at the contact point.
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
 * ContactKinematicsAccelConstraint is a 3-dimensional constraint which enforces
 * zero linear acceleration at the contact point for a link.
 */
inline gtsam::Vector3_ ContactKinematicsAccelConstraint(
    gtsam::Key accel_key, const gtsam::Pose3 &cTcom) {
  gtsam::Matrix36 H_acc;
  H_acc << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  gtsam::Matrix36 H = H_acc * cTcom.AdjointMap();
  gtsam::Vector6_ accel(accel_key);
  const std::function<gtsam::Vector3(gtsam::Vector6)> f =
      [H](const gtsam::Vector6 &A) { return H * A; };
  gtsam::Vector3_ error = gtsam::linearExpression(f, accel, H);
  return error;
}

/**
 * ContactKinematicsAccelFactor is unary nonlinear factor which enforces zero
 * linear acceleration at the contact point for a link.
 */
class ContactKinematicsAccelFactor
    : public gtsam::ExpressionFactor<gtsam::Vector3> {
 private:
  using This = ContactKinematicsAccelFactor;
  using Base = gtsam::ExpressionFactor<gtsam::Vector3>;

 public:
  /**
   * Contact kinematics factor for zero (linear) acceleration at contact.
   * @param accel_key LabeledKey corresponding to this link's acceleration.
   * @param cost_model Noise model for this factor.
   * @param cTcom Contact frame expressed in com frame.
   */
  ContactKinematicsAccelFactor(
      gtsam::Key accel_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &cTcom)
      : Base(cost_model, gtsam::Vector3::Zero(),
             ContactKinematicsAccelConstraint(accel_key, cTcom)) {}

  virtual ~ContactKinematicsAccelFactor() {}

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
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
