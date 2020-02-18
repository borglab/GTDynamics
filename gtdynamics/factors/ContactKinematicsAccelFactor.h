/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContactKinematicsAccelFactor.h
 * @brief Factor to enforce zero linear acceleration at the contact point.
 * @Author: Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include "gtdynamics/utils/utils.h"

namespace gtdynamics {

/** ContactKinematicsAccelFactor is unary nonlinear factor which enforces
 *  zero linear acceleration at the contact point for a link. */
class ContactKinematicsAccelFactor
    : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 private:
  typedef ContactKinematicsAccelFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector6> Base;
  gtsam::Pose3 cTcom_;

 public:
  /** Contact kinematics factor for zero (linear) acceleration at contact.
      Keyword argument:
          accel_key  -- gtsam::LabeledKey corresponding to this link's
            acceleration.
          cost_model -- gtsam::noiseModel for this factor.
          cTcom      -- Contact frame expressed in com frame.
   */
  ContactKinematicsAccelFactor(
      gtsam::Key accel_key,
      const gtsam::noiseModel::Base::shared_ptr &cost_model,
      const gtsam::Pose3 &cTcom)
      : Base(cost_model, accel_key), cTcom_(cTcom) {}
  virtual ~ContactKinematicsAccelFactor() {}

 public:
  /** Evaluate contact point linear acceleration errors.
      Keyword argument:
          twist         -- acceleration on this link
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &accel,
      boost::optional<gtsam::Matrix &> H_accel = boost::none) const override {
    gtsam::Matrix36 H_acc;
    H_acc << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

    // Transform the twist from the link COM frame to the contact frame.
    gtsam::Vector3 error = H_acc * cTcom_.AdjointMap() * accel;

    if (H_accel) *H_accel = H_acc * cTcom_.AdjointMap();

    return error;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "wrench equivalence factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
