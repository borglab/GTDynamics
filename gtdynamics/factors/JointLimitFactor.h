/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointLimitFactor.h
 * @brief apply joint limit
 * @author: Frank Dellaert, Mandy Xie, Stephanie McCormick, Varun Agrawal
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "gtdynamics/universal_robot/JointTyped.h"

namespace gtdynamics {

/**
 * JointLimitFactor is a class which enforces joint angle, velocity,
 * acceleration and torque value to be within limi
 */
class JointLimitFactor
    : public gtsam::NoiseModelFactor1<JointTyped::JointCoordinateType> {
 private:
  using JointCoordinateType = JointTyped::JointCoordinateType;
  using This = JointLimitFactor;
  using Base = gtsam::NoiseModelFactor1<JointCoordinateType>;
  JointCoordinateType low_, high_;

 public:
  /**
   * Construct from joint limits
   * @param q_key joint value key
   * @param cost_model noise model
   * @param lower_limit joint lower limit
   * @param upper_limit joint upper limit
   * @param limit_threshold joint limit threshold
   */
  JointLimitFactor(gtsam::Key q_key,
                   const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   JointCoordinateType lower_limit,
                   JointCoordinateType upper_limit,
                   JointCoordinateType limit_threshold = 0.0)
      : Base(cost_model, q_key),
        low_(lower_limit + limit_threshold),
        high_(upper_limit - limit_threshold) {}

  virtual ~JointLimitFactor() {}

 public:
  /**
   * Evaluate joint limit errors
   *
   * Hinge loss function:
   *    error = low - q if q < low
   *    error = 0 if q >= low and q <= high
   *    error = q - high if q > high
   *
   * @param q joint value
   */
  gtsam::Vector evaluateError(
      const JointCoordinateType &q,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    if (q < low_) {
      if (H_q) *H_q = -gtsam::I_1x1;
      return gtsam::Vector1(low_ - q);
    } else if (q <= high_) {
      if (H_q) *H_q = gtsam::Z_1x1;
      return gtsam::Vector1(0.0);
    } else {
      if (H_q) *H_q = gtsam::I_1x1;
      return gtsam::Vector1(q - high_);
    }
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
    std::cout << s << "JointLimitFactor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar &low_;
    ar &high_;
  }
};

}  // namespace gtdynamics
