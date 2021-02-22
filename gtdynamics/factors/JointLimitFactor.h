/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointLimitFactor.h
 * @brief apply joint limit
 * @author: Frank Dellaert and Mandy Xie
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

namespace gtdynamics {

/**
 * JointLimitFactor is a class which enforces joint angle, velocity,
 * acceleration and torque value to be within limits
 */
class JointLimitFactor : public gtsam::NoiseModelFactor1<double> {
 private:
  using This = JointLimitFactor;
  using Base = gtsam::NoiseModelFactor1<double>;
  double lower_limit_, upper_limit_, limit_threshold_;

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
                   const double &lower_limit, const double &upper_limit,
                   const double &limit_threshold)
      : Base(cost_model, q_key),
        lower_limit_(lower_limit),
        upper_limit_(upper_limit),
        limit_threshold_(limit_threshold) {}

  virtual ~JointLimitFactor() {}

 public:
  /**
   * Evaluate joint limit errors
   * @param q joint value
   */
  gtsam::Vector evaluateError(
      const double &q,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    // clang-format off
    // (don't format below documentation)
    /**
     * hinge-loss function:
     *  error = 0 if q >= lower_limit + limit_threshold and q <= upper_limit - limit_threshold
     *  error = lower_limit_ + limit_threshold - q if q < lower_limit + limit_threshold
     *  error = q - upper_limit_ + limit_threshold if q > upper_limit + limit_threshold
     */
    // clang-format on
    gtsam::Vector error(1);

    if (q < lower_limit_ + limit_threshold_) {
      if (H_q) *H_q = -gtsam::I_1x1;
      error << lower_limit_ + limit_threshold_ - q;
    } else if (q <= upper_limit_ - limit_threshold_) {
      if (H_q) *H_q = gtsam::Z_1x1;
      error << 0.0;
    } else {
      if (H_q) *H_q = gtsam::I_1x1;
      error << q - upper_limit_ + limit_threshold_;
    }
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
  }
};

}  // namespace gtdynamics
