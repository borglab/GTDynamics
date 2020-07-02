/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointLimitFactor.h
 * @brief apply joint limit
 * @Author: Frank Dellaert and Mandy Xie
 */
#ifndef GTDYNAMICS_FACTORS_JOINTLIMITFACTOR_H_
#define GTDYNAMICS_FACTORS_JOINTLIMITFACTOR_H_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <boost/optional.hpp>

namespace gtdynamics {

/** JointLimitFactor is a class which enforces joint angle, velocity,
 * acceleration and torque value to be within limit*/
class JointLimitFactor : public gtsam::NoiseModelFactor1<double> {
 private:
  typedef JointLimitFactor This;
  typedef gtsam::NoiseModelFactor1<double> Base;
  double lower_limit_, upper_limit_, limit_threshold_;

 public:
  /**
   * Construct from joint limits
   * Keyword arguments:
      q_key              -- joint value key
      cost_model         -- noise model
      lower_limit        -- joint lower limit
      upper_limit        -- joint upper limit
      limit_threshold    -- joint limit threshold
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
  /** evaluate joint limit errors
      Keyword argument:
          q  -- joint value
      hingloss function:
      error = 0 if q >= lower_limit + limit_threshold and q <= upper_limit -
     limit_threshold error = lower_limit_ + limit_threshold - q if q <
     lower_limit + limit_threshold error = q - upper_limit_ + limit_threshold if
     q > upper_limit + limit_threshold
  */
  gtsam::Vector evaluateError(
      const double &q,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const override {
    if (q < lower_limit_ + limit_threshold_) {
      if (H_q) *H_q = -gtsam::I_1x1;
      return gtsam::Vector1(lower_limit_ + limit_threshold_ - q);
    } else if (q <= upper_limit_ - limit_threshold_) {
      if (H_q) *H_q = gtsam::Z_1x1;
      return gtsam::Vector1(0.0);
    } else {
      if (H_q) *H_q = gtsam::I_1x1;
      return gtsam::Vector1(q - upper_limit_ + limit_threshold_);
    }
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
    std::cout << s << "JointLimitFactor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) { // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics

#endif  // GTDYNAMICS_FACTORS_JOINTLIMITFACTOR_H_
