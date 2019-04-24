/**
 * @file  JointLimitFactor.h
 * @brief apply joint limit
 * @Author: Frank Dellaert and Mandy Xie
 */
#pragma once

#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

namespace manipulator {

class JointLimitFactor : public gtsam::NoiseModelFactor1<double> {
 private:
  typedef JointLimitFactor This;
  typedef gtsam::NoiseModelFactor1<double> Base;
  double lower_limit_, upper_limit_, miu_;

 public:
  /**
   * Construct from joint limits
   * Keyword arguments:
      q_key                 -- joint value key
      cost_model            -- noise model
      lower_limit           -- joint lower limit
      upper_limit           -- joint upper limit
      miu                   -- parameter determine how sharp is the barrier function
   */
  JointLimitFactor(gtsam::LabeledSymbol q_key,
                   const gtsam::noiseModel::Base::shared_ptr &cost_model,
                   const double &lower_limit, const double &upper_limit,
                   const double &miu)
      : Base(cost_model, q_key),
        lower_limit_(lower_limit),
        upper_limit_(upper_limit),
        miu_(miu) {}

  virtual ~JointLimitFactor() {}

 public:
  /** evaluate joint limit errors
      Keyword argument:
        q    -- joint value
  */
  gtsam::Vector evaluateError(
      const double &q,
      boost::optional<gtsam::Matrix &> H_q = boost::none) const {
    if ((q <= lower_limit_) || (q >= upper_limit_)) {
      if (H_q) *H_q = (gtsam::Matrix(1, 1) << 0).finished();
      return gtsam::Vector1(std::numeric_limits<double>::infinity());
    } else if (q <= 0) {
      if (H_q)
        *H_q = (gtsam::Matrix(1, 1) << miu_ / (lower_limit_ - q)).finished();
      return gtsam::Vector1(miu_ *
                            (-log(q - lower_limit_) + log(-lower_limit_)));
    } else {
      if (H_q)
        *H_q = (gtsam::Matrix(1, 1) << miu_ / (upper_limit_ - q)).finished();
      return gtsam::Vector1(miu_ * (-log(upper_limit_ - q) + log(upper_limit_)));
    }
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "JointLimitFactor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace manipulator
