/**
 * @file  BaseTwistAccelFactor.h
 * @brief Factor enforcing base acceleration.
 * @Author: Frank Dellaert and Mandy Xie
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <vector>

namespace manipulator {

/** BaseTwistAccelFactor is a one-way nonlinear factor which enforces the
 * acceleration of the base*/
class BaseTwistAccelFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 private:
  typedef BaseTwistAccelFactor This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector6> Base;
  gtsam::Vector6 base_twistAccel_;

 public:
  /** Construct Factor enforcing base acceleration.
   *  Keyword argument:
          base_twist_accel -- optional acceleration for base
      Example: if you wish to model gravity forces, use
          base_twist_accel = gtsam::Vector(0, 0, 0, 0, 0, -9.8)
      which imparts upwards acceleration on the base, which then will be
      propagated to all links, forcing wrenches and torques to generate
      upward forces consistent with gravity compensation.
      However, we do not recommand using this method!
   */
  BaseTwistAccelFactor(gtsam::Key twistAccel_key_0,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model,
                       const gtsam::Vector6 &base_twistAccel)
      : Base(cost_model, twistAccel_key_0), base_twistAccel_(base_twistAccel) {}

  virtual ~BaseTwistAccelFactor() {}

 public:
  /** evaluate base acceleration errors
      Keyword argument:
          twistAccel_0  -- twist on the base
  */
  gtsam::Vector evaluateError(
      const gtsam::Vector6 &twistAccel_0,
      boost::optional<gtsam::Matrix &> H_twistAccel_0 = boost::none) const override {
    if (H_twistAccel_0) {
      *H_twistAccel_0 = gtsam::I_6x6;
    }
    return twistAccel_0 - base_twistAccel_;
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
    std::cout << s << "base twist acceleration factor" << std::endl;
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
