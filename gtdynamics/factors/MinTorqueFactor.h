/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MinTorqueFactor.h
 * @brief Factor to minimize torque.
 * @Author: Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

#include <boost/optional.hpp>

namespace gtdynamics {

/** MinTorqueFactor is a unary factor which minimizes torque. */
class MinTorqueFactor : public gtsam::NoiseModelFactor1<double> {
 private:
  typedef MinTorqueFactor This;
  typedef gtsam::NoiseModelFactor1<double> Base;

 public:
  /** torque factor, common between forward and inverse dynamics.
      Keyword argument:
          torque_key -- Key corresponding the the torque variable for a joint.
    
      This factor computes the error as e = tau^2.
   */
  MinTorqueFactor(gtsam::Key torque_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, torque_key) {}
  virtual ~MinTorqueFactor() {}

 public:
  /** evaluate wrench balance errors
      Keyword argument:
          wrench       -- wrench on this link
          torque       -- torque on this link joint
  */
  gtsam::Vector evaluateError(
      const double &torque, boost::optional<gtsam::Matrix &> H_torque
        = boost::none) const override {
    gtsam::Vector error = (gtsam::Vector(1) << torque).finished();

    if (H_torque)
      *H_torque = gtsam::I_1x1;

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
    std::cout << s << "min torque factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};
}  // namespace gtdynamics
