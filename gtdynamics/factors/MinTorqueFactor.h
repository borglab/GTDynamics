/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MinTorqueFactor.h
 * @brief Factor to minimize torque.
 * @author Alejandro Escontrela
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <optional>
#include <string>

namespace gtdynamics {

/// MinTorqueFactor is a unary factor which minimizes torque.
class MinTorqueFactor : public gtsam::NoiseModelFactorN<double> {
 private:
  using This = MinTorqueFactor;
  using Base = gtsam::NoiseModelFactorN<double>;

 public:
  /**
   * Torque factor, common between forward and inverse dynamics.
   * This factor computes the error as e = tau^2.
   *
   * @param torque_key Key corresponding the the torque variable for a joint.
   */
  MinTorqueFactor(gtsam::Key torque_key,
                  const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, torque_key) {}

  virtual ~MinTorqueFactor() {}

  /**
   * Evaluate wrench balance errors
   * @param wrench wrench on this link
   * @param torque torque on this link joint
   */
  gtsam::Vector evaluateError(
      const double &torque,
      gtsam::OptionalMatrixType H_torque = nullptr) const override {
    gtsam::Vector error(1);
    error(0) = torque;

    if (H_torque) *H_torque = gtsam::I_1x1;

    return error;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "min torque factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE const &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics
