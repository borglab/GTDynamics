/**
 * @file WinchFactor.h
 * @brief Maps motor input torque to cable tension
 * @author Gerry Chen
 * 
 * Applies a mapping from input torque to cable tension based on:
 *  - winch radius
 *  - motor inertia
 *  - static friction
 *  - viscous friction
 * 
 * Input torque is given by (current) * (motor torque constant)
 * External torques are given by
 *    (tension) * (radius) + 
 *    (static friction) +
 *    (viscous friction)
 * These together cause:
 *    (motor inertia) * (angular acceleration)
 **/
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

namespace gtdynamics {

/**
 * WinchParams holds parameters relevant to winches
 */
struct WinchParams {
  double radius_;           // winch radius in m
  double inertia_;          // motor inertia in kg.m^2
  double staticFriction_;   // static friction of the winch in N.m
  double viscousFriction_;  // viscous friction of the winch in N.m / rad
  WinchParams(double radius, double inertia, double staticFriction,
              double viscousFriction)
      : radius_(radius),
        inertia_(inertia),
        staticFriction_(staticFriction),
        viscousFriction_(viscousFriction) {}
};

/**
 * WinchFactor is a 4-way factor which converts motor input torque to cable
 * tension, taking into account winch radius, inertia, and friction.
 * 
 * The relationship is given by
 *  $$ tau = tension * radius - inertia * acceleration / radius + friction * radius $$
 * where
 *  $$ friction = - (static * sign(v) + viscous * v) $$
 * 
 * This is just rewriting $\sum{torques} = I * alpha$ and adjusting for sign
 * convention (torque is backwards...)
 * 
 * Note that static friction causes a major discontinuity, so instead it is
 * approximated as tanh(v/eps) as is done in Gouttefarde15TRO, with eps=1/50.
 */
class WinchFactor
    : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  typedef WinchFactor This;
  typedef gtsam::NoiseModelFactor4<double, double, double, double> Base;
  const WinchParams params_;

 public:
  /** Winch factor
   * @param torque -- key for motor torque input
   * @param tenison -- key for cable tension
   * @param cableVelocity -- key for cable velocity
   * @param cableAcceleration -- key for cable acceleration
   * @param cost_model -- noise model (1 dimensional)
   * @param params -- winch parameters
   */
  WinchFactor(gtsam::Key torque, gtsam::Key tension, gtsam::Key cableVelocity,
              gtsam::Key cableAcceleration,
              const gtsam::noiseModel::Base::shared_ptr &cost_model,
              WinchParams params)
      : Base(cost_model, torque, tension, cableVelocity, cableAcceleration),
        params_(params) {}
  virtual ~WinchFactor() {}

 public:
  /** Winch factor
   * @param torque -- motor torque input
   * @param tension -- cable tension
   * @param vel -- cable velocity
   * @param acc -- cable acceleration
   * @return cable torque minus computed cable torque
   */
  gtsam::Vector evaluateError(
      const double &torque, const double &tension, const double &vel,
      const double &acc,
      boost::optional<gtsam::Matrix &> H_torque = boost::none,
      boost::optional<gtsam::Matrix &> H_tension = boost::none,
      boost::optional<gtsam::Matrix &> H_vel = boost::none,
      boost::optional<gtsam::Matrix &> H_acc = boost::none) const override {
    const double &r = params_.radius_;
    const double &I = params_.inertia_;
    const double &mu = params_.staticFriction_;
    const double &b = params_.viscousFriction_;

    double sign_vel = tanh(50 * vel);  // tanh approximation to sign(vel)

    if (H_torque) *H_torque = gtsam::Vector1(1);
    if (H_tension) *H_tension = gtsam::Vector1(-r);
    if (H_vel)
      *H_vel = gtsam::Vector1(mu * 50 * (1 - sign_vel * sign_vel) + b / r);
    if (H_acc) *H_acc = gtsam::Vector1(I / r);

    return gtsam::Vector1(torque -            // tau =
                          tension * r +       //  F * r -
                          I * acc / r -       //  I * alpha +
                          (-mu * sign_vel) -  //  static friction +
                          (-b * vel / r));    //  viscous friction
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(params_);
  }
};

}  // namespace gtdynamics
