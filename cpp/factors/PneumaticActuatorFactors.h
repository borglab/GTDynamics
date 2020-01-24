/**
 * @file  PneumaticActuatorFactor.h
 * @brief pneumatic actuator factor.
 * @Author: Yetong Zhang
 */

#pragma once

#include <utils.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

namespace robot {


/** PressureFactor is a three-way nonlinear factor which characterize the how
 * pressure in pneumatic actuator evolve with time */
class PressureFactor : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  typedef PressureFactor This;
  typedef gtsam::NoiseModelFactor4<double, double, double, double> Base;
  std::vector<double> coeffs_; //t0_, c1_, c2_, c3_

 public:
  /** Create single factor relating the pressure to time
   *
   Keyword arguments:
     create factor corresponding to the equation:
     p = init_p/(1+exp(c3*x^3 + c2*x^2 + c1*x)), 
      where x = current_t - start_t - t0
   */
  PressureFactor(gtsam::Key start_t_key, gtsam::Key current_t_key, 
                gtsam::Key init_p_key, gtsam::Key p_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const std::vector<double> &coeffs)
      : Base(cost_model, start_t_key, current_t_key, init_p_key, p_key),
        coeffs_(coeffs){}
  virtual ~PressureFactor() {}

 private:
 public:
  /** evaluate pressure vs. time error
      Keyword argument:
          start_t       -- time to open the valve
          current_t     -- current time
          init_p        -- preset pressure
          current_p     -- current pressure
  */
  gtsam::Vector evaluateError(
      const double &start_t, const double &current_t, const double &init_p, const double &current_p,
      boost::optional<gtsam::Matrix &> H_start_t = boost::none,
      boost::optional<gtsam::Matrix &> H_current_t = boost::none,
      boost::optional<gtsam::Matrix &> H_init_p = boost::none,
      boost::optional<gtsam::Matrix &> H_p = boost::none) const override {
    const double x = current_t - start_t - coeffs_[0];
    const double exponent = coeffs_[3] * pow(x, 3.0) + coeffs_[2] * pow(x, 2.0) + coeffs_[1] * x;
    const double exponential = exp(exponent);
    const double result = init_p / (1.0 + exponential);
    double derivative_t = result * exponential / (1 + exponential) *
                      (3 * coeffs_[3] * pow(x, 2.0) + 2 * coeffs_[2] * x + coeffs_[1]);
    if (H_start_t) {
      *H_start_t = gtsam::I_1x1 * derivative_t;
    }
    if (H_current_t) {
      *H_current_t = -gtsam::I_1x1 * derivative_t;
    }
    if (H_init_p) {
      *H_init_p = gtsam::I_1x1 * (1.0 / (1.0 + exponential));
    }
    if (H_p) {
      *H_p = -gtsam::I_1x1;
    }

    return gtsam::Vector1(result - current_p);
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
    std::cout << s << "pressure factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};







/** PneumaticActuatorFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class PneumaticActuatorFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef PneumaticActuatorFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  std::vector<double> coeffs_;
  const std::vector<double> powx_ =
      std::vector<double>{0, 1, 0, 2, 1, 0, 3, 2, 1, 0};
  const std::vector<double> powy_ =
      std::vector<double>{0, 0, 1, 0, 1, 2, 0, 1, 2, 3};

 public:
  /** Create pneumatic actuator factor
   *
   Keyword arguments:
     create factor corresponding to the equation:
     f = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 +
              p21*x^2*y + p12*x*y^2 + p03*y^3
      p00, p10, p01, p20, p11, p02, p30, p21, p12, p03
   */
  PneumaticActuatorFactor(gtsam::Key delta_x_key, gtsam::Key p_key,
                          gtsam::Key f_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const std::vector<double> &coeffs)
      : Base(cost_model, delta_x_key, p_key, f_key),
        coeffs_(coeffs) {}
  virtual ~PneumaticActuatorFactor() {}

 private:
 public:
  /** evaluate errors
      Keyword argument:
          delta_x     -- contraction length
          p           --  pressure
          f           -- force
  */
  gtsam::Vector evaluateError(
      const double &delta_x, const double &p, const double &f,
      boost::optional<gtsam::Matrix &> H_delta_x = boost::none,
      boost::optional<gtsam::Matrix &> H_p = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none) const override {
    double f_expected = 0;
    for (size_t i = 0; i < coeffs_.size(); i++) {
      f_expected += coeffs_[i] * pow(delta_x, powx_[i]) * pow(p, powy_[i]);
    }
    if (H_delta_x) {
      double derivative_x = 0;
      for (size_t i = 0; i < coeffs_.size(); i++) {
        if (powx_[i] > 0) {
          derivative_x += coeffs_[i] * powx_[i] * pow(delta_x, powx_[i] - 1) *
                          pow(p, powy_[i]);
        }
      }
      *H_delta_x = gtsam::I_1x1 * derivative_x;
    }
    if (H_p) {
      double derivative_y = 0;
      for (size_t i = 0; i < coeffs_.size(); i++) {
        if (powy_[i] > 0) {
          derivative_y += coeffs_[i] * powy_[i] * pow(p, powy_[i] - 1) *
                          pow(delta_x, powx_[i]);
        }
      }
      *H_p = gtsam::I_1x1 * derivative_y;
    }
    if (H_f) {
      *H_f = -gtsam::I_1x1;
    }
    return gtsam::Vector1(f_expected - f);
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
    std::cout << s << "pneumatic actuator factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};




/** JointBalanceFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class JointBalanceFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef JointBalanceFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  double k_, r_, qRest_;
  bool flipped_;

 public:
  /** Create pneumatic actuator factor
   *
   Keyword arguments:
     k      -- spring constant
     r      -- pulley radius
     create factor corresponding to the equation:
     F/k - delta_x - r * theta = 0
   */
  JointBalanceFactor(gtsam::Key delta_x_key, gtsam::Key q_key,
                          gtsam::Key f_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const double k, const double r, const double qRest, const bool flipped=false)
      : Base(cost_model, delta_x_key, q_key, f_key),
        k_(k), r_(r), qRest_(qRest), flipped_(flipped) {}
  virtual ~JointBalanceFactor() {}

 private:
 public:
  /** evaluate errors
      Keyword argument:
          delta_x     -- contraction length
          q           --  joint angle
          f           -- force
  */
  gtsam::Vector evaluateError(
      const double &delta_x, const double &q, const double &f,
      boost::optional<gtsam::Matrix &> H_delta_x = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none) const override {
    if (H_delta_x) {
      *H_delta_x = gtsam::I_1x1/100.0; // from cm to m
    }
    if (H_q) {
      if (flipped_)
      {
        *H_q = -gtsam::I_1x1 * r_;
      }
      else {
        *H_q = gtsam::I_1x1 * r_;
      }
      
    }
    if (H_f) {
      *H_f = -gtsam::I_1x1 / k_;
    }
    if (flipped_)
    {
      return gtsam::Vector1(-f/k_ + delta_x/100.0 - r_*(q-qRest_));
    }
    else {
      return gtsam::Vector1(-f/k_ + delta_x/100.0 + r_*(q-qRest_));
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
    std::cout << s << "actuator joint factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace robot
