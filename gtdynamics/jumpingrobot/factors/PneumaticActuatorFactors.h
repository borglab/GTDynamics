/**
 * @file  PneumaticActuatorFactor.h
 * @brief pneumatic actuator factor.
 * @Author: Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/optional.hpp>
#include <iostream>
#include <string>

namespace gtdynamics {

/** PressureFactor is a three-way nonlinear factor which characterize the how
 * pressure in pneumatic actuator evolve with time */
class PressureFactor
    : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  typedef PressureFactor This;
  typedef gtsam::NoiseModelFactor4<double, double, double, double> Base;
  std::vector<double> coeffs_;  // t0_, c1_, c2_, c3_

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
        coeffs_(coeffs) {}
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
      const double &start_t, const double &current_t, const double &init_p,
      const double &current_p,
      boost::optional<gtsam::Matrix &> H_start_t = boost::none,
      boost::optional<gtsam::Matrix &> H_current_t = boost::none,
      boost::optional<gtsam::Matrix &> H_init_p = boost::none,
      boost::optional<gtsam::Matrix &> H_p = boost::none) const override {
    const double x = current_t - start_t - coeffs_[0];
    const double exponent =
        coeffs_[3] * pow(x, 3.0) + coeffs_[2] * pow(x, 2.0) + coeffs_[1] * x;
    const double exponential = exp(exponent);
    const double result = init_p / (1.0 + exponential);
    double derivative_t =
        result * exponential / (1 + exponential) *
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
  double extension_k_;

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
        coeffs_(coeffs),
        extension_k_(-200) {}
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
    if (delta_x < 0) {
      // calculate F(p, 0)
      double f_x0 = 0;
      for (size_t i = 0; i < coeffs_.size(); i++) {
        f_x0 += coeffs_[i] * pow(0, powx_[i]) * pow(p, powy_[i]);
      }
      if (f_x0 < 0) {
        f_x0 = 0;
        if (H_p) {
          *H_p = gtsam::I_1x1 * 0;
        }
      } else {
        if (H_p) {
          double derivative_y = 0;
          for (size_t i = 0; i < coeffs_.size(); i++) {
            if (powy_[i] > 0) {
              derivative_y += coeffs_[i] * powy_[i] * pow(p, powy_[i] - 1) *
                              pow(0, powx_[i]);
            }
          }
          *H_p = gtsam::I_1x1 * derivative_y;
        }
      }

      if (H_delta_x) {
        *H_delta_x = gtsam::I_1x1 * extension_k_;
      }
      if (H_f) {
        *H_f = -gtsam::I_1x1;
      }
      f_expected = f_x0 + extension_k_ * delta_x;
      // std::cout << "f_x0: " << f_x0 << "\n";
      // std::cout << "f_expected: " << f_expected << "\n";
      return gtsam::Vector1(f_expected - f);
    }

    f_expected = 0;
    for (size_t i = 0; i < coeffs_.size(); i++) {
      f_expected += coeffs_[i] * pow(delta_x, powx_[i]) * pow(p, powy_[i]);
    }
    if (delta_x > 8 || (p < 100 && delta_x > 6.5) || (p < 0 && delta_x > 0) ||
        f_expected < 0) {
      if (H_delta_x) {
        *H_delta_x = gtsam::I_1x1 * 0;
      }
      if (H_p) {
        *H_p = gtsam::I_1x1 * 0;
      }
      if (H_f) {
        *H_f = -gtsam::I_1x1;
      }
      return gtsam::Vector1(-f);
    } else {
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

/** SimpleActuatorFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class SimpleActuatorFactor : public gtsam::NoiseModelFactor2<double, double> {
 private:
  typedef SimpleActuatorFactor This;
  typedef gtsam::NoiseModelFactor2<double, double> Base;
  std::vector<double> coeffs_;
  const std::vector<double> powx_ =
      std::vector<double>{0, 1, 0, 2, 1, 0, 3, 2, 1, 0};
  const std::vector<double> powy_ =
      std::vector<double>{0, 0, 1, 0, 1, 2, 0, 1, 2, 3};
  double extension_k_;
  double p_;

 public:
  /** Create pneumatic actuator factor
   *
   Keyword arguments:
     create factor corresponding to the equation:
     f = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 +
              p21*x^2*y + p12*x*y^2 + p03*y^3
      p00, p10, p01, p20, p11, p02, p30, p21, p12, p03
   */
  SimpleActuatorFactor(gtsam::Key delta_x_key, gtsam::Key f_key,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model,
                       const double p, const std::vector<double> &coeffs)
      : Base(cost_model, delta_x_key, f_key),
        coeffs_(coeffs),
        extension_k_(-200),
        p_(p) {}
  virtual ~SimpleActuatorFactor() {}

 private:
 public:
  /** evaluate errors
      Keyword argument:
          delta_x     -- contraction length
          p           --  pressure
          f           -- force
  */
  gtsam::Vector evaluateError(
      const double &delta_x, const double &f,
      boost::optional<gtsam::Matrix &> H_delta_x = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none) const override {
    double f_expected = 0;
    if (delta_x < 0) {
      // calculate F(p, 0)
      double f_x0 = 0;
      for (size_t i = 0; i < coeffs_.size(); i++) {
        f_x0 += coeffs_[i] * pow(0, powx_[i]) * pow(p_, powy_[i]);
      }
      if (f_x0 < 0) {
        f_x0 = 0;
      }

      if (H_delta_x) {
        *H_delta_x = gtsam::I_1x1 * extension_k_;
      }
      if (H_f) {
        *H_f = -gtsam::I_1x1;
      }
      f_expected = f_x0 + extension_k_ * delta_x;
      // std::cout << "f_x0: " << f_x0 << "\n";
      // std::cout << "f_expected: " << f_expected << "\n";
      return gtsam::Vector1(f_expected - f);
    }

    f_expected = 0;
    for (size_t i = 0; i < coeffs_.size(); i++) {
      f_expected += coeffs_[i] * pow(delta_x, powx_[i]) * pow(p_, powy_[i]);
    }
    if (delta_x > 8 || (p_ < 100 && delta_x > 6.5) || (p_ < 0 && delta_x > 0) ||
        f_expected < 0) {
      if (H_delta_x) {
        *H_delta_x = gtsam::I_1x1 * 0;
      }
      if (H_f) {
        *H_f = -gtsam::I_1x1;
      }
      return gtsam::Vector1(-f);
    } else {
      if (H_delta_x) {
        double derivative_x = 0;
        for (size_t i = 0; i < coeffs_.size(); i++) {
          if (powx_[i] > 0) {
            derivative_x += coeffs_[i] * powx_[i] * pow(delta_x, powx_[i] - 1) *
                            pow(p_, powy_[i]);
          }
        }
        *H_delta_x = gtsam::I_1x1 * derivative_x;
      }
      if (H_f) {
        *H_f = -gtsam::I_1x1;
      }
      return gtsam::Vector1(f_expected - f);
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
    std::cout << s << "pneumatic actuator factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

/** ForceBalanceFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class ForceBalanceFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef ForceBalanceFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  double k_, r_, qRest_;
  bool positive_;

 public:
  /** Create pneumatic actuator factor
   *
   Keyword arguments:
     k      -- spring constant
     r      -- pulley radius
     create factor corresponding to the equation:
     F/k - delta_x - r * theta = 0
   */
  ForceBalanceFactor(gtsam::Key delta_x_key, gtsam::Key q_key, gtsam::Key f_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const double k, const double r, const double qRest,
                     const bool contract = false)
      : Base(cost_model, delta_x_key, q_key, f_key),
        k_(k),
        r_(r),
        qRest_(qRest),
        positive_(contract) {}
  virtual ~ForceBalanceFactor() {}

 private:
 public:
  /** evaluate errors
      Keyword argument:
          delta_x     -- contraction length, in cm
          q           -- joint angle, in rad
          f           -- force, in N
  */
  gtsam::Vector evaluateError(
      const double &delta_x, const double &q, const double &f,
      boost::optional<gtsam::Matrix &> H_delta_x = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none) const override {
    if (H_delta_x) {
      *H_delta_x = gtsam::I_1x1 / 100.0;  // from cm to m
    }
    if (H_q) {
      if (positive_) {
        *H_q = -gtsam::I_1x1 * r_;
      } else {
        *H_q = gtsam::I_1x1 * r_;
      }
    }
    if (H_f) {
      *H_f = -gtsam::I_1x1 / k_;
    }
    if (positive_) {
      return gtsam::Vector1(-f / k_ + delta_x / 100.0 - r_ * (q - qRest_));
    } else {
      return gtsam::Vector1(-f / k_ + delta_x / 100.0 + r_ * (q - qRest_));
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

/** JointTorqueFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class JointTorqueFactor
    : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  typedef JointTorqueFactor This;
  typedef gtsam::NoiseModelFactor4<double, double, double, double> Base;
  double q_limit_, ka_, r_, b_;
  bool positive_;

 public:
  /** Create pneumatic actuator factor
   *
   Keyword arguments:
     q_limit  -- spring engagement starting angle
     ka       -- stiffness of antagonistic spring
     r        -- pulley radius
     b        -- damping coefficient
     create factor corresponding to the equation:
     F/k - delta_x - r * theta = 0
   */
  JointTorqueFactor(gtsam::Key q_key, gtsam::Key v_key, gtsam::Key f_key,
                    gtsam::Key torque_key,
                    const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    const double q_limit, const double ka, const double r,
                    const double b, const bool positive = false)
      : Base(cost_model, q_key, v_key, f_key, torque_key),
        q_limit_(q_limit),
        ka_(ka),
        r_(r),
        b_(b),
        positive_(positive) {}
  virtual ~JointTorqueFactor() {}

 private:
 public:
  /** evaluate errors
      Keyword argument:
          delta_x     -- contraction length, in cm
          q           -- joint angle, in rad
          f           -- force, in N
  */
  gtsam::Vector evaluateError(
      const double &q, const double &v, const double &f, const double &torque,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_v = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none) const override {
    bool antagonistic_active =
        (positive_ && q > q_limit_) || (!positive_ && q < q_limit_);
    double delta_q = antagonistic_active ? q - q_limit_ : 0;

    if (H_q) {
      if (antagonistic_active) {
        *H_q = -gtsam::I_1x1 * ka_;
      } else {
        *H_q = gtsam::I_1x1 * 0.0;
      }
    }
    if (H_v) {
      *H_v = -gtsam::I_1x1 * b_;
    }
    if (H_f) {
      if (positive_) {
        *H_f = gtsam::I_1x1 * r_;
      } else {
        *H_f = -gtsam::I_1x1 * r_;
      }
    }
    if (H_torque) {
      *H_torque = -gtsam::I_1x1;
    }

    if (positive_) {
      return gtsam::Vector1(r_ * f - ka_ * delta_q - b_ * v - torque);
    } else {
      return gtsam::Vector1(-r_ * f - ka_ * delta_q - b_ * v - torque);
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
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};






/** SmoothActuatorFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class SmoothActuatorFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef SmoothActuatorFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  std::vector<double> x0_coeffs_;
  std::vector<double> k_coeffs_;
  std::vector<double> f0_coeffs_;

 public:
  /** Create pneumatic actuator factor
   *
   */
  SmoothActuatorFactor(gtsam::Key delta_x_key, gtsam::Key p_key,
                          gtsam::Key f_key,
                          const gtsam::noiseModel::Base::shared_ptr &cost_model,
                          const std::vector<double> &x0_coeffs,
                          const std::vector<double> &k_coeffs,
                          const std::vector<double> &f0_coeffs)
      : Base(cost_model, delta_x_key, p_key, f_key),
        x0_coeffs_(x0_coeffs), k_coeffs_(k_coeffs), f0_coeffs_(f0_coeffs) {}
  virtual ~SmoothActuatorFactor() {}

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

      double gauge_p = p - 101.325;

      if (H_f) {
        *H_f = -gtsam::I_1x1;
      }

      double x0 = 0;
      for (size_t i = 0; i < x0_coeffs_.size(); i++) {
        x0 += x0_coeffs_[i] * pow(gauge_p, i);
      }

      // over contraction: should return 0
      if (gauge_p<=0 || delta_x > x0) {
        if (H_delta_x) *H_delta_x = gtsam::I_1x1 * 0;
        if (H_p) * H_p = gtsam::I_1x1 * 0;
        return gtsam::Vector1(-f);
      }
      
      double k = 0;
      for (size_t i = 0; i < k_coeffs_.size(); i++) {
        k += k_coeffs_[i] * pow(gauge_p, i);
      }
      double f0 = 0;
      for (size_t i = 0; i < f0_coeffs_.size(); i++) {
        f0 += f0_coeffs_[i] * pow(gauge_p, i);
      }
      double j_k_p, j_f0_p;
      if (H_p) {
        j_k_p = 0;
        for (size_t i = 1; i < k_coeffs_.size(); i++) {
          j_k_p += i * k_coeffs_[i] * pow(gauge_p, i-1);
        }
        j_f0_p = 0;
        for (size_t i = 1; i < f0_coeffs_.size(); i++) {
          j_f0_p += i * f0_coeffs_[i] * pow(gauge_p, i-1);
        }
      }

      // over extension: should model as a spring
      if (delta_x < 0) {
        if (H_delta_x) *H_delta_x = gtsam::I_1x1 * (-k);
        if (H_p) {
          *H_p = gtsam::I_1x1 * (j_f0_p - j_k_p * delta_x);
        }
        return gtsam::Vector1(f0 - k * delta_x - f);
      }

      // std::cout << x0 << ", " << f0 << ", " << k << "\n";

      // normal condition
      double c = (2*k*x0 - 3*f0) / (x0 * x0);
      double d = (-k*x0 + 2*f0) / (x0 * x0 * x0);
      double x0_2 = x0 * x0;
      double x0_3 = x0_2 * x0;
      double x0_4 = x0_3 * x0;
      if (H_delta_x) *H_delta_x = gtsam::I_1x1 * (3*d*delta_x*delta_x + 2*c*delta_x - k);
      if (H_p) {
        double j_x0_p = 0;
        for (size_t i = 1; i < x0_coeffs_.size(); i++) {
          j_x0_p += i * x0_coeffs_[i] * pow(gauge_p, i-1);
        }
        double j_c_p = (-2*k/x0_2+6*f0/x0_3) * j_x0_p + (2/x0) * j_k_p + (-3/x0_2) * j_f0_p;
        double j_d_p = (2*k/x0_3 - 6*f0/x0_4) * j_x0_p + (-1/x0_2) * j_k_p + (2/x0_3) * j_f0_p;
        double j_p = pow(delta_x, 3) * j_d_p + pow(delta_x, 2) * j_c_p + delta_x * (-j_k_p) + j_f0_p;
        // std::cout << j_k_p << ", " << j_f0_p << ", " << j_x0_p << ", " << j_c_p << ", " << j_d_p << ", " << j_p << "\n";
        *H_p = gtsam::I_1x1 * j_p;
      }
      double expected_f = d*pow(delta_x, 3) + c*pow(delta_x, 2) + (-k)*delta_x + f0;
      return gtsam::Vector1(expected_f - f);
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



}  // namespace gtdynamics
