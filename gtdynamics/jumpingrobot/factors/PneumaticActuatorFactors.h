/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ClippingActuatorFactor.h
 * @brief Factors related to pneumatic actuator.
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

/** ForceBalanceFactor is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class ForceBalanceFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef ForceBalanceFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  double k_, r_, q_rest_;
  bool positive_;
  gtsam::Matrix H_delta_x_, H_q_, H_f_;

 public:
  /** Create pneumatic actuator factor
   *
   Keyword arguments:
     k      -- spring constant
     r      -- pulley radius
     delta_x_key -- key for actuator contraction in cm
     create factor corresponding to the equation:
     F/k - delta_x - r * theta = 0
   */
  ForceBalanceFactor(gtsam::Key delta_x_key, gtsam::Key q_key, gtsam::Key f_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const double k, const double r, const double q_rest,
                     const bool contract = false)
      : Base(cost_model, delta_x_key, q_key, f_key),
        k_(k),
        r_(r),
        q_rest_(q_rest),
        positive_(contract) {
    double sign = positive_ ? 1 : -1;
    H_delta_x_.setConstant(1, 1, k_ * 0.01);  // from cm to m
    H_q_.setConstant(1, 1, -sign * k_ * r_);
    H_f_.setConstant(1, 1, -1);
  }
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
    if (H_delta_x) *H_delta_x = H_delta_x_;
    if (H_q) *H_q = H_q_;
    if (H_f) *H_f = H_f_;
    return H_f_ * f + H_delta_x_ * delta_x + H_q_ * (q - q_rest_);
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
  gtsam::Matrix H_v_, H_f_, H_torque_;
  double sign_;

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
        positive_(positive) {
    sign_ = positive_ ? 1 : -1;
    H_v_.setConstant(1, 1, -b_);
    H_f_.setConstant(1, 1, sign_ * r_);
    H_torque_.setConstant(1, 1, -1);
  }
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
        H_q->setConstant(1, 1, -ka_);
      } else {
        H_q->setConstant(1, 1, 0);
      }
    }
    if (H_v) *H_v = H_v_;
    if (H_f) *H_f = H_f_;
    if (H_torque) *H_torque = H_torque_;

    return gtsam::Vector1(sign_ * r_ * f - ka_ * delta_q - b_ * v - torque);
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

/** SmoothActuatorFactor fits a smooth relationship between pressure,
 * contraction length and force */
class SmoothActuatorFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef SmoothActuatorFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  const gtsam::Vector5 x0_coeffs_ =
      (gtsam::Vector(5) << 3.05583930e+00, 7.58361626e-02, -4.91579771e-04,
       1.42792618e-06, -1.54817477e-09)
          .finished();  // TODO(yetong): using static
  const gtsam::Vector2 f0_coeffs_ = gtsam::Vector2(0, 1.966409);
  const gtsam::Vector2 k_coeffs_ = gtsam::Vector2(0, 0.35541599);

 public:
  /** Create pneumatic actuator factor
   *  delta_x_key -- key for actuator contraction in cm
   */
  SmoothActuatorFactor(gtsam::Key delta_x_key, gtsam::Key p_key,
                       gtsam::Key f_key,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, delta_x_key, p_key, f_key) {}
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
      H_f->setConstant(1, 1, -1);
    }

    gtsam::Vector5 gauge_p_powers5;
    gtsam::Vector2 gauge_p_powers2(1, gauge_p);
    // (1, gauge_p, gauge_p*gauge_p, pow(gauge_p,3), pow(gauge_p,4));

    gauge_p_powers5(0) = 1;
    for (size_t i = 1; i < 5; i++) {
      gauge_p_powers5(i) = gauge_p_powers5(i - 1) * gauge_p;
    }

    double x0 = x0_coeffs_.dot(gauge_p_powers5);

    // over contraction: should return 0
    if (gauge_p <= 0 || delta_x > x0) {
      if (H_delta_x) H_delta_x->setConstant(1, 1, 0);
      if (H_p) H_p->setConstant(1, 1, 0);
      return gtsam::Vector1(-f);
    }

    double k = k_coeffs_.dot(gauge_p_powers2);
    double f0 = f0_coeffs_.dot(gauge_p_powers2);
    double j_k_p, j_f0_p;
    if (H_p) {
      j_k_p = 0;
      for (size_t i = 1; i < 2; i++) {
        j_k_p += i * k_coeffs_(i) * gauge_p_powers2(i - 1);
      }
      j_f0_p = 0;
      for (size_t i = 1; i < 2; i++) {
        j_f0_p += i * f0_coeffs_(i) * gauge_p_powers2(i - 1);
      }
    }

    // over extension: should model as a spring
    if (delta_x < 0) {
      if (H_delta_x) H_delta_x->setConstant(1, 1, -k);
      if (H_p) H_p->setConstant(1, 1, j_f0_p - j_k_p * delta_x);
      return gtsam::Vector1(f0 - k * delta_x - f);
    }

    // normal condition
    double delta_x_2 = delta_x * delta_x;
    double delta_x_3 = delta_x_2 * delta_x;
    double c = (2 * k * x0 - 3 * f0) / (x0 * x0);
    double d = (-k * x0 + 2 * f0) / (x0 * x0 * x0);
    double x0_2 = x0 * x0;
    double x0_3 = x0_2 * x0;
    double x0_4 = x0_3 * x0;
    if (H_delta_x)
      H_delta_x->setConstant(1, 1, 3 * d * delta_x_2 + 2 * c * delta_x - k);
    if (H_p) {
      double j_x0_p = 0;
      for (size_t i = 1; i < 5; i++) {
        j_x0_p += i * x0_coeffs_(i) * gauge_p_powers5(i - 1);
      }
      double j_c_p = (-2 * k / x0_2 + 6 * f0 / x0_3) * j_x0_p +
                     (2 / x0) * j_k_p + (-3 / x0_2) * j_f0_p;
      double j_d_p = (2 * k / x0_3 - 6 * f0 / x0_4) * j_x0_p +
                     (-1 / x0_2) * j_k_p + (2 / x0_3) * j_f0_p;
      double j_p =
          delta_x_3 * j_d_p + delta_x_2 * j_c_p + delta_x * (-j_k_p) + j_f0_p;
      H_p->setConstant(1, 1, j_p);
    }
    double expected_f = d * delta_x_3 + c * delta_x_2 + (-k) * delta_x + f0;
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
        "SmoothActuatorFactor", boost::serialization::base_object<Base>(*this));
  }
};

/** ClippingActuatorFactor (deprecated) fits a non-smooth relationship between
 * pressure, contraction length and force, and use clipping for force < 0 */
class ClippingActuatorFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef ClippingActuatorFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  std::vector<double> coeffs_{-17.39,    1.11,       2.22,   -0.9486,
                              -0.4481,   -0.0003159, 0.1745, 0.01601,
                              0.0001081, -7.703e-07};
  const std::vector<double> powx_ =
      std::vector<double>{0, 1, 0, 2, 1, 0, 3, 2, 1, 0};
  const std::vector<double> powy_ =
      std::vector<double>{0, 0, 1, 0, 1, 2, 0, 1, 2, 3};
  const double extension_k_ = -200;

 public:
  /** Create factor corresponding to the equation:
     f = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 +
              p21*x^2*y + p12*x*y^2 + p03*y^3
      p00, p10, p01, p20, p11, p02, p30, p21, p12, p03
   Keyword arguments:
     delta_x_key -- key for actuator contraction in cm
   */
  ClippingActuatorFactor(gtsam::Key delta_x_key, gtsam::Key p_key,
                         gtsam::Key f_key,
                         const gtsam::noiseModel::Base::shared_ptr &cost_model)
      : Base(cost_model, delta_x_key, p_key, f_key) {}
  virtual ~ClippingActuatorFactor() {}

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
    std::vector<double> p_powers(4, 1);
    std::vector<double> delta_x_powers(4, 1);
    for (size_t i = 1; i < 4; i++) {
      p_powers[i] = p_powers[i - 1] * p;
      delta_x_powers[i] = delta_x_powers[i - 1] * delta_x;
    }

    double f_expected = 0;
    if (delta_x < 0) {
      // calculate F(p, 0)
      double f_x0 = 0;
      for (size_t i = 0; i < coeffs_.size(); i++) {
        if (powx_[i] == 0) f_x0 += coeffs_[i] * p_powers[powy_[i]];
      }
      if (f_x0 < 0) {
        f_x0 = 0;
        if (H_p) {
          H_p->setConstant(1, 1, 0);
        }
      } else {
        if (H_p) {
          double derivative_y = 0;
          for (size_t i = 0; i < coeffs_.size(); i++) {
            if (powy_[i] > 0 && powx_[i] == 0) {
              derivative_y += coeffs_[i] * powy_[i] * p_powers[powy_[i] - 1];
            }
          }
          H_p->setConstant(1, 1, derivative_y);
        }
      }

      if (H_delta_x) {
        H_delta_x->setConstant(1, 1, extension_k_);
      }
      if (H_f) {
        H_f->setConstant(1, 1, -1);
      }
      f_expected = f_x0 + extension_k_ * delta_x;
      return gtsam::Vector1(f_expected - f);
    }

    f_expected = 0;
    for (size_t i = 0; i < coeffs_.size(); i++) {
      f_expected += coeffs_[i] * delta_x_powers[powx_[i]] * p_powers[powy_[i]];
    }
    if (delta_x > 8 || (p < 100 && delta_x > 6.5) || (p < 0 && delta_x > 0) ||
        f_expected < 0) {
      if (H_delta_x) {
        H_delta_x->setConstant(1, 1, 0);
      }
      if (H_p) {
        H_p->setConstant(1, 1, 0);
      }
      if (H_f) {
        H_f->setConstant(1, 1, -1);
      }
      return gtsam::Vector1(-f);
    } else {
      if (H_delta_x) {
        double derivative_x = 0;
        for (size_t i = 0; i < coeffs_.size(); i++) {
          if (powx_[i] > 0) {
            derivative_x += coeffs_[i] * powx_[i] *
                            delta_x_powers[powx_[i] - 1] * p_powers[powy_[i]];
          }
        }
        H_delta_x->setConstant(1, 1, derivative_x);
      }
      if (H_p) {
        double derivative_y = 0;
        for (size_t i = 0; i < coeffs_.size(); i++) {
          if (powy_[i] > 0) {
            derivative_y += coeffs_[i] * powy_[i] * p_powers[powy_[i] - 1] *
                            delta_x_powers[powx_[i]];
          }
        }
        H_p->setConstant(1, 1, derivative_y);
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

/** ActuatorVolumeFactor characterize the relationship between actuator volume
 * and contraction length */
class ActuatorVolumeFactor : public gtsam::NoiseModelFactor2<double, double> {
 private:
  typedef ActuatorVolumeFactor This;
  typedef gtsam::NoiseModelFactor2<double, double> Base;
  gtsam::Vector4 c_ = gtsam::Vector4(4.243e-5, 3.141e-5, -3.251e-6, 1.28e-7);
  double D_, L_;

 public:
  ActuatorVolumeFactor(gtsam::Key v_key, gtsam::Key l_key,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model,
                       const double d_tube, const double l_tube)
      : Base(cost_model, v_key, l_key), D_(d_tube), L_(l_tube) {}
  virtual ~ActuatorVolumeFactor() {}

 public:
  double computeVolume(const double &l, boost::optional<gtsam::Matrix &> H_l =
                                            boost::none) const {
    gtsam::Vector4 l_powers(1, l, l*l, l*l*l);
    double expected_v = L_ * M_PI * pow(D_ / 2, 2);
    expected_v += c_.dot(l_powers);

    if (H_l) {
      double derivative = 0;
      for (size_t i = 1; i < 4; i++) {
        derivative += i * c_(i) * l_powers(i - 1);
      }
      H_l->setConstant(1, 1, derivative);
    }
    return expected_v;
  }

  gtsam::Vector evaluateError(
      const double &v, const double &l,
      boost::optional<gtsam::Matrix &> H_v = boost::none,
      boost::optional<gtsam::Matrix &> H_l = boost::none) const override {
    double expected_v = computeVolume(l, H_l);
    if (H_v) {
      *H_v = -gtsam::I_1x1;
    }
    return gtsam::Vector1(expected_v - v);
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
    std::cout << s << "actuator volume factor" << std::endl;
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

}  // namespace gtdynamics
