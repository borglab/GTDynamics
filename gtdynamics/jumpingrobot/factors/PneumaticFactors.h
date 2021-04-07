/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PneumaticActuatorFactor.h
 * @brief Pneumatic factors.
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

/** GassLawFactor: P*V=Rs*T */
class GassLawFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef GassLawFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  double c_;

 public:
  GassLawFactor(gtsam::Key p_key, gtsam::Key v_key, gtsam::Key m_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const double c)
      : Base(cost_model, p_key, v_key, m_key),
        c_(c) {}
  virtual ~GassLawFactor() {}

 private:
 public:
  gtsam::Vector evaluateError(
      const double &p, const double &v, const double &m,
      boost::optional<gtsam::Matrix &> H_p = boost::none,
      boost::optional<gtsam::Matrix &> H_v = boost::none,
      boost::optional<gtsam::Matrix &> H_m = boost::none) const override {
    if (H_p) {
      H_p->setConstant(1, 1, 1e3 * v);
    }
    if (H_v) {
      H_v->setConstant(1, 1, 1e3 * p);
    }
    if (H_m) {
      H_m->setConstant(1, 1, -c_);
    }

    return gtsam::Vector1(1e3 * p * v - c_ * m);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "gass law factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};



/** MassFlowRateFactor: compute mdot from pressures*/
class MassFlowRateFactor
    : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef MassFlowRateFactor This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  double D_, L_, mu_, epsilon_, k_;
  double term1_, term2_, c1_, coeff_;

 public:
  MassFlowRateFactor(gtsam::Key pm_key, gtsam::Key ps_key, gtsam::Key mdot_key,
                 const gtsam::noiseModel::Base::shared_ptr &cost_model,
                 const double D, const double L, const double mu, const double epsilon, const double k)
      : Base(cost_model, pm_key, ps_key, mdot_key),
        D_(D), L_(L), mu_(mu), epsilon_(epsilon), k_(k),
        term1_(6.9/4 * M_PI * D_ * mu_), term2_(pow(epsilon_ / (3.7 * D_), 1.11)),
        c1_(pow(1.8 / log(10), -2)), coeff_(1e3 * sqrt(pow(M_PI, 2) * pow(D_, 5) * k_ / (16.0 * L_))) {}
  virtual ~MassFlowRateFactor() {}

 public:
  gtsam::Vector evaluateError(
      const double &pm, const double &ps, const double &mdot,
      boost::optional<gtsam::Matrix &> H_pm = boost::none,
      boost::optional<gtsam::Matrix &> H_ps = boost::none,
      boost::optional<gtsam::Matrix &> H_mdot = boost::none) const override {

    double tmp = term1_ / abs(mdot) + term2_;
    double fD = c1_ * pow(log(tmp), -2);
    double p_square_diff = abs(ps * ps - pm * pm);
    int sign_p = abs(ps) > abs(pm) ? 1 : -1;
    int sign_mdot = mdot > 0 ? 1 : -1;
    double expected_mdot = sign_p * coeff_ * sqrt(p_square_diff/fD);

    if (H_pm) {
      H_pm->setConstant(1, 1, -coeff_ * pow(p_square_diff*fD, -0.5) * pm);
    }
    if (H_ps) {
      H_ps->setConstant(1, 1, coeff_ * pow(p_square_diff*fD, -0.5) * ps);
    }
    if (H_mdot) {
      double d_fD = -0.5 * sign_p * coeff_ * sqrt(p_square_diff) * pow(fD, -1.5);
      double d_fD_tmp = c1_ * (-2) * pow(log(tmp), -3) / tmp;
      double d_tmp_mdot = -term1_ / (mdot * mdot) * sign_mdot;
      H_mdot->setConstant(1, 1, -1 + d_fD * d_fD_tmp * d_tmp_mdot);
    }
    return gtsam::Vector1(expected_mdot - mdot);
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
    std::cout << s << "mass flow rate factor" << std::endl;
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


double sigmoid (double x,  boost::optional<gtsam::Matrix &> H_x = boost::none) {
    double neg_exp = exp(-x);
    if (H_x) {
        H_x->setConstant(1, 1, neg_exp / pow(1.0 + neg_exp, 2));
    }
    return 1.0/(1.0+neg_exp);
}


/** ValveControlFactor: compute true mdot based on valve open/close time */
class ValveControlFactor
    : public gtsam::NoiseModelFactor5<double, double, double, double, double> {
 private:
  typedef ValveControlFactor This;
  typedef gtsam::NoiseModelFactor5<double, double, double, double, double>  Base;
  double ct_ = 0.1;

 public:
  ValveControlFactor(gtsam::Key t_key, gtsam::Key to_key, gtsam::Key tc_key, 
                     gtsam::Key mdot_key, gtsam::Key true_mdot_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    const double ct)
      : Base(cost_model, t_key, to_key, tc_key, mdot_key, true_mdot_key),
        ct_(ct) {}
  virtual ~ValveControlFactor() {}

 public:
  gtsam::Vector evaluateError(
      const double &t, const double &to, const double &tc, const double& mdot, const double& true_mdot,
      boost::optional<gtsam::Matrix &> H_t = boost::none,
      boost::optional<gtsam::Matrix &> H_to = boost::none,
      boost::optional<gtsam::Matrix &> H_tc = boost::none,
      boost::optional<gtsam::Matrix &> H_mdot = boost::none,
      boost::optional<gtsam::Matrix &> H_true_mdot = boost::none) const override {

    double dto = (t - to) / ct_;
    double dtc = (t - tc) / ct_;
    double coeff = sigmoid(dto, H_to) - sigmoid(dtc, H_tc);

    if (H_t) {
      *H_t = (*H_to - *H_tc) * (mdot / ct_);
    }
    if (H_to) {
      *H_to = *H_to * (-mdot / ct_);
    }
    if (H_tc) {
      *H_tc = *H_tc * (mdot / ct_);
    }
    if (H_mdot) {
      H_mdot->setConstant(1, 1, coeff);
    }
    if (H_true_mdot) {
      H_true_mdot->setConstant(1, 1, -1);
    }
    return gtsam::Vector1(coeff * mdot - true_mdot);
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
    std::cout << s << "valve control factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};



}  // namespace gtdynamics