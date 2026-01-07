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

// TODO(yetong): split into different files

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <iostream>
#include <string>

namespace gtdynamics {

/** GasLawFactor: P*V=Rs*T */
class GasLawFactor : public gtsam::NoiseModelFactorN<double, double, double> {
 private:
  typedef GasLawFactor This;
  typedef gtsam::NoiseModelFactorN<double, double, double> Base;
  double c_;

 public:
  GasLawFactor(gtsam::Key p_key, gtsam::Key v_key, gtsam::Key m_key,
               const gtsam::noiseModel::Base::shared_ptr &cost_model,
               const double c)
      : Base(cost_model, p_key, v_key, m_key), c_(c) {}
  virtual ~GasLawFactor() {}

 private:
 public:
  gtsam::Vector evaluateError(
      const double &p, const double &v, const double &m,
      gtsam::OptionalMatrixType H_p = nullptr,
      gtsam::OptionalMatrixType H_v = nullptr,
      gtsam::OptionalMatrixType H_m = nullptr) const override {
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
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "gass law factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

/** MassFlowRateFactor: compute mdot from pressures*/
class MassFlowRateFactor
    : public gtsam::NoiseModelFactorN<double, double, double> {
 private:
  typedef MassFlowRateFactor This;
  typedef gtsam::NoiseModelFactorN<double, double, double> Base;
  double D_, L_, mu_, epsilon_, k_;
  double term1_, term2_, c1_, coeff_;

 public:
  MassFlowRateFactor(gtsam::Key pm_key, gtsam::Key ps_key, gtsam::Key mdot_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const double D, const double L, const double mu,
                     const double epsilon, const double k)
      : Base(cost_model, pm_key, ps_key, mdot_key),
        D_(D),
        L_(L),
        mu_(mu),
        epsilon_(epsilon),
        k_(k),
        term1_(6.9 / 4 * M_PI * D_ * mu_),
        term2_(pow(epsilon_ / (3.7 * D_), 1.11)),
        c1_(pow(1.8 / log(10), -2)),
        coeff_(1e3 * sqrt(pow(M_PI, 2) * pow(D_, 5) * k_ / (16.0 * L_))) {}
  virtual ~MassFlowRateFactor() {}

 public:
  /** compute the mass flow assuming valve is always open
   * fD = [-1.8 log(6.9/Re+(epsilon/3.7D)^1.11)]^-2
   * mdot^2 = (pi^2 D^5)/(16fD L Rs T)/(ps^2-pm^2)
   */
  double computeExpectedMassFlow(
      const double &pm, const double &ps, const double &mdot,
      gtsam::OptionalMatrixType H_pm = nullptr,
      gtsam::OptionalMatrixType H_ps = nullptr,
      gtsam::OptionalMatrixType H_mdot = nullptr) const {
    double tmp = term1_ / abs(mdot) + term2_;
    double fD = c1_ * pow(log(tmp), -2);
    double p_square_diff = abs(ps * ps - pm * pm);
    int sign_p = abs(ps) > abs(pm) ? 1 : -1;
    int sign_mdot = mdot > 0 ? 1 : -1;
    double expected_mdot = sign_p * coeff_ * sqrt(p_square_diff / fD);

    if (H_pm) {
      H_pm->setConstant(1, 1, -coeff_ * pow(p_square_diff * fD, -0.5) * pm);
    }
    if (H_ps) {
      H_ps->setConstant(1, 1, coeff_ * pow(p_square_diff * fD, -0.5) * ps);
    }
    if (H_mdot) {
      double d_fD =
          -0.5 * sign_p * coeff_ * sqrt(p_square_diff) * pow(fD, -1.5);
      double d_fD_tmp = c1_ * (-2) * pow(log(tmp), -3) / tmp;
      double d_tmp_mdot = -term1_ / (mdot * mdot) * sign_mdot;
      H_mdot->setConstant(1, 1, d_fD * d_fD_tmp * d_tmp_mdot);
    }
    return expected_mdot;
  }

  gtsam::Vector evaluateError(
      const double &pm, const double &ps, const double &mdot,
      gtsam::OptionalMatrixType H_pm = nullptr,
      gtsam::OptionalMatrixType H_ps = nullptr,
      gtsam::OptionalMatrixType H_mdot = nullptr) const override {
    double expected_mdot =
        computeExpectedMassFlow(pm, ps, mdot, H_pm, H_ps, H_mdot);
    if (H_mdot) {
      *H_mdot = *H_mdot - gtsam::I_1x1;
    }
    return gtsam::Vector1(expected_mdot - mdot);
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
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
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

/** Sigmoid function, 1/(1+e^-x), used to model the change of mass flow
 * rate when valve is open/closed. */
double sigmoid(double x, gtsam::OptionalMatrixType H_x = nullptr) {
  double neg_exp = exp(-x);
  if (H_x) {
    H_x->setConstant(1, 1, neg_exp / pow(1.0 + neg_exp, 2));
  }
  return 1.0 / (1.0 + neg_exp);
}

/** ValveControlFactor: compute true mdot based on valve open/close time. */
class ValveControlFactor
    : public gtsam::NoiseModelFactorN<double, double, double, double, double> {
 private:
  typedef ValveControlFactor This;
  typedef gtsam::NoiseModelFactorN<double, double, double, double, double> Base;
  double ct_inv_;

 public:
  ValveControlFactor(gtsam::Key t_key, gtsam::Key to_key, gtsam::Key tc_key,
                     gtsam::Key mdot_key, gtsam::Key true_mdot_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const double ct)
      : Base(cost_model, t_key, to_key, tc_key, mdot_key, true_mdot_key),
        ct_inv_(1 / ct) {}
  virtual ~ValveControlFactor() {}

 public:
  /* Compute the expected mass flow given current time, valve open/close times,
   * and the nominal mass flow rate assuming valve is always open.
   * mdot_sigma = (sigmoid((t-to)/ct) - sigmoid((t-tc)/ct)) * mdot
   */
  double computeExpectedTrueMassFlow(
      const double &t, const double &to, const double &tc, const double &mdot,
      gtsam::OptionalMatrixType H_t = nullptr,
      gtsam::OptionalMatrixType H_to = nullptr,
      gtsam::OptionalMatrixType H_tc = nullptr,
      gtsam::OptionalMatrixType H_mdot = nullptr) const {
    double dto = (t - to) * ct_inv_;
    double dtc = (t - tc) * ct_inv_;
    double coeff = sigmoid(dto, H_to) - sigmoid(dtc, H_tc);

    if (H_t) {
      *H_t = (*H_to - *H_tc) * (mdot * ct_inv_);
    }
    if (H_to) {
      *H_to = *H_to * (-mdot * ct_inv_);
    }
    if (H_tc) {
      *H_tc = *H_tc * (mdot * ct_inv_);
    }
    if (H_mdot) {
      H_mdot->setConstant(1, 1, coeff);
    }
    return coeff * mdot;
  }

  gtsam::Vector evaluateError(
      const double &t, const double &to, const double &tc, const double &mdot,
      const double &true_mdot, gtsam::OptionalMatrixType H_t = nullptr,
      gtsam::OptionalMatrixType H_to = nullptr,
      gtsam::OptionalMatrixType H_tc = nullptr,
      gtsam::OptionalMatrixType H_mdot = nullptr,
      gtsam::OptionalMatrixType H_true_mdot = nullptr) const override {
    double expected_true_mdot =
        computeExpectedTrueMassFlow(t, to, tc, mdot, H_t, H_to, H_tc, H_mdot);
    if (H_true_mdot) {
      H_true_mdot->setConstant(1, 1, -1);
    }
    return gtsam::Vector1(expected_true_mdot - true_mdot);
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
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
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtdynamics