/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SystemIdentificationFactors.h
 * @brief System identification factors for jumping robot.
 * @Author: Yetong Zhang
 */

namespace gtdynamics {


/** ForceBalanceFactorId is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class ForceBalanceFactorId
    : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  typedef ForceBalanceFactorId This;
  typedef gtsam::NoiseModelFactor4<double, double, double, double> Base;
  double r_, q_rest_;
  bool positive_;
  gtsam::Matrix H_f_;

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
  ForceBalanceFactorId(gtsam::Key delta_x_key, gtsam::Key q_key, gtsam::Key f_key,
                       gtsam::Key k_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const double r, const double q_rest,
                     const bool contract = false)
      : Base(cost_model, delta_x_key, q_key, f_key, k_key),
        r_(r),
        q_rest_(q_rest),
        positive_(contract) {
    H_f_.setConstant(1, 1, -1);
  }
  virtual ~ForceBalanceFactorId() {}

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
      const double &k,
      boost::optional<gtsam::Matrix &> H_delta_x = boost::none,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none,
      boost::optional<gtsam::Matrix &> H_k = boost::none) const override {
    double sign = positive_ ? 1 : -1;
    gtsam::Matrix H_delta_x_, H_q_;
    H_delta_x_.setConstant(1, 1, k * 0.01);  // from cm to m
    H_q_.setConstant(1, 1, -sign * k * r_);
    if (H_delta_x) *H_delta_x = H_delta_x_;
    if (H_q) *H_q = H_q_;
    if (H_f) *H_f = H_f_;
    if (H_k) H_k -> setConstant(1, 1, 0.01*delta_x - sign * r_ * (q-q_rest_));
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
    std::cout << s << "actuator joint factor (sysid)" << std::endl;
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





/** JointTorqueFactorId is a three-way nonlinear factor which characterize
 * the relationship between pressure, contraction length and force */
class JointTorqueFactorId
    : public gtsam::NoiseModelFactor5<double, double, double, double, double> {
 private:
  typedef JointTorqueFactorId This;
  typedef gtsam::NoiseModelFactor5<double, double, double, double, double> Base;
  double q_limit_, ka_, r_;
  bool positive_;
  gtsam::Matrix H_f_, H_torque_;
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
  JointTorqueFactorId(gtsam::Key q_key, gtsam::Key v_key, gtsam::Key f_key,
                    gtsam::Key torque_key, gtsam::Key b_key,
                    const gtsam::noiseModel::Base::shared_ptr &cost_model,
                    const double q_limit, const double ka, const double r,
                const bool positive = false)
      : Base(cost_model, q_key, v_key, f_key, torque_key, b_key),
        q_limit_(q_limit),
        ka_(ka),
        r_(r),
        positive_(positive) {
    sign_ = positive_ ? 1 : -1;
    // H_v_.setConstant(1, 1, -b_);
    H_f_.setConstant(1, 1, sign_ * r_);
    H_torque_.setConstant(1, 1, -1);
  }
  virtual ~JointTorqueFactorId() {}

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
      const double &b,
      boost::optional<gtsam::Matrix &> H_q = boost::none,
      boost::optional<gtsam::Matrix &> H_v = boost::none,
      boost::optional<gtsam::Matrix &> H_f = boost::none,
      boost::optional<gtsam::Matrix &> H_torque = boost::none,
      boost::optional<gtsam::Matrix &> H_b = boost::none) const override {
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
    if (H_v) H_v->setConstant(1, 1, -b);
    if (H_f) *H_f = H_f_;
    if (H_torque) *H_torque = H_torque_;
    if (H_b) H_b->setConstant(1, 1, -v);

    return gtsam::Vector1(sign_ * r_ * f - ka_ * delta_q - b * v - torque);
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
    std::cout << s << "actuator joint factor (sysid)" << std::endl;
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



/** ActuatorVolumeFactorId characterize the relationship between actuator volume
 * and contraction length */
class ActuatorVolumeFactorId : public gtsam::NoiseModelFactor3<double, double, double> {
 private:
  typedef ActuatorVolumeFactorId This;
  typedef gtsam::NoiseModelFactor3<double, double, double> Base;
  gtsam::Vector4 c_ = gtsam::Vector4(4.243e-5, 3.141e-5, -3.251e-6, 1.28e-7);
  double L_;

 public:
  ActuatorVolumeFactorId(gtsam::Key v_key, gtsam::Key l_key, gtsam::Key d_tube_key,
                       const gtsam::noiseModel::Base::shared_ptr &cost_model,
                       const double l_tube)
      : Base(cost_model, v_key, l_key, d_tube_key), L_(l_tube) {}
  virtual ~ActuatorVolumeFactorId() {}

 public:
  double computeVolume(
      const double &l, const double &d_tube,
      boost::optional<gtsam::Matrix &> H_l = boost::none,
      boost::optional<gtsam::Matrix &> H_d_tube = boost::none) const {
    
    double expected_v = L_ * M_PI * pow(d_tube / 2, 2);
    if (H_d_tube) {
        H_d_tube->setConstant(1, 1, 0.5 * L_* M_PI * d_tube);
    }

    gtsam::Vector4 l_powers(1, l, l * l, l * l * l);
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
      const double &v, const double &l, const double &d_tube,
      boost::optional<gtsam::Matrix &> H_v = boost::none,
      boost::optional<gtsam::Matrix &> H_l = boost::none,
      boost::optional<gtsam::Matrix &> H_d_tube = boost::none) const override {
    double expected_v = computeVolume(l, d_tube, H_l, H_d_tube);
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
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
  }
};




/** MassFlowRateFactorId: compute mdot from pressures*/
class MassFlowRateFactorId
    : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  typedef MassFlowRateFactorId This;
  typedef gtsam::NoiseModelFactor4<double, double, double, double> Base;
  double D_, L_, mu_, epsilon_, k_;
  double term1_, term2_, c1_;

 public:
  MassFlowRateFactorId(gtsam::Key pm_key, gtsam::Key ps_key, gtsam::Key mdot_key,
                     gtsam::Key d_tube_key,
                     const gtsam::noiseModel::Base::shared_ptr &cost_model,
                     const double D, const double L, const double mu,
                     const double epsilon, const double k)
      : Base(cost_model, pm_key, ps_key, mdot_key, d_tube_key),
        D_(D),
        L_(L),
        mu_(mu),
        epsilon_(epsilon),
        k_(k),
        term1_(6.9 / 4 * M_PI * D_ * mu_),
        term2_(pow(epsilon_ / (3.7 * D_), 1.11)),
        c1_(pow(1.8 / log(10), -2)) {}
  virtual ~MassFlowRateFactorId() {}

 public:
  /** compute the mass flow assuming valve is always open
   * fD = [-1.8 log(6.9/Re+(epsilon/3.7D)^1.11)]^-2
   * mdot^2 = (pi^2 D^5)/(16fD L Rs T)/(ps^2-pm^2)
   */
  double computeExpectedMassFlow(
      const double &pm, const double &ps, const double &mdot, const double &d_tube,
      boost::optional<gtsam::Matrix &> H_pm = boost::none,
      boost::optional<gtsam::Matrix &> H_ps = boost::none,
      boost::optional<gtsam::Matrix &> H_mdot = boost::none,
      boost::optional<gtsam::Matrix &> H_d_tube = boost::none) const {
    double tmp = term1_ / abs(mdot) + term2_;
    double fD = c1_ * pow(log(tmp), -2);
    double p_square_diff = abs(ps * ps - pm * pm);
    int sign_p = abs(ps) > abs(pm) ? 1 : -1;
    int sign_mdot = mdot > 0 ? 1 : -1;
    double coeff_ = 1e3 * sqrt(pow(M_PI, 2) * pow(d_tube, 5) * k_ / (16.0 * L_));
    
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
    if (H_d_tube) {
      double coeff = sign_p * sqrt(pow(M_PI, 2) * k_ / (16.0 * L_)) * 1e3 * sqrt(p_square_diff / fD);
      H_d_tube->setConstant(1, 1, 2.5*coeff*pow(d_tube, 1.5));
    }
    return expected_mdot;
  }

  gtsam::Vector evaluateError(
      const double &pm, const double &ps, const double &mdot, const double &d_tube,
      boost::optional<gtsam::Matrix &> H_pm = boost::none,
      boost::optional<gtsam::Matrix &> H_ps = boost::none,
      boost::optional<gtsam::Matrix &> H_mdot = boost::none,
      boost::optional<gtsam::Matrix &> H_d_tube = boost::none) const override {
    double expected_mdot =
        computeExpectedMassFlow(pm, ps, mdot, d_tube, H_pm, H_ps, H_mdot, H_d_tube);
    if (H_mdot) {
      *H_mdot = *H_mdot - gtsam::I_1x1;
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
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
};


}  // namespace gtdynamics