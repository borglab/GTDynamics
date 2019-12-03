/**
 *  @file  IntegrationFactor.h
 *  @brief Integrate the acceleration and velocity to get the velocity and joint
 *angle for next time step
 *  @author Yetong Zhang
 **/
#pragma once

#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/lexical_cast.hpp>

#include <string>

namespace robot {

/**
 * 5-way factor for integration
 * Dynamic model:
 * q2 = q1 + v1 * dt + 1/2 * a1 * dt^2
 * v2 = v1 + a1 * dt
 * error:
 * Phi * x - x_plus
 */
class IntegrationFactor
    : public gtsam::NoiseModelFactor5<double, double, double, double, double> {
 private:
  double delta_t_;
  gtsam::Matrix Phi_;

  typedef IntegrationFactor This;
  typedef gtsam::NoiseModelFactor5<double, double, double, double, double> Base;

 public:
  IntegrationFactor() {} /* Default constructor only for serialization */

  /** Constructor
    delta_t is the time between the two states
  */
  IntegrationFactor(gtsam::Key q_key1, gtsam::Key qVel_key1,
                    gtsam::Key qAccel_key1, gtsam::Key q_key2,
                    gtsam::Key qVel_key2,
                    const gtsam::noiseModel::Gaussian::shared_ptr &noise_model,
                    double delta_t)
      : Base(noise_model, q_key1, qVel_key1, qAccel_key1, q_key2, qVel_key2),
        delta_t_(delta_t) {
    Phi_ = (gtsam::Matrix(2, 3) << 1, delta_t, 0.5 * delta_t * delta_t, 0, 1,
            delta_t)
               .finished();
  }

  virtual ~IntegrationFactor() {}

  /** evaluate Gaussian process errors
    Keyword argument:
      q1         -- joint coordinates at time 1
      qVel1      -- joint velocity at time 1
      qAccel1    -- joint acceleration at time 1
      q2         -- joint coordinates at time 2
      qVel2      -- joint velocity at time 2
    */
  gtsam::Vector evaluateError(
      const double &q1, const double &qVel1, const double &qAccel1,
      const double &q2, const double &qVel2,
      boost::optional<gtsam::Matrix &> H_q1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_q2 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel2 = boost::none) const override {
    // state vector
    gtsam::Vector3 x1;
    gtsam::Vector2 x2;
    x1 << q1, qVel1, qAccel1;
    x2 << q2, qVel2;

    // Jacobians
    if (H_q1) *H_q1 = Phi_.col(0);
    if (H_qVel1) *H_qVel1 = Phi_.col(1);
    if (H_qAccel1) *H_qAccel1 = Phi_.col(2);
    if (H_q2) *H_q2 = (gtsam::Matrix(2, 1) << -1, 0).finished();
    if (H_qVel2) *H_qVel2 = (gtsam::Matrix(2, 1) << 0, -1).finished();

    // transition matrix & error
    return Phi_ * x1 - x2;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** equals specialized to this factor */
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "5-way Gaussian Process Factor Linear" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(delta_t_);
  }
};  // IntegrationFactor

/**
 * 6-way factor for integration
 * Dynamic model:
 * q2 = q1 + v1 * dt + 1/2 * a1 * dt^2
 * v2 = v1 + a1 * dt
 * error:
 * Phi * x - x_plus
 */
class SoftIntegrationFactor
    : public gtsam::NoiseModelFactor6<double, double, double, double, double, double> {
 private:
  typedef SoftIntegrationFactor This;
  typedef gtsam::NoiseModelFactor6<double, double, double, double, double, double> Base;

 public:
  SoftIntegrationFactor() {} /* Default constructor only for serialization */

  /** Constructor */
  SoftIntegrationFactor(gtsam::Key q_key1, gtsam::Key qVel_key1,
                    gtsam::Key qAccel_key1, gtsam::Key q_key2,
                    gtsam::Key qVel_key2, gtsam::Key dt_key,
                    const gtsam::noiseModel::Gaussian::shared_ptr &noise_model)
      : Base(noise_model, q_key1, qVel_key1, qAccel_key1, q_key2, qVel_key2, dt_key) {}

  virtual ~SoftIntegrationFactor() {}

  /** evaluate Gaussian process errors
    Keyword argument:
      q1         -- joint coordinates at time 1
      qVel1      -- joint velocity at time 1
      qAccel1    -- joint acceleration at time 1
      q2         -- joint coordinates at time 2
      qVel2      -- joint velocity at time 2
      dt         -- duration for timestep
    */
  gtsam::Vector evaluateError(
      const double &q1, const double &qVel1, const double &qAccel1,
      const double &q2, const double &qVel2, const double &dt,
      boost::optional<gtsam::Matrix &> H_q1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_q2 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel2 = boost::none,
      boost::optional<gtsam::Matrix &> H_dt = boost::none) const override {
    // state vector
    gtsam::Vector3 x1;
    gtsam::Vector2 x2;
    x1 << q1, qVel1, qAccel1;
    x2 << q2, qVel2;
    gtsam::Matrix Phi = (gtsam::Matrix(2, 3) << 1, dt, 0.5 * dt * dt, 0, 1, dt) .finished();
    // Jacobians
    if (H_q1) *H_q1 = Phi.col(0);
    if (H_qVel1) *H_qVel1 = Phi.col(1);
    if (H_qAccel1) *H_qAccel1 = Phi.col(2);
    if (H_q2) *H_q2 = (gtsam::Matrix(2, 1) << -1, 0).finished();
    if (H_qVel2) *H_qVel2 = (gtsam::Matrix(2, 1) << 0, -1).finished();
    if (H_dt) *H_dt = (gtsam::Matrix(2, 1) << qVel1 + qAccel1 * dt, qAccel1).finished();

    // transition matrix & error
    return Phi * x1 - x2;
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** equals specialized to this factor */
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol);
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "5-way Gaussian Process Factor Linear" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};  // SoftIntegrationFactor

}  // namespace robot


/// traits
namespace gtsam {
template <>
struct traits<robot::IntegrationFactor>
    : public Testable<robot::IntegrationFactor> {};
template <>
struct traits<robot::SoftIntegrationFactor>
    : public Testable<robot::SoftIntegrationFactor> {};
}  // namespace gtsam
