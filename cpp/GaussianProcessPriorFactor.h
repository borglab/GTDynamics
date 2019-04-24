/**
 *  @file  GaussianProcessPriorFactor.h
 *  @brief Linear GP prior, see Barfoot14rss
 *  @author Mandy Xie and Frank Dellaert
 **/
#pragma once

#include <utils.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/lexical_cast.hpp>

namespace manipulator {

/**
 * 6-way factor for Gaussian Process prior factor, linear version
 * Dynamic model:
 * x_plus = Phi * x + noise
 * error:
 * Phi * x - x_plus
 */
class GaussianProcessPriorFactor
    : public gtsam::NoiseModelFactor6<double, double, double, double, double,
                                      double> {
 private:
  double delta_t_;
  gtsam::Matrix3 Phi_;

  typedef GaussianProcessPriorFactor This;
  typedef gtsam::NoiseModelFactor6<double, double, double, double, double,
                                   double>
      Base;

 public:
  GaussianProcessPriorFactor() {
  } /* Default constructor only for serialization */

  /** Constructor
    delta_t is the time between the two states
  */
  GaussianProcessPriorFactor(
      gtsam::Key q_key1, gtsam::Key qVel_key1, gtsam::Key qAccel_key1,
      gtsam::Key q_key2, gtsam::Key qVel_key2, gtsam::Key qAccel_key2,
      const gtsam::noiseModel::Gaussian::shared_ptr &Qc_model, double delta_t)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             q_key1, qVel_key1, qAccel_key1, q_key2, qVel_key2, qAccel_key2),
        delta_t_(delta_t) {
    Phi_ = calcPhi(delta_t_);
  }

  virtual ~GaussianProcessPriorFactor() {}

  /** evaluate Gaussian process errors
    Keyword argument:
      q1         -- joint coordinates at time 1
      qVel1      -- joint velocity at time 1
      qAccel1    -- joint acceleration at time 1
      q2         -- joint coordinates at time 2
      qVel2      -- joint velocity at time 2
      qAccel2    -- joint acceleration at time 2
    */
  gtsam::Vector evaluateError(
      const double &q1, const double &qVel1, const double &qAccel1,
      const double &q2, const double &qVel2, const double &qAccel2,
      boost::optional<gtsam::Matrix &> H_q1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel1 = boost::none,
      boost::optional<gtsam::Matrix &> H_q2 = boost::none,
      boost::optional<gtsam::Matrix &> H_qVel2 = boost::none,
      boost::optional<gtsam::Matrix &> H_qAccel2 = boost::none) const override {
    // state vector
    gtsam::Vector3 x1, x2;
    x1 << q1, qVel1, qAccel1;
    x2 << q2, qVel2, qAccel2;

    // Jacobians
    if (H_q1) *H_q1 = Phi_.col(0);
    if (H_qVel1) *H_qVel1 = Phi_.col(1);
    if (H_qAccel1) *H_qAccel1 = Phi_.col(2);
    if (H_q2) *H_q2 = (gtsam::Matrix(3, 1) << -1, 0, 0).finished();
    if (H_qVel2) *H_qVel2 = (gtsam::Matrix(3, 1) << 0, -1, 0).finished();
    if (H_qAccel2) *H_qAccel2 = (gtsam::Matrix(3, 1) << 0, 0, -1).finished();

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
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "6-way Gaussian Process Factor Linear" << std::endl;
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

};  // GaussianProcessPriorFactor

}  // namespace manipulator

/// traits
namespace gtsam {
template <>
struct traits<manipulator::GaussianProcessPriorFactor>
    : public Testable<manipulator::GaussianProcessPriorFactor> {};
}  // namespace gtsam
