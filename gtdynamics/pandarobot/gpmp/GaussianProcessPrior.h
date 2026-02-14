#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/lexical_cast.hpp>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {
class GaussianProcessPrior
    : public gtsam::NoiseModelFactor4<double, double, double, double> {
 private:
  double delta_t_;

  using This = GaussianProcessPrior;
  using Base = gtsam::NoiseModelFactor4<double, double, double, double>;

 public:
  GaussianProcessPrior() {} /* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPrior(gtsam::Key thetaKey1, gtsam::Key thetadKey1,
                       gtsam::Key thetaKey2, gtsam::Key thetadKey2,
                       double delta_t, double Qc_model)
      :  // TODO:define calcQ
        Base(gtsam::noiseModel::Gaussian::Covariance(calcQ(Qc_model, delta_t)),
             thetaKey1, thetadKey1, thetaKey2, thetadKey2),
        delta_t_(delta_t) {}

  virtual ~GaussianProcessPrior() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// factor error function
  gtsam::Vector evaluateError(
      const double& theta1, const double& thetad1, const double& theta2,
      const double& thetad2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override {

    // state vector
    gtsam::Vector2 error;
    error << -(theta2 - theta1) + delta_t_ * thetad1, -(thetad2 - thetad1);

    // Jacobians
    if (H1) *H1 = (gtsam::Matrix(2, 1) << 1, 0).finished();
    if (H2) *H2 = (gtsam::Matrix(2, 1) << delta_t_, 1).finished();
    if (H3) *H3 = (gtsam::Matrix(2, 1) << -1, 0).finished();
    if (H4) *H4 = (gtsam::Matrix(2, 1) << 0, -1).finished();

    // transition matrix & error
    return error;
  }

  // double calcF(gtsam::Matrix error, gtsam::Matrix Qinv) {
  //   return (1/2 * error.transposed() * Qinv * error);
  // }

  gtsam::Matrix calcQ(double Qc, double delta_t) {
    return (gtsam::Matrix(2, 2) << 1.0 / 3 * pow(delta_t, 3.0) * Qc,
            1.0 / 2 * pow(delta_t, 2.0) * Qc, 1.0 / 2 * pow(delta_t, 2.0) * Qc,
            delta_t * Qc)
        .finished();
  }

  //   gtsam::Matrix calcQ_inv(double Qc, double delta_t) {
  //   return (gtsam::Matrix(2*Qc.rows(), 2*Qc.rows()) <<
  //           12.0 * pow(delta_t, -3.0) / Qc_inv, (-6.0) * pow(delta_t, -2.0) /
  //           Qc_inv,
  //           (-6.0) * pow(delta_t, -2.0) / Qc_inv, 4.0 * pow(delta_t, -1.0) /
  //           Qc_inv).finished();
  // }

  /** demensions */
  size_t dim() const override { return 1; }

  /** number of variables attached to this factor */
  size_t size() const { return 4; }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor& expected,
                      double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "4-way Gaussian Process Factor Linear(" << 1 << ")"
              << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(delta_t_);
  }
#endif

};  // GaussianProcessPrior

}  // namespace gtdynamics
