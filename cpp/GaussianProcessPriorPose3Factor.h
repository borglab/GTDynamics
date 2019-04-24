/**
 *  @file  GaussianProcessPriorPose3Factor.h
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
 * 6-way factor for Gaussian Process prior factor, Pose3 version
 * Dynamic model:
 * x is the state, which contains pose, twist, and acceleration
 * x_plus = Phi * x + noise
 * error:
 * Phi * x - x_plus
 */
class GaussianProcessPriorPose3Factor
    : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector6,
                                      gtsam::Vector6, gtsam::Pose3,
                                      gtsam::Vector6, gtsam::Vector6> {
 private:
  double delta_t_;
  double delta_t_square_;

  typedef GaussianProcessPriorPose3Factor This;
  typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector6, gtsam::Vector6,
                                   gtsam::Pose3, gtsam::Vector6, gtsam::Vector6>
      Base;

 public:
  GaussianProcessPriorPose3Factor() {
  } /* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorPose3Factor(
      gtsam::Key p1_key, gtsam::Key v1_key, gtsam::Key vdot1_key,
      gtsam::Key p2_key, gtsam::Key v2_key, gtsam::Key vdot2_key,
      const gtsam::noiseModel::Gaussian::shared_ptr &Qc_model, double delta_t)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             p1_key, v1_key, vdot1_key, p2_key, v2_key, vdot2_key),
        delta_t_(delta_t), delta_t_square_(delta_t*delta_t) {}

  virtual ~GaussianProcessPriorPose3Factor() {}

  /** evaluate Gaussian process errors
      Keyword argument:
        p1         -- pose at time 1
        v1         -- velocity at time 1
        v1dot         -- acceleration at time 1
        p2         -- pose at time 2
        v2         -- velocity at time 2
        v2dot         -- acceleration at time 2
    */
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &p1, const gtsam::Vector6 &v1,
      const gtsam::Vector6 &v1dot, const gtsam::Pose3 &p2,
      const gtsam::Vector6 &v2, const gtsam::Vector6 &v2dot,
      boost::optional<gtsam::Matrix &> H_p1 = boost::none,
      boost::optional<gtsam::Matrix &> H_v1 = boost::none,
      boost::optional<gtsam::Matrix &> H_v1dot = boost::none,
      boost::optional<gtsam::Matrix &> H_p2 = boost::none,
      boost::optional<gtsam::Matrix &> H_v2 = boost::none,
      boost::optional<gtsam::Matrix &> H_v2dot = boost::none) const override {
    gtsam::Matrix6 Hinv, Hcomp1, Hcomp2, Hlogmap;
    gtsam::Vector6 r;
    if (H_p1 || H_p2) {
      r = gtsam::Pose3::Logmap(p1.inverse(Hinv).compose(p2, Hcomp1, Hcomp2),
                               Hlogmap);
    } else {
      r = gtsam::Pose3::Logmap(p1.inverse() * p2);
    }

    if (H_p1)
      *H_p1 = (gtsam::Matrix(3 * 6, 6) << Hlogmap * Hcomp1 * Hinv,
               gtsam::Z_6x6, gtsam::Z_6x6)
                  .finished();
    if (H_v1)
      *H_v1 =
          (gtsam::Matrix(3 * 6, 6) << -delta_t_ * gtsam::I_6x6,
           -gtsam::I_6x6)
              .finished();
    if (H_v1dot)
      *H_v1dot = (gtsam::Matrix(3 * 6, 6) << -0.5 * delta_t_square_ *
                                              gtsam::I_6x6,
               -delta_t_ * gtsam::I_6x6,
               -gtsam::I_6x6)
                  .finished();
    if (H_p2)
      *H_p2 = (gtsam::Matrix(3 * 6, 6) << Hlogmap * Hcomp2,
               gtsam::Z_6x6, gtsam::Z_6x6)
                  .finished();
    if (H_v2)
      *H_v2 = (gtsam::Matrix(3 * 6, 6) << gtsam::Z_6x6,
               gtsam::I_6x6, gtsam::Z_6x6)
                  .finished();
    if (H_v2dot)
      *H_v2dot = (gtsam::Matrix(3 * 6, 6) << gtsam::Z_6x6,
               gtsam::Z_6x6, gtsam::I_6x6)
                  .finished();

    return (gtsam::Vector(3 * 6)
                << (r - v1 * delta_t_ - v1dot * 0.5 * delta_t_square_),
            (v2 - v1 - v1dot * delta_t_), v2dot - v1dot)
        .finished();
  }

  // @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override{
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
    std::cout << s << "6-way Gaussian Process Factor Pose3" << std::endl;
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

};  // GaussianProcessPriorPose3Factor

}  // namespace manipulator

/// traits
namespace gtsam {
template <>
struct traits<manipulator::GaussianProcessPriorPose3Factor>
    : public Testable<manipulator::GaussianProcessPriorPose3Factor> {};
}  // namespace gtsam
