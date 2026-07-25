/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPLinearPriorFactor.h
 * @brief Linear Gaussian process prior factor, see Barfoot14rss.
 * @author Karthik Shaji - Adapted from gpmp2 by Xinyan Yan and Jing Dong.
 */

#pragma once

#include <gtdynamics/gpmp2/GPUtils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {

/**
 * A 4-way GaussianProcess prior factor, linear version.
 * Implemented similarly to that used in the GPMP2 paper. Each state consists of
 * a pose and a velocity, separated by deltaT. The error is
 * calcPhiAccel(dof, deltaT) * x1 - x2, where x1 and x2 stack the pose and
 * velocity of the first and second state respectively.
 */
class GPLinearPrior
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
 private:
  using This = GPLinearPrior;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector>;

  size_t dof_;
  double deltaT_;

 public:
  /// Default constructor, only for serialization.
  GPLinearPrior() {}

  /**
   * Constructor from the keys of the two states.
   * @param poseKey1 key for the pose of the first state
   * @param velKey1 key for the velocity of the first state
   * @param poseKey2 key for the pose of the second state
   * @param velKey2 key for the velocity of the second state
   * @param deltaT time between the two states
   * @param QcModel Gaussian noise model whose covariance is Qc
   */
  GPLinearPrior(gtsam::Key poseKey1, gtsam::Key velKey1,
                             gtsam::Key poseKey2, gtsam::Key velKey2,
                             double deltaT,
                             const gtsam::SharedNoiseModel &QcModel)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQAccel(getQc(QcModel), deltaT)),
             poseKey1, velKey1, poseKey2, velKey2),
        dof_(QcModel->dim()),
        deltaT_(deltaT) {
    checkGPDeltaT(deltaT_);
  }

  ~GPLinearPrior() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the Gaussian process prior error and its Jacobians.
  gtsam::Vector evaluateError(
      const gtsam::Vector &pose1, const gtsam::Vector &vel1,
      const gtsam::Vector &pose2, const gtsam::Vector &vel2,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override {
    const gtsam::Matrix identity = gtsam::Matrix::Identity(dof_, dof_);
    const gtsam::Matrix zero = gtsam::Matrix::Zero(dof_, dof_);

    // Stack pose and velocity into the two states.
    gtsam::Vector x1(2 * dof_), x2(2 * dof_);
    x1 << pose1, vel1;
    x2 << pose2, vel2;

    if (H1) *H1 = (gtsam::Matrix(2 * dof_, dof_) << identity, zero).finished();
    if (H2)
      *H2 = (gtsam::Matrix(2 * dof_, dof_) << deltaT_ * identity, identity)
                .finished();
    if (H3) *H3 = (gtsam::Matrix(2 * dof_, dof_) << -identity, zero).finished();
    if (H4) *H4 = (gtsam::Matrix(2 * dof_, dof_) << zero, -identity).finished();

    return calcPhiAccel(dof_, deltaT_) * x1 - x2;
  }

  /// Return the degrees of freedom of a single state.
  size_t dof() const { return dof_; }

  /// Return the time between the two states.
  double deltaT() const { return deltaT_; }

  /// Equality up to a tolerance.
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           std::fabs(this->deltaT_ - e->deltaT_) < tol;
  }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "4-way Gaussian process prior factor, linear(" << dof_
              << ")" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(dof_);
    ar &BOOST_SERIALIZATION_NVP(deltaT_);
  }
#endif
};  // \class GPLinearPrior

}  // namespace gtdynamics

/// traits
namespace gtsam {
template <>
struct traits<gtdynamics::GPLinearPrior>
    : public Testable<gtdynamics::GPLinearPrior> {};
}  // namespace gtsam
