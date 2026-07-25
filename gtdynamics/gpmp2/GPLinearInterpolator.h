/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPLinearInterpolator.h
 * @brief Gaussian process interpolator, linear version.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong and Xinyan Yan.
 */

#pragma once

#include <gtdynamics/gpmp2/GPUtils.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>

#include <cmath>
#include <iostream>
#include <string>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {

/**
 * Gaussian process interpolator, linear version. Given the two support states
 * (pose1, vel1) and (pose2, vel2) separated by deltaT, interpolates the pose
 * and velocity at time tau after the first support state.
 */
class GPLinearInterpolator {
 private:
  using This = GPLinearInterpolator;

  size_t dof_;
  double deltaT_;  ///< time between the two support states
  double tau_;      ///< time from the first support state

  gtsam::Matrix Qc_;
  gtsam::Matrix Lambda_;
  gtsam::Matrix Psi_;

 public:
  /// Default constructor, only for serialization.
  GPLinearInterpolator() {}

  /**
   * Constructor.
   * @param QcModel Gaussian noise model whose covariance is Qc
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  GPLinearInterpolator(const gtsam::SharedNoiseModel &QcModel, double deltaT,
                       double tau)
      : dof_(QcModel->dim()), deltaT_(deltaT), tau_(tau) {
    checkGPInterval(deltaT_, tau_);
    Qc_ = getQc(QcModel);
    Lambda_ = calcLambdaAccel(Qc_, deltaT_, tau_);
    Psi_ = calcPsiAccel(Qc_, deltaT_, tau_);
  }

  ~GPLinearInterpolator() {}

  /// Interpolate the pose at tau, with Jacobians w.r.t. the support states.
  gtsam::Vector interpolatePose(
      const gtsam::Vector &pose1, const gtsam::Vector &vel1,
      const gtsam::Vector &pose2, const gtsam::Vector &vel2,
      gtsam::Matrix *H1 = nullptr,
      gtsam::Matrix *H2 = nullptr,
      gtsam::Matrix *H3 = nullptr,
      gtsam::Matrix *H4 = nullptr) const {
    gtsam::Vector x1(2 * dof_), x2(2 * dof_);
    x1 << pose1, vel1;
    x2 << pose2, vel2;

    if (H1) *H1 = Lambda_.block(0, 0, dof_, dof_);
    if (H2) *H2 = Lambda_.block(0, dof_, dof_, dof_);
    if (H3) *H3 = Psi_.block(0, 0, dof_, dof_);
    if (H4) *H4 = Psi_.block(0, dof_, dof_, dof_);

    // Only the upper block of the interpolated state is needed.
    return Lambda_.block(0, 0, dof_, 2 * dof_) * x1 +
           Psi_.block(0, 0, dof_, 2 * dof_) * x2;
  }

  /// Interpolate the velocity at tau, with Jacobians w.r.t. the support states.
  gtsam::Vector interpolateVelocity(
      const gtsam::Vector &pose1, const gtsam::Vector &vel1,
      const gtsam::Vector &pose2, const gtsam::Vector &vel2,
      gtsam::Matrix *H1 = nullptr,
      gtsam::Matrix *H2 = nullptr,
      gtsam::Matrix *H3 = nullptr,
      gtsam::Matrix *H4 = nullptr) const {
    gtsam::Vector x1(2 * dof_), x2(2 * dof_);
    x1 << pose1, vel1;
    x2 << pose2, vel2;

    if (H1) *H1 = Lambda_.block(dof_, 0, dof_, dof_);
    if (H2) *H2 = Lambda_.block(dof_, dof_, dof_, dof_);
    if (H3) *H3 = Psi_.block(dof_, 0, dof_, dof_);
    if (H4) *H4 = Psi_.block(dof_, dof_, dof_, dof_);

    // Only the lower block of the interpolated state is needed.
    return Lambda_.block(dof_, 0, dof_, 2 * dof_) * x1 +
           Psi_.block(dof_, 0, dof_, 2 * dof_) * x2;
  }

  /// Chain the Jacobian of a cost at the interpolated pose back to the states.
  static void updatePoseJacobians(
      const gtsam::Matrix &Hpose, const gtsam::Matrix &Hint1,
      const gtsam::Matrix &Hint2, const gtsam::Matrix &Hint3,
      const gtsam::Matrix &Hint4, gtsam::Matrix *H1,
      gtsam::Matrix *H2, gtsam::Matrix *H3,
      gtsam::Matrix *H4) {
    if (H1) *H1 = Hpose * Hint1;
    if (H2) *H2 = Hpose * Hint2;
    if (H3) *H3 = Hpose * Hint3;
    if (H4) *H4 = Hpose * Hint4;
  }

  /// Return the degrees of freedom of a single state.
  size_t dim() const { return dof_; }

  /// Equality up to a tolerance.
  bool equals(const This &expected, double tol = 1e-9) const {
    return std::fabs(this->deltaT_ - expected.deltaT_) < tol &&
           std::fabs(this->tau_ - expected.tau_) < tol &&
           gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
           gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
           gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
  }

  /// Print contents.
  void print(const std::string &s = "") const {
    std::cout << s << "GPLinearInterpolator(" << dof_ << ")" << std::endl;
    std::cout << "deltaT = " << deltaT_ << ", tau = " << tau_ << std::endl;
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {  // NOLINT
    using boost::serialization::make_array;
    using boost::serialization::make_nvp;
    ar &BOOST_SERIALIZATION_NVP(dof_);
    ar &BOOST_SERIALIZATION_NVP(deltaT_);
    ar &BOOST_SERIALIZATION_NVP(tau_);
    ar &make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
    ar &make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
    ar &make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
  }
#endif
};  // \class GPLinearInterpolator

}  // namespace gtdynamics

/// traits
namespace gtsam {
template <>
struct traits<gtdynamics::GPLinearInterpolator>
    : public Testable<gtdynamics::GPLinearInterpolator> {};
}  // namespace gtsam
