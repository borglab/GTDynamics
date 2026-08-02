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
#include <functional>
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
 * and velocity at time tau after the first support state. Only the cached
 * scalar coefficients of Lambda and Psi are stored, since Qc cancels.
 */
class GPLinearInterpolator {
 private:
  using This = GPLinearInterpolator;

  size_t dof_;
  double deltaT_;  ///< time between the two support states
  double tau_;     ///< time from the first support state

  gtsam::Matrix2 lambda_, psi_;  ///< scalar block coefficients

 public:
  /// Default constructor, only for serialization.
  GPLinearInterpolator() {}

  /**
   * Constructor.
   * @param QcModel Gaussian noise model, only its dimension is used
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  GPLinearInterpolator(const gtsam::SharedNoiseModel &QcModel, double deltaT,
                       double tau)
      : dof_(QcModel->dim()), deltaT_(deltaT), tau_(tau) {
    calcInterpCoefficientsAccel(deltaT_, tau_, &lambda_, &psi_);
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
    if (H1) *H1 = lambda_(0, 0) * gtsam::Matrix::Identity(dof_, dof_);
    if (H2) *H2 = lambda_(0, 1) * gtsam::Matrix::Identity(dof_, dof_);
    if (H3) *H3 = psi_(0, 0) * gtsam::Matrix::Identity(dof_, dof_);
    if (H4) *H4 = psi_(0, 1) * gtsam::Matrix::Identity(dof_, dof_);
    return lambda_(0, 0) * pose1 + lambda_(0, 1) * vel1 + psi_(0, 0) * pose2 +
           psi_(0, 1) * vel2;
  }

  /// Interpolate the velocity at tau, with Jacobians w.r.t. the support states.
  gtsam::Vector interpolateVelocity(
      const gtsam::Vector &pose1, const gtsam::Vector &vel1,
      const gtsam::Vector &pose2, const gtsam::Vector &vel2,
      gtsam::Matrix *H1 = nullptr,
      gtsam::Matrix *H2 = nullptr,
      gtsam::Matrix *H3 = nullptr,
      gtsam::Matrix *H4 = nullptr) const {
    if (H1) *H1 = lambda_(1, 0) * gtsam::Matrix::Identity(dof_, dof_);
    if (H2) *H2 = lambda_(1, 1) * gtsam::Matrix::Identity(dof_, dof_);
    if (H3) *H3 = psi_(1, 0) * gtsam::Matrix::Identity(dof_, dof_);
    if (H4) *H4 = psi_(1, 1) * gtsam::Matrix::Identity(dof_, dof_);
    return lambda_(1, 0) * pose1 + lambda_(1, 1) * vel1 + psi_(1, 0) * pose2 +
           psi_(1, 1) * vel2;
  }

  /// Chain the Jacobian of a cost at the interpolated pose back to the states.
  void updatePoseJacobians(const gtsam::Matrix &Hpose, gtsam::Matrix *H1,
                           gtsam::Matrix *H2, gtsam::Matrix *H3,
                           gtsam::Matrix *H4) const {
    if (H1) *H1 = lambda_(0, 0) * Hpose;
    if (H2) *H2 = lambda_(0, 1) * Hpose;
    if (H3) *H3 = psi_(0, 0) * Hpose;
    if (H4) *H4 = psi_(0, 1) * Hpose;
  }

  /// Evaluate errorAt at the interpolated pose and chain its Jacobian with
  /// respect to that pose back to the four support state Jacobians.
  gtsam::Vector errorAtInterpolatedPose(
      const gtsam::Vector &pose1, const gtsam::Vector &vel1,
      const gtsam::Vector &pose2, const gtsam::Vector &vel2,
      const std::function<gtsam::Vector(const gtsam::Vector &q,
                                        gtsam::Matrix *Hq)> &errorAt,
      gtsam::Matrix *H1, gtsam::Matrix *H2, gtsam::Matrix *H3,
      gtsam::Matrix *H4) const {
    const bool computeJacobians = (H1 || H2 || H3 || H4);
    gtsam::Matrix Hq;
    const gtsam::Vector err =
        errorAt(interpolatePose(pose1, vel1, pose2, vel2),
                computeJacobians ? &Hq : nullptr);
    if (computeJacobians) updatePoseJacobians(Hq, H1, H2, H3, H4);
    return err;
  }

  /// Return the degrees of freedom of a single state.
  size_t dim() const { return dof_; }

  /// Equality up to a tolerance.
  bool equals(const This &expected, double tol = 1e-9) const {
    return std::fabs(this->deltaT_ - expected.deltaT_) < tol &&
           std::fabs(this->tau_ - expected.tau_) < tol &&
           gtsam::equal_with_abs_tol(this->lambda_, expected.lambda_, tol) &&
           gtsam::equal_with_abs_tol(this->psi_, expected.psi_, tol);
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
    ar &make_nvp("lambda", make_array(lambda_.data(), lambda_.size()));
    ar &make_nvp("psi", make_array(psi_.data(), psi_.size()));
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
