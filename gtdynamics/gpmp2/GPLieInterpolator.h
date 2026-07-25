/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPLieInterpolator.h
 * @brief Gaussian process interpolator on any Lie group.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/gpmp2/GPUtils.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/concepts.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {

/**
 * Gaussian process interpolator on any Lie group T. Given the two support
 * states (pose1, vel1) and (pose2, vel2) separated by deltaT, interpolates the
 * pose and velocity at time tau after the first support state. Only the cached
 * scalar coefficients of Lambda and Psi are stored, since Qc cancels.
 */
template <typename T>
class GPLieInterpolator {
 private:
  GTSAM_CONCEPT_ASSERT(gtsam::IsLieGroup<T>);

  using This = GPLieInterpolator<T>;

  size_t dof_;
  double deltaT_;  ///< time between the two support states
  double tau_;     ///< time from the first support state

  gtsam::Matrix2 lambda_, psi_;  ///< scalar block coefficients

 public:
  /// Default constructor, only for serialization.
  GPLieInterpolator() {}

  /**
   * Constructor.
   * @param QcModel Gaussian noise model, only its dimension is used. It must
   * equal the tangent dimension of T, e.g. 6 for Pose3.
   * @param deltaT time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  GPLieInterpolator(const gtsam::SharedNoiseModel &QcModel, double deltaT,
                    double tau)
      : dof_(QcModel->dim()), deltaT_(deltaT), tau_(tau) {
    // A mismatched Qc dimension would silently mis-size the Jacobians, so
    // reject it up front. Only checkable when T has a fixed dimension.
    if (gtsam::traits<T>::dimension != Eigen::Dynamic &&
        dof_ != static_cast<size_t>(gtsam::traits<T>::dimension)) {
      throw std::invalid_argument(
          "GPLieInterpolator: QcModel dimension must equal the tangent "
          "dimension of the Lie group.");
    }
    calcInterpCoefficientsAccel(deltaT_, tau_, &lambda_, &psi_);
  }

  ~GPLieInterpolator() {}

  /// Interpolate the pose at tau, with Jacobians w.r.t. the support states.
  T interpolatePose(const T &pose1, const gtsam::Vector &vel1, const T &pose2,
                    const gtsam::Vector &vel2,
                    gtsam::Matrix *H1 = nullptr,
                    gtsam::Matrix *H2 = nullptr,
                    gtsam::Matrix *H3 = nullptr,
                    gtsam::Matrix *H4 = nullptr) const {
    const bool computeJacobians = (H1 || H2 || H3 || H4);

    // Relative increment between the two support poses, in the tangent space.
    gtsam::Matrix Hinv, Hcomp11, Hcomp12, Hlogmap;
    gtsam::Vector r;
    if (computeJacobians) {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1, Hinv),
                                    pose2, Hcomp11, Hcomp12),
          Hlogmap);
    } else {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1), pose2));
    }

    const gtsam::Vector xi =
        lambda_(0, 1) * vel1 + psi_(0, 0) * r + psi_(0, 1) * vel2;

    if (!computeJacobians) {
      return gtsam::traits<T>::Compose(pose1, gtsam::traits<T>::Expmap(xi));
    }

    gtsam::Matrix Hcomp21, Hcomp22, Hexp;
    const T pose = gtsam::traits<T>::Compose(
        pose1, gtsam::traits<T>::Expmap(xi, Hexp), Hcomp21, Hcomp22);
    const gtsam::Matrix Hexpr1 = Hcomp22 * Hexp;

    if (H1)
      *H1 = Hcomp21 + psi_(0, 0) * Hexpr1 * Hlogmap * Hcomp11 * Hinv;
    if (H2) *H2 = lambda_(0, 1) * Hexpr1;
    if (H3) *H3 = psi_(0, 0) * Hexpr1 * Hlogmap * Hcomp12;
    if (H4) *H4 = psi_(0, 1) * Hexpr1;

    return pose;
  }

  /// Interpolate the velocity at tau, with Jacobians w.r.t. the support states.
  gtsam::Vector interpolateVelocity(
      const T &pose1, const gtsam::Vector &vel1, const T &pose2,
      const gtsam::Vector &vel2, gtsam::Matrix *H1 = nullptr,
      gtsam::Matrix *H2 = nullptr,
      gtsam::Matrix *H3 = nullptr,
      gtsam::Matrix *H4 = nullptr) const {
    const bool computeJacobians = (H1 || H2 || H3 || H4);

    gtsam::Matrix Hinv, Hcomp11, Hcomp12, Hlogmap;
    gtsam::Vector r;
    if (computeJacobians) {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1, Hinv),
                                    pose2, Hcomp11, Hcomp12),
          Hlogmap);
    } else {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1), pose2));
    }

    if (H1) *H1 = psi_(1, 0) * Hlogmap * Hcomp11 * Hinv;
    if (H2) *H2 = lambda_(1, 1) * gtsam::Matrix::Identity(dof_, dof_);
    if (H3) *H3 = psi_(1, 0) * Hlogmap * Hcomp12;
    if (H4) *H4 = psi_(1, 1) * gtsam::Matrix::Identity(dof_, dof_);

    return lambda_(1, 1) * vel1 + psi_(1, 0) * r + psi_(1, 1) * vel2;
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
           gtsam::equal_with_abs_tol(this->lambda_, expected.lambda_, tol) &&
           gtsam::equal_with_abs_tol(this->psi_, expected.psi_, tol);
  }

  /// Print contents.
  void print(const std::string &s = "") const {
    std::cout << s << "GPLieInterpolator<" << dof_ << ">" << std::endl;
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
};  // \class GPLieInterpolator

}  // namespace gtdynamics
