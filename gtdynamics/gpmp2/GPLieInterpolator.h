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

#include <gtdynamics/gpmp2/GPutils.h>
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
 * states (pose1, vel1) and (pose2, vel2) separated by delta_t, interpolates the
 * pose and velocity at time tau after the first support state.
 */
template <typename T>
class GPLieInterpolator {
 private:
  GTSAM_CONCEPT_ASSERT(gtsam::IsLieGroup<T>);

  using This = GPLieInterpolator<T>;

  size_t dof_;
  double delta_t_;  ///< time between the two support states
  double tau_;      ///< time from the first support state

  gtsam::Matrix Qc_;
  gtsam::Matrix Lambda_;
  gtsam::Matrix Psi_;

 public:
  /// Default constructor, only for serialization.
  GPLieInterpolator() {}

  /**
   * Constructor.
   * @param Qc_model Gaussian noise model whose covariance is Qc. Its dimension
   * must equal the tangent dimension of T, e.g. 6 for Pose3.
   * @param delta_t time between the two support states
   * @param tau time from the first support state to the interpolated state
   */
  GPLieInterpolator(const gtsam::SharedNoiseModel &Qc_model, double delta_t,
                    double tau)
      : dof_(Qc_model->dim()), delta_t_(delta_t), tau_(tau) {
    checkGPInterval(delta_t_, tau_);
    // A mismatched Qc dimension silently mis-slices Lambda and Psi, so reject
    // it up front. Only checkable when T has a fixed dimension.
    if (gtsam::traits<T>::dimension != Eigen::Dynamic &&
        dof_ != static_cast<size_t>(gtsam::traits<T>::dimension)) {
      throw std::invalid_argument(
          "GPLieInterpolator: Qc_model dimension must equal the tangent "
          "dimension of the Lie group.");
    }
    Qc_ = getQc(Qc_model);
    Lambda_ = calcLambdaAccel(Qc_, delta_t_, tau_);
    Psi_ = calcPsiAccel(Qc_, delta_t_, tau_);
  }

  ~GPLieInterpolator() {}

  /// Interpolate the pose at tau, with Jacobians w.r.t. the support states.
  T interpolatePose(const T &pose1, const gtsam::Vector &vel1, const T &pose2,
                    const gtsam::Vector &vel2,
                    gtsam::Matrix *H1 = nullptr,
                    gtsam::Matrix *H2 = nullptr,
                    gtsam::Matrix *H3 = nullptr,
                    gtsam::Matrix *H4 = nullptr) const {
    const bool use_H = (H1 || H2 || H3 || H4);

    gtsam::Vector r1(2 * dof_);
    r1 << gtsam::Vector::Zero(dof_), vel1;

    // Relative increment between the two support poses, in the tangent space.
    gtsam::Matrix Hinv, Hcomp11, Hcomp12, Hlogmap;
    gtsam::Vector r;
    if (use_H) {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1, Hinv),
                                    pose2, Hcomp11, Hcomp12),
          Hlogmap);
    } else {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1), pose2));
    }
    gtsam::Vector r2(2 * dof_);
    r2 << r, vel2;

    const gtsam::Vector xi = Lambda_.block(0, 0, dof_, 2 * dof_) * r1 +
                             Psi_.block(0, 0, dof_, 2 * dof_) * r2;

    if (!use_H) {
      return gtsam::traits<T>::Compose(pose1, gtsam::traits<T>::Expmap(xi));
    }

    gtsam::Matrix Hcomp21, Hcomp22, Hexp;
    const T pose = gtsam::traits<T>::Compose(
        pose1, gtsam::traits<T>::Expmap(xi, Hexp), Hcomp21, Hcomp22);
    const gtsam::Matrix Hexpr1 = Hcomp22 * Hexp;

    if (H1)
      *H1 = Hcomp21 +
            Hexpr1 * Psi_.block(0, 0, dof_, dof_) * Hlogmap * Hcomp11 * Hinv;
    if (H2) *H2 = Hexpr1 * Lambda_.block(0, dof_, dof_, dof_);
    if (H3) *H3 = Hexpr1 * Psi_.block(0, 0, dof_, dof_) * Hlogmap * Hcomp12;
    if (H4) *H4 = Hexpr1 * Psi_.block(0, dof_, dof_, dof_);

    return pose;
  }

  /// Interpolate the velocity at tau, with Jacobians w.r.t. the support states.
  gtsam::Vector interpolateVelocity(
      const T &pose1, const gtsam::Vector &vel1, const T &pose2,
      const gtsam::Vector &vel2, gtsam::Matrix *H1 = nullptr,
      gtsam::Matrix *H2 = nullptr,
      gtsam::Matrix *H3 = nullptr,
      gtsam::Matrix *H4 = nullptr) const {
    const bool use_H = (H1 || H2 || H3 || H4);

    gtsam::Vector r1(2 * dof_);
    r1 << gtsam::Vector::Zero(dof_), vel1;

    gtsam::Matrix Hinv, Hcomp11, Hcomp12, Hlogmap;
    gtsam::Vector r;
    if (use_H) {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1, Hinv),
                                    pose2, Hcomp11, Hcomp12),
          Hlogmap);
    } else {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1), pose2));
    }
    gtsam::Vector r2(2 * dof_);
    r2 << r, vel2;

    if (H1) *H1 = Psi_.block(dof_, 0, dof_, dof_) * Hlogmap * Hcomp11 * Hinv;
    if (H2) *H2 = Lambda_.block(dof_, dof_, dof_, dof_);
    if (H3) *H3 = Psi_.block(dof_, 0, dof_, dof_) * Hlogmap * Hcomp12;
    if (H4) *H4 = Psi_.block(dof_, dof_, dof_, dof_);

    return Lambda_.block(dof_, 0, dof_, 2 * dof_) * r1 +
           Psi_.block(dof_, 0, dof_, 2 * dof_) * r2;
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
    return std::fabs(this->delta_t_ - expected.delta_t_) < tol &&
           std::fabs(this->tau_ - expected.tau_) < tol &&
           gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
           gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
           gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
  }

  /// Print contents.
  void print(const std::string &s = "") const {
    std::cout << s << "GPLieInterpolator<" << dof_ << ">" << std::endl;
    std::cout << "delta_t = " << delta_t_ << ", tau = " << tau_ << std::endl;
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
    ar &BOOST_SERIALIZATION_NVP(delta_t_);
    ar &BOOST_SERIALIZATION_NVP(tau_);
    ar &make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
    ar &make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
    ar &make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
  }
#endif
};  // \class GPLieInterpolator

}  // namespace gtdynamics
