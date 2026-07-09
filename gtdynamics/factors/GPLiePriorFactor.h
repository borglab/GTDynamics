/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPLiePriorFactor.h
 * @brief Gaussian process prior factor on any Lie group, see Barfoot14rss.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/gpmp2/GPutils.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#endif

namespace gtdynamics {

/**
 * A 4-way GaussianProcess prior factor on any Lie group T.
 * Implemented similarly to that used in the GPMP2 paper. Each state consists of
 * a pose in T and a velocity in the tangent space, separated by delta_t. With
 * r = Logmap(Inverse(pose1) * pose2), the error is
 * [r - delta_t * vel1; vel2 - vel1].
 */
template <typename T>
class GPLiePrior
    : public gtsam::NoiseModelFactorN<T, gtsam::Vector, T, gtsam::Vector> {
 private:
  GTSAM_CONCEPT_ASSERT(gtsam::IsLieGroup<T>);

  using This = GPLiePrior<T>;
  using Base = gtsam::NoiseModelFactorN<T, gtsam::Vector, T, gtsam::Vector>;

  size_t dof_;
  double delta_t_;

 public:
  /// Default constructor, only for serialization.
  GPLiePrior() {}

  /**
   * Constructor from the keys of the two states.
   * @param pose_key1 key for the pose of the first state
   * @param vel_key1 key for the velocity of the first state
   * @param pose_key2 key for the pose of the second state
   * @param vel_key2 key for the velocity of the second state
   * @param delta_t time between the two states
   * @param Qc_model Gaussian noise model whose covariance is Qc
   */
  GPLiePrior(gtsam::Key pose_key1, gtsam::Key vel_key1, gtsam::Key pose_key2,
             gtsam::Key vel_key2, double delta_t,
             const gtsam::SharedNoiseModel &Qc_model)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             pose_key1, vel_key1, pose_key2, vel_key2),
        dof_(Qc_model->dim()),
        delta_t_(delta_t) {}

  ~GPLiePrior() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the Gaussian process prior error and its Jacobians.
  gtsam::Vector evaluateError(
      const T &pose1, const gtsam::Vector &vel1, const T &pose2,
      const gtsam::Vector &vel2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override {
    // Relative increment between the two poses, in the tangent space.
    gtsam::Matrix Hinv, Hcomp1, Hcomp2, Hlogmap;
    gtsam::Vector r;
    if (H1 || H2 || H3 || H4) {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1, Hinv),
                                    pose2, Hcomp1, Hcomp2),
          Hlogmap);
    } else {
      r = gtsam::traits<T>::Logmap(
          gtsam::traits<T>::Compose(gtsam::traits<T>::Inverse(pose1), pose2));
    }

    const gtsam::Matrix identity = gtsam::Matrix::Identity(dof_, dof_);
    const gtsam::Matrix zero = gtsam::Matrix::Zero(dof_, dof_);

    if (H1)
      *H1 = (gtsam::Matrix(2 * dof_, dof_) << Hlogmap * Hcomp1 * Hinv, zero)
                .finished();
    if (H2)
      *H2 = (gtsam::Matrix(2 * dof_, dof_) << -delta_t_ * identity, -identity)
                .finished();
    if (H3)
      *H3 =
          (gtsam::Matrix(2 * dof_, dof_) << Hlogmap * Hcomp2, zero).finished();
    if (H4) *H4 = (gtsam::Matrix(2 * dof_, dof_) << zero, identity).finished();

    return (gtsam::Vector(2 * dof_) << (r - vel1 * delta_t_), (vel2 - vel1))
        .finished();
  }

  /// Return the degrees of freedom of a single state.
  size_t dof() const { return dof_; }

  /// Return the time between the two states.
  double deltaT() const { return delta_t_; }

  /// Equality up to a tolerance.
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           std::fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "4-way Gaussian process prior factor on Lie<" << dof_
              << ">" << std::endl;
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
    ar &BOOST_SERIALIZATION_NVP(delta_t_);
  }
#endif
};  // \class GPLiePrior

}  // namespace gtdynamics
