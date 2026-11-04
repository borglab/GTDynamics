/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DoubleIntegratorFactor.h
 * @brief Joint-space double integrator dynamics between two states.
 * @author Harrison Jenkins
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
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
 * Double integrator over one step of length dt, per joint: state x = [q; v]
 * (joint angles and velocities) and control u = qddot, held constant over the
 * step. The error is the dynamics residual
 *
 *   error = x2 - Fx x1 - Fu u
 *         = [ q2 - q1 - dt v1 - 0.5 dt^2 u ]
 *           [ v2 - v1 - dt u               ]
 *
 * which is zero exactly when (q2, v2) follows from (q1, v1) under u. It is the
 * linear (LTI) dynamics factor of SCATE (Bhattacharyya et al.), in joint space.
 * The factor is linear, so its Jacobians are constant. Keys are in time order,
 * as in the GP prior factors.
 */
class DoubleIntegratorFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector> {
 private:
  using This = DoubleIntegratorFactor;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector, gtsam::Vector,
                                        gtsam::Vector>;
  double dt_ = 0.0;

 public:
  /// Default constructor, only for serialization.
  DoubleIntegratorFactor() {}

  /**
   * Constructor.
   * @param q_key1 joint angles at the start of the step
   * @param v_key1 joint velocities at the start of the step
   * @param q_key2 joint angles at the end of the step
   * @param v_key2 joint velocities at the end of the step
   * @param u_key joint accelerations applied during the step
   * @param dt step length, must be positive
   * @param cost_model noise model of dimension 2n for n joints
   */
  DoubleIntegratorFactor(gtsam::Key q_key1, gtsam::Key v_key1,
                         gtsam::Key q_key2, gtsam::Key v_key2,
                         gtsam::Key u_key, double dt,
                         const gtsam::SharedNoiseModel &cost_model)
      : Base(cost_model, q_key1, v_key1, q_key2, v_key2, u_key), dt_(dt) {
    if (!(dt > 0.0)) {
      throw std::invalid_argument(
          "DoubleIntegratorFactor: dt must be positive.");
    }
  }

  ~DoubleIntegratorFactor() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Step length.
  double dt() const { return dt_; }

  /**
   * Dynamics residual x2 - Fx x1 - Fu u (size 2n), with its constant
   * Jacobians with respect to q1, v1, q2, v2 and u. Throws if the variables
   * do not all have the same size.
   */
  gtsam::Vector evaluateError(
      const gtsam::Vector &q1, const gtsam::Vector &v1,
      const gtsam::Vector &q2, const gtsam::Vector &v2,
      const gtsam::Vector &u, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr,
      gtsam::OptionalMatrixType H5 = nullptr) const override {
    const long n = q1.size();
    if (v1.size() != n || q2.size() != n || v2.size() != n || u.size() != n) {
      throw std::invalid_argument(
          "DoubleIntegratorFactor: all variables must have the same size.");
    }

    if (H1 || H2 || H3 || H4 || H5) {
      const gtsam::Matrix identity = gtsam::Matrix::Identity(n, n);
      const gtsam::Matrix zero = gtsam::Matrix::Zero(n, n);
      if (H1) *H1 = stackRows(-identity, zero);                // d/dq1
      if (H2) *H2 = stackRows(-dt_ * identity, -identity);     // d/dv1
      if (H3) *H3 = stackRows(identity, zero);                 // d/dq2
      if (H4) *H4 = stackRows(zero, identity);                 // d/dv2
      if (H5) *H5 = stackRows(-0.5 * dt_ * dt_ * identity,     // d/du
                              -dt_ * identity);
    }

    gtsam::Vector error(2 * n);
    error.head(n) = q2 - q1 - dt_ * v1 - 0.5 * dt_ * dt_ * u;
    error.tail(n) = v2 - v1 - dt_ * u;
    return error;
  }

  /// Equality up to a tolerance.
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           std::abs(dt_ - e->dt_) <= tol;
  }

  /// Print the step length and the keys.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "DoubleIntegratorFactor(dt = " << dt_ << ")"
              << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Stack two n x n blocks into one 2n x n matrix.
  static gtsam::Matrix stackRows(const gtsam::Matrix &top,
                                 const gtsam::Matrix &bottom) {
    gtsam::Matrix stacked(top.rows() + bottom.rows(), top.cols());
    stacked << top, bottom;
    return stacked;
  }

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(dt_);
  }
#endif
};  // \class DoubleIntegratorFactor

}  // namespace gtdynamics
