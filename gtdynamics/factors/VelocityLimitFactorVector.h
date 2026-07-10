/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  VelocityLimitFactorVector.h
 * @brief Velocity limit factor on a stacked joint velocity vector.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/gpmp2/JointLimitCost.h>
#include <gtsam/base/Matrix.h>
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

/// Unary factor keeping every entry of a stacked joint velocity vector within
/// its symmetric limit, one error row per joint.
class VelocityLimitFactorVector
    : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = VelocityLimitFactorVector;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  gtsam::Vector vel_limit_, limit_thresh_;

 public:
  /// Default constructor, only for serialization.
  VelocityLimitFactorVector() {}

  /**
   * Constructor.
   * @param v_key key of the stacked joint velocity vector
   * @param cost_model cost function covariance, one row per joint
   * @param vel_limit magnitude of each joint's velocity limit
   * @param limit_thresh standoff kept from each joint's limits
   */
  VelocityLimitFactorVector(gtsam::Key v_key,
                            const gtsam::SharedNoiseModel &cost_model,
                            const gtsam::Vector &vel_limit,
                            const gtsam::Vector &limit_thresh)
      : Base(cost_model, v_key),
        vel_limit_(vel_limit),
        limit_thresh_(limit_thresh) {
    const size_t dof = cost_model->dim();
    if (static_cast<size_t>(vel_limit.size()) != dof ||
        static_cast<size_t>(limit_thresh.size()) != dof) {
      throw std::invalid_argument(
          "VelocityLimitFactorVector: limit vectors must match the noise model "
          "dimension.");
    }
    for (size_t i = 0; i < dof; ++i) {
      // Both hinges would be active everywhere, leaving the factor no zero.
      if (limit_thresh_(i) > vel_limit_(i)) {
        throw std::invalid_argument(
            "VelocityLimitFactorVector: a joint's limit and threshold leave no "
            "feasible interval.");
      }
    }
  }

  ~VelocityLimitFactorVector() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the hinge loss at every joint, and its Jacobian.
  gtsam::Vector evaluateError(
      const gtsam::Vector &v,
      gtsam::OptionalMatrixType H1 = nullptr) const override {
    const size_t dof = this->dof();
    if (static_cast<size_t>(v.size()) != dof) {
      throw std::invalid_argument(
          "VelocityLimitFactorVector: v size must match the limit vectors.");
    }

    if (H1) *H1 = gtsam::Matrix::Zero(dof, dof);
    gtsam::Vector err(dof);
    for (size_t i = 0; i < dof; ++i) {
      if (H1) {
        double H_v;
        err(i) = hingeLossJointLimitCost(v(i), -vel_limit_(i), vel_limit_(i),
                                         limit_thresh_(i), &H_v);
        (*H1)(i, i) = H_v;
      } else {
        err(i) = hingeLossJointLimitCost(v(i), -vel_limit_(i), vel_limit_(i),
                                         limit_thresh_(i));
      }
    }
    return err;
  }

  /// Return the degrees of freedom.
  size_t dof() const { return vel_limit_.size(); }

  const gtsam::Vector &velLimit() const { return vel_limit_; }
  const gtsam::Vector &limitThreshold() const { return limit_thresh_; }

  /// Equality up to a tolerance.
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::equal_with_abs_tol(vel_limit_, e->vel_limit_, tol) &&
           gtsam::equal_with_abs_tol(limit_thresh_, e->limit_thresh_, tol);
  }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "VelocityLimitFactorVector(" << dof() << ")" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "limit threshold: " << limit_thresh_.transpose() << std::endl;
  }

 private:
#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactorN", boost::serialization::base_object<Base>(*this));
  }
#endif
};  // \class VelocityLimitFactorVector

}  // namespace gtdynamics
