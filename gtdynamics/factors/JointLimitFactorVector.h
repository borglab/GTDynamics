/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointLimitFactorVector.h
 * @brief Joint limit factor on a stacked joint angle vector.
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

/// Unary factor keeping every entry of a stacked joint angle vector within its
/// limits, one error row per joint.
class JointLimitFactorVector : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  using This = JointLimitFactorVector;
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;

  gtsam::Vector down_limit_, up_limit_, limit_thresh_;

 public:
  /// Default constructor, only for serialization.
  JointLimitFactorVector() {}

  /**
   * Constructor.
   * @param q_key key of the stacked joint angle vector
   * @param cost_model cost function covariance, one row per joint
   * @param down_limit lower limit of each joint
   * @param up_limit upper limit of each joint
   * @param limit_thresh standoff kept from each joint's limits
   */
  JointLimitFactorVector(gtsam::Key q_key,
                         const gtsam::SharedNoiseModel &cost_model,
                         const gtsam::Vector &down_limit,
                         const gtsam::Vector &up_limit,
                         const gtsam::Vector &limit_thresh)
      : Base(cost_model, q_key),
        down_limit_(down_limit),
        up_limit_(up_limit),
        limit_thresh_(limit_thresh) {
    const size_t dof = cost_model->dim();
    if (static_cast<size_t>(down_limit.size()) != dof ||
        static_cast<size_t>(up_limit.size()) != dof ||
        static_cast<size_t>(limit_thresh.size()) != dof) {
      throw std::invalid_argument(
          "JointLimitFactorVector: limit vectors must match the noise model "
          "dimension.");
    }
    for (size_t i = 0; i < dof; ++i) {
      // Both hinges would be active everywhere, leaving the factor no zero.
      if (down_limit_(i) + limit_thresh_(i) > up_limit_(i) - limit_thresh_(i)) {
        throw std::invalid_argument(
            "JointLimitFactorVector: a joint's limits and threshold leave no "
            "feasible interval.");
      }
    }
  }

  ~JointLimitFactorVector() override {}

  /// Return a deep copy of this factor.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// Evaluate the hinge loss at every joint, and its Jacobian.
  gtsam::Vector evaluateError(
      const gtsam::Vector &q,
      gtsam::OptionalMatrixType H1 = nullptr) const override {
    const size_t dof = this->dof();
    if (static_cast<size_t>(q.size()) != dof) {
      throw std::invalid_argument(
          "JointLimitFactorVector: q size must match the limit vectors.");
    }

    if (H1) *H1 = gtsam::Matrix::Zero(dof, dof);
    gtsam::Vector err(dof);
    for (size_t i = 0; i < dof; ++i) {
      if (H1) {
        double H_q;
        err(i) = hingeLossJointLimitCost(q(i), down_limit_(i), up_limit_(i),
                                         limit_thresh_(i), &H_q);
        (*H1)(i, i) = H_q;
      } else {
        err(i) = hingeLossJointLimitCost(q(i), down_limit_(i), up_limit_(i),
                                         limit_thresh_(i));
      }
    }
    return err;
  }

  /// Return the degrees of freedom.
  size_t dof() const { return down_limit_.size(); }

  const gtsam::Vector &downLimit() const { return down_limit_; }
  const gtsam::Vector &upLimit() const { return up_limit_; }
  const gtsam::Vector &limitThreshold() const { return limit_thresh_; }

  /// Equality up to a tolerance.
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::equal_with_abs_tol(down_limit_, e->down_limit_, tol) &&
           gtsam::equal_with_abs_tol(up_limit_, e->up_limit_, tol) &&
           gtsam::equal_with_abs_tol(limit_thresh_, e->limit_thresh_, tol);
  }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "JointLimitFactorVector(" << dof() << ")" << std::endl;
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
};  // \class JointLimitFactorVector

}  // namespace gtdynamics
