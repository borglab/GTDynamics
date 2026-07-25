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

  gtsam::Vector velLimit_, limitThreshold_;

 public:
  /// Default constructor, only for serialization.
  VelocityLimitFactorVector() {}

  /**
   * Constructor.
   * @param vKey key of the stacked joint velocity vector
   * @param costModel cost function covariance, one row per joint
   * @param velLimit magnitude of each joint's velocity limit
   * @param limitThreshold standoff kept from each joint's limits
   */
  VelocityLimitFactorVector(gtsam::Key vKey,
                            const gtsam::SharedNoiseModel &costModel,
                            const gtsam::Vector &velLimit,
                            const gtsam::Vector &limitThreshold)
      : Base(costModel, vKey),
        velLimit_(velLimit),
        limitThreshold_(limitThreshold) {
    const size_t dof = costModel->dim();
    if (static_cast<size_t>(velLimit.size()) != dof ||
        static_cast<size_t>(limitThreshold.size()) != dof) {
      throw std::invalid_argument(
          "VelocityLimitFactorVector: limit vectors must match the noise model "
          "dimension.");
    }
    for (size_t i = 0; i < dof; ++i) {
      // Both hinges would be active everywhere, leaving the factor no zero.
      if (limitThreshold_(i) > velLimit_(i)) {
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
        double Hv;
        err(i) = hingeLossJointLimitCost(v(i), -velLimit_(i), velLimit_(i),
                                         limitThreshold_(i), &Hv);
        (*H1)(i, i) = Hv;
      } else {
        err(i) = hingeLossJointLimitCost(v(i), -velLimit_(i), velLimit_(i),
                                         limitThreshold_(i));
      }
    }
    return err;
  }

  /// Return the degrees of freedom.
  size_t dof() const { return velLimit_.size(); }

  const gtsam::Vector &velLimit() const { return velLimit_; }
  const gtsam::Vector &limitThreshold() const { return limitThreshold_; }

  /// Equality up to a tolerance.
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::equal_with_abs_tol(velLimit_, e->velLimit_, tol) &&
           gtsam::equal_with_abs_tol(limitThreshold_, e->limitThreshold_, tol);
  }

  /// Print contents.
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "VelocityLimitFactorVector(" << dof() << ")" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "limit threshold: " << limitThreshold_.transpose()
              << std::endl;
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
