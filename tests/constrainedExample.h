/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  constrainedExample.h
 * @brief Examples for constrained optimization. From
 * http://www.seas.ucla.edu/~vandenbe/133B/lectures/nllseq.pdf
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtdynamics {
namespace constrained_example {

using gtsam::Double_;
using gtsam::Key;
using gtsam::Symbol;

double cost1(const double &x1, const double &x2,
             gtsam::OptionalJacobian<1, 1> H_x1 = boost::none,
             gtsam::OptionalJacobian<1, 1> H_x2 = boost::none) {
  double result = x1 + exp(-x2);
  if (H_x1) H_x1->setConstant(1.0);
  if (H_x2) H_x2->setConstant(-exp(-x2));
  return result;
}

double cost2(const double &x1, const double &x2,
             gtsam::OptionalJacobian<1, 1> H_x1 = boost::none,
             gtsam::OptionalJacobian<1, 1> H_x2 = boost::none) {
  double result = x1 * x1 + 2 * x2 + 1;
  if (H_x1) H_x1->setConstant(2 * x1);
  if (H_x2) H_x2->setConstant(2.0);
  return result;
}

double constraint1(const double &x1, const double &x2,
                   gtsam::OptionalJacobian<1, 1> H_x1 = boost::none,
                   gtsam::OptionalJacobian<1, 1> H_x2 = boost::none) {
  double result = x1 + x1 * x1 * x1 + x2 + x2 * x2;
  if (H_x1) H_x1->setConstant(1 + 3 * x1 * x1);
  if (H_x2) H_x2->setConstant(1 + 2 * x2);
  return result;
}

Symbol x1_key('x', 1);
Symbol x2_key('x', 2);

Double_ x1_expr(x1_key);
Double_ x2_expr(x2_key);
Double_ cost1_expr(cost1, x1_expr, x2_expr);
Double_ cost2_expr(cost2, x1_expr, x2_expr);
Double_ constraint1_expr(constraint1, x1_expr, x2_expr);

class AuglConstraint1Factor : public gtsam::NoiseModelFactor2<double, double> {
 private:
  using This = AuglConstraint1Factor;
  using Base = gtsam::NoiseModelFactor2<double, double>;
  double constant_;

 public:
  AuglConstraint1Factor(Key x1_key, Key x2_key,
                        const gtsam::noiseModel::Base::shared_ptr &cost_model,
                        const double &constant)
      : Base(cost_model, x1_key, x2_key), constant_(constant) {}

  virtual ~AuglConstraint1Factor() {}

 public:
  gtsam::Vector evaluateError(
      const double &x1, const double &x2,
      boost::optional<gtsam::Matrix &> H_x1 = boost::none,
      boost::optional<gtsam::Matrix &> H_x2 = boost::none) const override {
    double result = x1 + x1 * x1 * x1 + x2 + x2 * x2 + constant_;
    if (H_x1) H_x1->setConstant(1, 1, 1 + 3 * x1 * x1);
    if (H_x2) H_x2->setConstant(1, 1, 1 + 2 * x2);
    return gtsam::Vector1(result);
  }

  //// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print contents
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "AuglConstraint1Factor" << std::endl;
    Base::print("", keyFormatter);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {  // NOLINT
    ar &boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace constrained_example

}  // namespace gtdynamics