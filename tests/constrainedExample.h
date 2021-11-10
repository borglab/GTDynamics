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
using gtsam::Expression;
using gtsam::Key;
using gtsam::Symbol;
using gtsam::Vector1;
using gtsam::Vector2;
using gtsam::Vector2_;

/// Exponential function e^x.
double exp_func(const double& x,
                gtsam::OptionalJacobian<1, 1> H1 = boost::none) {
  double result = exp(x);
  if (H1) H1->setConstant(result);
  return result;
}

/// Exponential expression e^x.
Double_ exp(const Double_& x) { return Double_(exp_func, x); }

/// Pow functor used for pow function.
class PowFunctor {
 private:
  double c_;

 public:
  PowFunctor(const double& c) : c_(c) {}

  double operator()(const double& x,
                    gtsam::OptionalJacobian<1, 1> H1 = boost::none) const {
    if (H1) H1->setConstant(c_ * pow(x, c_ - 1));
    return pow(x, c_);
  }
};

/// Pow function.
Double_ pow(const Double_& x, const double& c) {
  PowFunctor pow_functor(c);
  return Double_(pow_functor, x);
}

/// Plus between Double expression and double.
Double_ operator+(const Double_& x, const double& d) { return x + Double_(d); }

/// Negative sign operator.
Double_ operator-(const Double_& x) { return Double_(0.0) - x; }

/// Keys for creating expressions.
Symbol x1_key('x', 1);
Symbol x2_key('x', 2);
Double_ x1(x1_key), x2(x2_key);

}  // namespace constrained_example

}  // namespace gtdynamics
