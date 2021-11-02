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

double exp_func(const double& x,
                gtsam::OptionalJacobian<1, 1> H1 = boost::none) {
  double result = exp(x);
  if (H1) H1->setConstant(result);
  return result;
}

class ExpExpression : public Double_ {
 public:
  explicit ExpExpression(const Double_& e) : Double_(exp_func, e) {}
};

Symbol x1_key('x', 1);
Symbol x2_key('x', 2);

Double_ x1_expr(x1_key);
Double_ x2_expr(x2_key);
Double_ cost1_expr = x1_expr + ExpExpression(Double_(0.0) - x2_expr);
Double_ cost2_expr = x1_expr * x1_expr + 2.0 * x2_expr + Double_(1.0);
Double_ constraint1_expr =
    x1_expr + x1_expr * x1_expr * x1_expr + x2_expr + x2_expr * x2_expr;

/// A 2-dimensional function that adds up 2 Vector2.
Vector2_ x1_vec_expr(x1_key);
Vector2_ x2_vec_expr(x2_key);
Vector2_ constraint_sum_vector2_expr = x1_vec_expr + x2_vec_expr;

}  // namespace constrained_example

}  // namespace gtdynamics
