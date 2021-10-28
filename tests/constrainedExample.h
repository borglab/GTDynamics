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

/** First cost function. */
double cost1(const double &x1, const double &x2,
             gtsam::OptionalJacobian<1, 1> H_x1 = boost::none,
             gtsam::OptionalJacobian<1, 1> H_x2 = boost::none) {
  double result = x1 + exp(-x2);
  if (H_x1) H_x1->setConstant(1.0);
  if (H_x2) H_x2->setConstant(-exp(-x2));
  return result;
}

/** Second cost function. */
double cost2(const double &x1, const double &x2,
             gtsam::OptionalJacobian<1, 1> H_x1 = boost::none,
             gtsam::OptionalJacobian<1, 1> H_x2 = boost::none) {
  double result = x1 * x1 + 2 * x2 + 1;
  if (H_x1) H_x1->setConstant(2 * x1);
  if (H_x2) H_x2->setConstant(2.0);
  return result;
}

/** Constraint function g(x). */
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

/// A 2-dimensional function that adds up 2 Vector2.
Vector2_ x1_vec_expr(x1_key);
Vector2_ x2_vec_expr(x2_key);
Vector2_ constraint_sum_vector2_expr = x1_vec_expr + x2_vec_expr;

}  // namespace constrained_example

}  // namespace gtdynamics
