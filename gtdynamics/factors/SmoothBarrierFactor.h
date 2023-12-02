#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

typedef std::function<double(const double &x, OptionalJacobian<1, 1> H)>
    DoublePenaltyFunc;

/** Smooth barrier function.
 * Function f(x) = x - max - offset  | x > max + b
 *                 k * (x-max)^2     | max < x < max + b
 *                 0                 | min < x < max
 *                 k * (min-x)^2     | min - b < x < min
 *                 min - x - offset  | x < min - b
 */

inline DoublePenaltyFunc SmoothBarrierFunction(const double &min_val,
                                               const double &max_val,
                                               const double &b) {

  double k = 0.5 / b;
  double offset = 0.5 * b;
  auto func = [=](const double &x, OptionalJacobian<1, 1> H = {}) -> double {
    int sign;
    double error;
    if (x < min_val) {
      error = min_val - x;
      sign = -1;
    } else if (x > max_val) {
      error = x - max_val;
      sign = 1;
    } else {
      if (H) {
        H->setConstant(0);
      }
      return 0;
    }
    if (error < b) {
      if (H) {
        H->setConstant(2 * sign * k * error);
      }
      return k * error * error;
    } else {
      if (H) {
        H->setConstant(sign);
      }
      return error - offset;
    }
  };
  return func;
}

inline NoiseModelFactor::shared_ptr
SmoothBarrierFactor(const Key &key, const double &min_val,
                    const double &max_val, const double &b,
                    const SharedNoiseModel &model) {
  Double_ x_expr(key);
  auto barrier_function = SmoothBarrierFunction(min_val, max_val, b);
  Double_ error_expr(barrier_function, x_expr);
  return std::make_shared<ExpressionFactor<double>>(model, 0.0, error_expr);
}

} // namespace gtsam
