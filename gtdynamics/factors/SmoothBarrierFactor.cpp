#include <gtdynamics/factors/SmoothBarrierFactor.h>

namespace gtsam {

/* ************************************************************************* */
DoublePenaltyFunc SmoothBarrierFunction(const double &min_val,
                                        const double &b) {
  double k = 0.5 / b;
  double offset = 0.5 * b;
  auto func = [=](const double &x, OptionalJacobian<1, 1> H = {}) -> double {
    if (x >= min_val) {
      if (H) {
        H->setConstant(0);
      }
      return 0;
    }
    double error = min_val - x;
    if (error < b) {
      if (H) {
        H->setConstant(-2 * k * error);
      }
      return k * error * error;
    } else {
      if (H) {
        H->setConstant(-1);
      }
      return error - offset;
    }
  };
  return func;
}

// /* *************************************************************************
// */ DoublePenaltyFunc SmoothBarrierFunction(const double &min_val,
//                                         const double &max_val,
//                                         const double &b) {

//   double k = 0.5 / b;
//   double offset = 0.5 * b;
//   auto func = [=](const double &x, OptionalJacobian<1, 1> H = {}) -> double {
//     int sign;
//     double error;
//     if (x < min_val) {
//       error = min_val - x;
//       sign = -1;
//     } else if (x > max_val) {
//       error = x - max_val;
//       sign = 1;
//     } else {
//       if (H) {
//         H->setConstant(0);
//       }
//       return 0;
//     }
//     if (error < b) {
//       if (H) {
//         H->setConstant(2 * sign * k * error);
//       }
//       return k * error * error;
//     } else {
//       if (H) {
//         H->setConstant(sign);
//       }
//       return error - offset;
//     }
//   };
//   return func;
// }

/* ************************************************************************* */
NoiseModelFactor::shared_ptr
SmoothBarrierFactor(const Key &key, const double &min_val, const double &b,
                    const SharedNoiseModel &model) {
  Double_ x_expr(key);
  auto barrier_function = SmoothBarrierFunction(min_val, b);
  Double_ error_expr(barrier_function, x_expr);
  return std::make_shared<ExpressionFactor<double>>(model, 0.0, error_expr);
}

/* ************************************************************************* */
NoiseModelFactor::shared_ptr
SmoothBarrierFactor(const Key &key, const double &min_val,
                    const double &max_val, const double &b,
                    const SharedNoiseModel &model) {
  Double_ x_expr(key);
  auto barrier_function_min = SmoothBarrierFunction(min_val, b);
  auto barrier_function_negmax = SmoothBarrierFunction(-max_val, b);

  Double_ error_min_expr(barrier_function_min, x_expr);
  Double_ error_max_expr(barrier_function_negmax, -1 * x_expr);
  Double_ error_expr = error_min_expr + error_max_expr;
  return std::make_shared<ExpressionFactor<double>>(model, 0.0, error_expr);
}

} // namespace gtsam