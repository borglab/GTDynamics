#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>

namespace gtsam {

typedef std::function<double(const double &x, OptionalJacobian<1, 1> H)>
    DoublePenaltyFunc;

/** Smooth barrier function for i-constraint x >= min
 * Function f(x) = 0                 | min <= x
 *                 k * (min-x)^2     | min - b < x < min
 *                 min - x - offset  | x < min - b
 */
DoublePenaltyFunc SmoothBarrierFunction(const double &min_val, const double &b);

// /** Smooth barrier function.
//  * Function f(x) = x - max - offset  | x > max + b
//  *                 k * (x-max)^2     | max < x < max + b
//  *                 0                 | min < x < max
//  *                 k * (min-x)^2     | min - b < x < min
//  *                 min - x - offset  | x < min - b
//  */
// DoublePenaltyFunc SmoothBarrierFunction(const double &min_val,
//                                         const double &max_val, const double &b);

/** Smooth barrier factor corresponding to the constraint x>=min. */
NoiseModelFactor::shared_ptr SmoothBarrierFactor(const Key &key,
                                                 const double &min_val,
                                                 const double &b,
                                                 const SharedNoiseModel &model);

/** Smooth barrier factor corresponding to the constraint max>=x>=min. */
NoiseModelFactor::shared_ptr SmoothBarrierFactor(const Key &key,
                                                 const double &min_val,
                                                 const double &max_val,
                                                 const double &b,
                                                 const SharedNoiseModel &model);

} // namespace gtsam
