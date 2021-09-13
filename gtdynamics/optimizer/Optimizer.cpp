/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Optimizer.cpp
 * @brief Optimization routines.
 * @author: Frank Dellaert
 */

#include <gtdynamics/optimizer/Optimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;

Values Optimizer::optimize(const NonlinearFactorGraph& graph,
                           const Values& initial_values) const {
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values,
                                               p_->lm_parameters);
  const Values result = optimizer.optimize();
  return result;
}
}  // namespace gtdynamics
