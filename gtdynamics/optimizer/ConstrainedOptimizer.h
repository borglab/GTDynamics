/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConstrainedOptimizer.h
 * @brief Base class constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "gtdynamics/optimizer/EqualityConstraint.h"

namespace gtdynamics {

/// Constrained optimization parameters shared between all solvers.
struct ConstrainedOptimizationParameters {
  gtsam::LevenbergMarquardtParams lm_parameters;  // LM parameters

  /// Constructor.
  ConstrainedOptimizationParameters() {}
};

/// Base class for constrained optimizer.
class ConstrainedOptimizer {

 public:
  /**
   * @brief Constructor.
   */
  ConstrainedOptimizer() {}

  /**
   * @brief Solve constrained optimization problem with optimizer settings.
   *
   * @param graph A Nonlinear factor graph representing cost.
   * @param cosntraints All the constraints.
   * @param initial_values Initial values for all variables.
   * @return Values The result of the constrained optimization.
   */
  virtual gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                                 const EqualityConstraints& constraints,
                                 const gtsam::Values& initial_values) const = 0;
};
}  // namespace gtdynamics
