/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Optimizer.h
 * @brief Base class for Kinematics class, etc....
 * @author: Frank Dellaert
 */

#pragma once

#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

// Forward declarations.
namespace gtsam {
class NonlinearFactorGraph;
class Values;
}  // namespace gtsam

namespace gtdynamics {

/// Optimization parameters shared between all solvers
struct OptimizationParameters {
  enum Method { SOFT_CONSTRAINTS = 0, PENALTY = 1, AUGMENTED_LAGRANGIAN = 2 };

  Method method = Method::SOFT_CONSTRAINTS;       // optimization method
  gtsam::LevenbergMarquardtParams lm_parameters;  // LM parameters
  OptimizationParameters() {
    lm_parameters.setlambdaInitial(1e7);
    lm_parameters.setAbsoluteErrorTol(1e-3);
  }
};

/// Base class for GTDynamics optimizer hierarchy.
class Optimizer {
 protected:
  const OptimizationParameters p_;

 public:
  /**
   * @fn Constructor.
   */
  Optimizer(const OptimizationParameters& parameters = OptimizationParameters())
      : p_(parameters) {}

  /**
   * @brief optimize graph using optimizer settings.
   *
   * @param graph a Nonlinear factor graph built by derived class
   * @param initial_values Initial values for all variables.
   * @return Values The result of the optimization.
   */
  // TODO(yetong): remove after discussing with team
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& initial_values) const;

  /**
   * @brief optimize with constraints using optimizer settings.
   *
   * @param graph a Nonlinear factor graph built by derived class
   * @param initial_values Initial values for all variables.
   * @return Values The result of the optimization.
   */
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                         const EqualityConstraints& constraints,
                         const gtsam::Values& initial_values) const;
};
}  // namespace gtdynamics
