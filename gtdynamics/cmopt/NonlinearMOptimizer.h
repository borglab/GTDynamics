/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  NonlinearMOptimizer.h
 * @brief Manifold optimizer that internally calls nonlinear optimizer.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

#include <optional>
#include <variant>

namespace gtdynamics {

using gtsam::DoglegParams;
using gtsam::GaussNewtonParams;
using gtsam::LevenbergMarquardtParams;
using gtsam::NonlinearFactorGraph;
using gtsam::NonlinearOptimizer;
using gtsam::Values;

/**
 * Generic optimizer wrapper for transformed manifold problems.
 *
 * This optimizer relies on `ManifoldOptimizer` for transformation and then
 * delegates optimization to a selected GTSAM nonlinear optimizer
 * (Gauss-Newton, LM, or Dogleg) on manifold-valued variables.
 *
 * @see README.md#solvers
 * @see README.md#ccc-transformation
 */
class NonlinearMOptimizer : public ManifoldOptimizer {
 public:
  using shared_ptr = std::shared_ptr<const NonlinearMOptimizer>;
  typedef std::variant<GaussNewtonParams, LevenbergMarquardtParams,
                       DoglegParams>
      NonlinearOptParamsVariant;

 protected:
  NonlinearOptParamsVariant nopt_params_;

 public:
  /**
   * Construct from manifold and nonlinear optimizer parameters.
   * @param mopt_params Parameters for manifold construction/transformation.
   * @param nopt_params Parameters for the underlying nonlinear optimizer.
   * @param basis_key_func Optional basis-key selector.
   */
  NonlinearMOptimizer(const ManifoldOptimizerParameters& mopt_params,
                      const NonlinearOptParamsVariant& nopt_params,
                      std::optional<BasisKeyFunc> basis_key_func = {})
      : ManifoldOptimizer(mopt_params), nopt_params_(nopt_params) {}

  /// Virtual destructor.
  virtual ~NonlinearMOptimizer() {}

  /**
   * Run manifold optimization from a constrained problem definition.
   * @param graph Cost factor graph.
   * @param constraints Equality constraints.
   * @param initial_values Initial values for all variables.
   * @return Optimized base-variable values.
   */
  virtual Values optimize(const NonlinearFactorGraph& graph,
                          const EqualityConstraints& constraints,
                          const Values& initial_values) const override;

  /**
   * Optimize an already transformed manifold optimization problem.
   * @param mopt_problem Transformed manifold optimization problem.
   * @return Optimized base-variable values.
   */
  Values optimizeMOpt(const ManifoldOptProblem& mopt_problem) const;

  /**
   * Create the underlying nonlinear optimizer instance.
   * @param mopt_problem Transformed manifold optimization problem.
   * @return Nonlinear optimizer configured for transformed variables.
   */
  std::shared_ptr<NonlinearOptimizer> constructNonlinearOptimizer(
      const ManifoldOptProblem& mopt_problem) const;

 protected:
};

}  // namespace gtdynamics
