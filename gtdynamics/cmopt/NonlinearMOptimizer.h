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

/** Manifold Optimizer that replace each constraint-connected component with a
 * constraint manifold variable */
class NonlinearMOptimizer : public ManifoldOptimizer {
 public:
  using shared_ptr = std::shared_ptr<const NonlinearMOptimizer>;
  typedef std::variant<GaussNewtonParams, LevenbergMarquardtParams,
                       DoglegParams>
      NonlinearOptParamsVariant;

 protected:
  NonlinearOptParamsVariant nopt_params_;

 public:
  /// Construct from parameters.
  NonlinearMOptimizer(const ManifoldOptimizerParameters& mopt_params,
                         const NonlinearOptParamsVariant& nopt_params,
                         std::optional<BasisKeyFunc> basis_key_func = {})
      : ManifoldOptimizer(mopt_params), nopt_params_(nopt_params) {}

  /// Virtual destructor.
  virtual ~NonlinearMOptimizer() {}

  /** Run manifold optimization by substituting the constrained variables with
   * the constraint manifold variables. */
  virtual Values optimize(const NonlinearFactorGraph& graph,
                          const EqualityConstraints& constraints,
                          const Values& initial_values) const override;

  /// Optimization given manifold optimization problem.
  Values optimizeMOpt(const ManifoldOptProblem& mopt_problem) const;

  /// Create the underlying nonlinear optimizer for manifold optimization.
  std::shared_ptr<NonlinearOptimizer> constructNonlinearOptimizer(
      const ManifoldOptProblem& mopt_problem) const;

 protected:

};

}  // namespace gtdynamics
