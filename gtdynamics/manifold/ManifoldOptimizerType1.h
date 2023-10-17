/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizerType1.h
 * @brief Manifold optimizer that use a constraint manifold to represent each
 * constraint-connected-component.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

#include <variant>

using gtdynamics::EqConsOptProblem;

namespace gtsam {


/** Manifold Optimizer that replace each constraint-connected component with a
 * constraint manifold variable */
class ManifoldOptimizerType1 : public ManifoldOptimizer {
 public:
  using shared_ptr = std::shared_ptr<const ManifoldOptimizerType1>;
  typedef std::variant<GaussNewtonParams, LevenbergMarquardtParams,
                       DoglegParams>
      NonlinearOptParamsVariant;

 protected:
  NonlinearOptParamsVariant nopt_params_;

 public:
  /// Construct from parameters.
  ManifoldOptimizerType1(const ManifoldOptimizerParameters& mopt_params,
                         const NonlinearOptParamsVariant& nopt_params,
                         std::optional<BasisKeyFunc> basis_key_func = {})
      : ManifoldOptimizer(mopt_params), nopt_params_(nopt_params) {}

  /// Virtual destructor.
  virtual ~ManifoldOptimizerType1() {}

  /** Run manifold optimization by substituting the constrained variables with
   * the constraint manifold variables. */
  virtual gtsam::Values optimize(
      const gtsam::NonlinearFactorGraph& graph,
      const gtdynamics::EqualityConstraints& constraints,
      const gtsam::Values& initial_values,
      gtdynamics::ConstrainedOptResult* intermediate_result =
          nullptr) const override;

  /// Optimization given manifold optimization problem.
  gtsam::Values optimize(
      const ManifoldOptProblem& mopt_problem,
      gtdynamics::ConstrainedOptResult* intermediate_result = nullptr) const;

  /// Create the underlying nonlinear optimizer for manifold optimization.
  std::shared_ptr<NonlinearOptimizer> constructNonlinearOptimizer(
      const ManifoldOptProblem& mopt_problem) const;

 protected:

};

}  // namespace gtsam
