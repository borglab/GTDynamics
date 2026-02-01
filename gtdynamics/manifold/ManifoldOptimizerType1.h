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

using gtdynamics::EConsOptProblem;

namespace gtsam {

/// Manifold optimization problem.
struct ManifoldOptProblem {
  NonlinearFactorGraph graph_;  // cost function on constraint manifolds
  std::vector<ConnectedComponent::shared_ptr>
      components_;  // All the constraint-connected components
  Values
      values_;  // values for constraint manifolds and unconstrained variables
  Values fixed_manifolds_;     // fully constrained variables
  KeySet unconstrained_keys_;  // keys for unconstrained variables
  KeySet manifold_keys_;       // keys for constraint manifold variables

  /** Dimension of the manifold optimization problem, as factor dimension x
   * variable dimension. */
  std::pair<size_t, size_t> problemDimension() const;

  /// Customizable print function.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
};

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
  virtual gtsam::Values optimizeWithIntermediate(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::NonlinearEqualityConstraints& constraints,
      const gtsam::Values& initial_values,
      gtdynamics::ConstrainedOptResult* intermediate_result =
          nullptr) const;

  /// Optimization given manifold optimization problem.
  gtsam::Values optimizeWithIntermediate(
      const ManifoldOptProblem& mopt_problem,
      gtdynamics::ConstrainedOptResult* intermediate_result = nullptr) const;

  /// Initialize the manifold optimization problem.
  ManifoldOptProblem initializeMoptProblem(
      const gtsam::NonlinearFactorGraph& costs,
      const gtsam::NonlinearEqualityConstraints& constraints,
      const gtsam::Values& init_values) const;

  /// Create the underlying nonlinear optimizer for manifold optimization.
  std::shared_ptr<NonlinearOptimizer> constructNonlinearOptimizer(
      const ManifoldOptProblem& mopt_problem) const;

  /// Construct values of original variables.
  Values baseValues(const ManifoldOptProblem& mopt_problem,
                    const Values& nopt_values) const;

 protected:
  /** Create values for the manifold optimization problem by (1) create
   * constraint manifolds for constraint-connected components; (2) identify if
   * the constraint manifold is fully constrained; (3) collect unconstrained
   * variables.
   */
  void constructMoptValues(const EConsOptProblem& ecopt_problem,
                           ManifoldOptProblem& mopt_problem) const;

  /// Create initial values for the constraint manifold variables.
  void constructManifoldValues(const EConsOptProblem& ecopt_problem,
                               ManifoldOptProblem& mopt_problem) const;

  /// Collect values for unconstrained variables.
  void constructUnconstrainedValues(const EConsOptProblem& ecopt_problem,
                                    ManifoldOptProblem& mopt_problem) const;

  /** Create a factor graph of cost function with the constraint manifold
   * variables. */
  void constructMoptGraph(const EConsOptProblem& ecopt_problem,
                          ManifoldOptProblem& mopt_problem) const;

  /** Transform an equality-constrained optimization problem into a manifold
   * optimization problem by creating constraint manifolds. */
  ManifoldOptProblem problemTransform(
      const EConsOptProblem& ecopt_problem) const;
};

}  // namespace gtsam
