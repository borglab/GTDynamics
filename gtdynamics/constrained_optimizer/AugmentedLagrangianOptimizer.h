/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  AugmentedLagrangianOptimizer.h
 * @brief Augmented Lagrangian method for constrained optimization.
 * @author Yetong Zhang, Frank Dellaert
 */

#pragma once

#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>
#include <gtdynamics/constraints/InequalityConstraint.h>

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using gtsam::Vector;

/// Parameters for Augmented Lagrangian method
struct AugmentedLagrangianParameters
    : public ConstrainedOptimizationParameters {
  using Base = ConstrainedOptimizationParameters;
  using shared_ptr = std::shared_ptr<AugmentedLagrangianParameters>;
  double initial_mu_e = 1.0; // initial penalty parameter
  double initial_mu_i = 1.0;
  double mu_e_increase_rate = 2.0;
  double mu_i_increase_rate = 2.0;
  double max_dual_step_size_e = 10; // maximum step size for dual ascent
  double max_dual_step_size_i = 10; // maximum step size for dual ascent
  double dual_step_size_factor_e = 1.0;
  double dual_step_size_factor_i = 0.1;
  double mu_increase_threshold = 0.25;
  std::vector<LevenbergMarquardtParams>
      iters_lm_params; // use different lm parameters for different iterations.

  AugmentedLagrangianParameters() : Base(20, LevenbergMarquardtParams()) {}

  AugmentedLagrangianParameters(const LevenbergMarquardtParams &_lm_params,
                                const size_t &_num_iterations = 20)
      : Base(_num_iterations, _lm_params) {}
};

/// Details for each iteration.
struct AugmentedLagrangianIterDetails : public ConstrainedOptIterDetails {
  double mu_e;                         // penalty parameter for e
  double mu_i;                         // penalty parameter for i
  std::vector<Vector> lambda_e;        // Lagrange multipliers for e
  std::vector<double> lambda_i;        // Lagrange multipliers for i
  std::vector<Values> lm_iters_values; // values after each lm iter
  std::vector<size_t> lm_inner_iters;  // lm inner iters in each iter
};
typedef std::vector<AugmentedLagrangianIterDetails>
    AugmentedLagrangianItersDetails;

/// Augmented Lagrangian method only considering equality constraints.
class AugmentedLagrangianOptimizer : public ConstrainedOptimizer {
protected:
  AugmentedLagrangianParameters::shared_ptr p_;
  std::shared_ptr<AugmentedLagrangianItersDetails> details_;

public:
  /// Constructor from parameters.
  AugmentedLagrangianOptimizer(
      AugmentedLagrangianParameters::shared_ptr p =
          std::make_shared<AugmentedLagrangianParameters>())
      : p_(p), details_(std::make_shared<AugmentedLagrangianItersDetails>()) {}

  /// Run optimization with equality constraints.
  Values optimize(const NonlinearFactorGraph &graph,
                  const EqualityConstraints &constraints,
                  const Values &initial_values) const override;

  /// Run optimization with both equality and inequality constraints.
  Values optimize(const NonlinearFactorGraph &graph,
                  const EqualityConstraints &e_constraints,
                  const InequalityConstraints &i_constraints,
                  const Values &init_values) const override;

  /// Return details of iterations.
  const AugmentedLagrangianItersDetails &details() const { return *details_; }

  /**
   * Lagrange dual function for equality constraints only:
   *   m(x) = 0.5 * ||f(x)||^2 - lambda_e * h(x) + 0.5 * mu_e * ||h(x)||^2
   * To express in nonlinear least squares form, it is rewritten as
   *   m(x) = 0.5 * ||f(x)||^2 + 0.5 * mu_e * ||h(x)- lambda_e/mu_e||^2 - c
   * where
   *   c = ||lambda||^2 / (2 * mu)
   * is a constant term.
   * @return: factor graph representing m(x) + c
   */
  static NonlinearFactorGraph
  LagrangeDualFunction(const NonlinearFactorGraph &graph,
                       const EqualityConstraints &e_constraints,
                       const double mu, const std::vector<Vector> &z);

  /**
   * Lagrange dual function for equality constraints and inequality constraints
   *   m(x) = 0.5 * ||f(x)||^2 - lambda_e * h(x) + 0.5 * mu_e * ||h(x)||^2
   *                           - lambda_i * g(x) + 0.5 * mu_i * ||g(x)_-||^2
   * To express in nonlinear least squares form, it is rewritten as
   *     m(x) + 0.5d * ||g(x)||^2
   *   = 0.5 * ||f(x)||^2 - lambda_e * h(x) + 0.5 * mu_e * ||h(x)||^2
   *     + 0.5d * ||g(x)||^2 - lambda_i * g(x) + 0.5 * mu_i * ||g(x)_-||^2
   *   = 0.5 * ||f(x)||^2
   *     + 0.5mu_e * ||h(x)- lambda_e/mu_e||^2
   *     + 0.5d * ||g(x)-lambda_i/mu_i||^2
   *     + 0.5mu_i * ||g(x)_-||^2
   *     - c
   * where
   *   c = ||lambda_e||^2 / (2 * mu_e) + ||lambda_i||^2 / (2 * d)
   * is a constant term.
   * Notice: a term (0.5 d * ||g(x)||^2) is added to incorporate Lagrange
   * multiplier terms in the nonlinear least squares form, with d very small.
   * @return: factor graph representing m(x) + 0.5d * ||g(x)||^2 + c
   */
  static NonlinearFactorGraph LagrangeDualFunction(
      const NonlinearFactorGraph &graph,
      const EqualityConstraints &e_constraints,
      const InequalityConstraints &i_constraints, const double mu_e,
      const double mu_i, const std::vector<Vector> &lambda_e,
      const std::vector<double> &lambda_i, const double d = 1e-6);

  /** Update penalty parameter and Lagrangian multipliers from unconstrained
   * optimization result. */
  void updateEParameters(const EqualityConstraints &constraints,
                         const Values &previous_values,
                         const Values &current_values, double &mu,
                         std::vector<Vector> &lambda) const;

  /** Update penalty parameter and Lagrangian multipliers from unconstrained
   * optimization result. */
  void updateIParameters(const InequalityConstraints &constraints,
                         const Values &previous_values,
                         const Values &current_values, double &mu,
                         std::vector<double> &lambda) const;

  /// Construct LM optimizer for the iteration.
  std::shared_ptr<LevenbergMarquardtOptimizer>
  CreateIterLMOptimizer(const NonlinearFactorGraph &graph, const Values &values,
                        const int i) const;

  AugmentedLagrangianIterDetails
  RetrieveIterDetails(std::shared_ptr<LevenbergMarquardtOptimizer> optimizer,
                      const Values &values) const;
};

} // namespace gtdynamics
