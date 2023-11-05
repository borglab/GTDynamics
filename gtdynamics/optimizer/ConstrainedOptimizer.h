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

#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtdynamics {

/// Constrained optimization parameters shared between all solvers.
struct ConstrainedOptimizationParameters {
  gtsam::LevenbergMarquardtParams lm_parameters;  // LM parameters
  bool verbose = false;
  
  /// Constructor.
  ConstrainedOptimizationParameters() {}

  /// Constructor with LM parameters.
  ConstrainedOptimizationParameters(
      const gtsam::LevenbergMarquardtParams& _lm_parameters)
      : lm_parameters(_lm_parameters) {}
};

/// Intermediate results for constrained optimization process.
struct ConstrainedOptResult {
  std::vector<gtsam::Values>
      intermediate_values;        // values after each inner loop
  std::vector<gtsam::VectorValues> tangent_vectors;
  std::vector<int> num_iters;     // number of LM iterations for each inner loop
  std::vector<int> num_inner_iters;
  std::vector<double> mu_values;  // penalty parameter for each inner loop
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
   * @param intermediate_result (optional) intermediate results during
   * optimization.
   * @return Values The result of the constrained optimization.
   */
  virtual gtsam::Values optimize(
      const gtsam::NonlinearFactorGraph& graph,
      const EqualityConstraints& constraints,
      const gtsam::Values& initial_values,
      ConstrainedOptResult* intermediate_result = nullptr) const = 0;
};

/** Equality-constrained optimization problem, in the form of 
 * argmin_x ||f(X)||^2
 * s.t.     h(X) = 0
 * where X represents the variables, ||f(X)||^2 represents the quadratic cost
 * functions, h(X)=0 represents the constraints.
*/
struct EqConsOptProblem {
  typedef std::function<void(const gtsam::Values &values)> EvalFunc;

  gtsam::NonlinearFactorGraph costs_;           // cost function, ||f(X)||^2
  gtdynamics::EqualityConstraints constraints_; // equality constraints. h(X)=0
  gtsam::Values values_;                        // values of all variables, X
  EvalFunc eval_func;
  /// Constructor.
  EqConsOptProblem(const gtsam::NonlinearFactorGraph& costs,
                   const gtdynamics::EqualityConstraints& constraints,
                   const gtsam::Values& values)
      : costs_(costs), constraints_(constraints), values_(values) {}

  const gtsam::NonlinearFactorGraph& costs() const {return costs_; }
  const gtdynamics::EqualityConstraints& constraints() const {return constraints_; }
  const gtsam::Values& initValues() const {return values_; }

  /// Evaluate the dimension of costs.
  size_t costsDimension() const {
    size_t costs_dim = 0;
    for (const auto &factor : costs_) {
      costs_dim += factor->dim();
    }
    return costs_dim;
  }

  /// Evaluate the dimension of constriants.
  size_t constraintsDimension() const {
    size_t constraints_dim = 0;
    for (const auto &constraint : constraints_) {
      constraints_dim += constraint->dim();
    }
    return constraints_dim;
  }

  /// Evaluate the dimension of variables.
  size_t valuesDimension() const {
    return values_.dim();
  }

  /// Evaluate the cost.
  double evaluateCost(const gtsam::Values& values) const {
    return costs_.error(values);
  }

  /// Evaluate the constraint violation (as L2 norm).
  double evaluateConstraintViolationL2Norm(const gtsam::Values& values) const {
    double violation = 0;
    for (const auto& constraint: constraints_) {
      violation += pow(constraint->toleranceScaledViolation(values).norm(), 2);
    }
    return sqrt(violation);
  }

  /// Return a graph of merit factors of constraints.
  virtual gtsam::NonlinearFactorGraph constraintsGraph(double mu=1.0) const {
    gtsam::NonlinearFactorGraph constraints_graph;
    for (const auto& constraint: constraints_) {
      constraints_graph.add(constraint->createFactor(mu));
    }
    return constraints_graph;
  }
};

}  // namespace gtdynamics
