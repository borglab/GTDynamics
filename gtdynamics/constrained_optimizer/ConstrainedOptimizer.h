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

#include <gtdynamics/constrained_optimizer/ConstrainedOptProblem.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

/// Constrained optimization parameters shared between all solvers.
struct ConstrainedOptimizationParameters {
  using shared_ptr = std::shared_ptr<ConstrainedOptimizationParameters>;
  size_t num_iterations;              // number of iterations
  LevenbergMarquardtParams lm_params; // LM parameters
  bool verbose = false;
  bool store_iter_details = false; // Store detailed info of each iter
  bool store_lm_details = false;   // Store iterations by LM

  /// Constructor with LM parameters.
  ConstrainedOptimizationParameters(
      const size_t _num_iterations = 10,
      const LevenbergMarquardtParams &_lm_params = LevenbergMarquardtParams())
      : num_iterations(_num_iterations), lm_params(_lm_params) {}
};

/// Results after each iteration during the constrained optimization progress.
struct ConstrainedOptIterDetails {
  Values values;    // values after each inner loop
  int num_lm_iters; // number of LM iterations for each inner loop
  int num_lm_inner_iters;
};

/// Results from constrained optimization runs.
struct ConstrainedOptResult {
  std::vector<size_t> num_iters;
};

/// Base class for constrained optimizer.
class ConstrainedOptimizer {
public:
  /**
   * @brief Constructor.
   */
  ConstrainedOptimizer() {}

  /**
   * @brief Solve constrained optimization problem with equality constraints
   * only.
   *
   * @param graph A Nonlinear factor graph representing cost.
   * @param constraints All the constraints.
   * @param initial_values Initial values for all variables.
   * @return Values The result of the constrained optimization.
   */
  virtual Values optimize(const NonlinearFactorGraph &graph,
                          const gtsam::NonlinearEqualityConstraints &constraints,
                          const Values &initial_values) const {
    throw std::runtime_error(
        "Equality constrained optimization not implemented");
  }

  /**
   * @brief Solve constrained optimization problem with both equality and
   * inequality constraints.
   *
   * @param graph A Nonlinear factor graph representing cost.
   * @param e_constraints All the nonlinear equality constraints.
   * @param i_constraints All the nonlinear inequality constraints.
   * @param initial_values Initial values for all variables.
   * @return Values The result of the constrained optimization.
   */
  virtual Values optimize(const NonlinearFactorGraph &graph,
                          const gtsam::NonlinearEqualityConstraints &e_constraints,
                          const gtsam::NonlinearInequalityConstraints &i_constraints,
                          const Values &initial_values) const {
    throw std::runtime_error(
        "Inequality constrained optimization not implemented");
  }

  /// Solve a constrained optimization problem with equality constraints only.
  Values optimizeE(const EConsOptProblem &problem) const {
    return optimize(problem.costs(), problem.constraints(),
                    problem.initValues());
  }

  /// Solve a constrained optimization problem with i and e constraints.
  Values optimizeIE(const IEConsOptProblem &problem) const {
    return optimize(problem.costs(), problem.eConstraints(),
                    problem.iConstraints(), problem.initValues());
  }

  /// Run optimization with auxiliary variables that transforms inequality
  /// constraints into equality constraints of the form g(x)-z^2=0.
  Values optimizeIEAuxiliary(const IEConsOptProblem &problem) const {
    return optimizeE(problem.auxiliaryProblem());
  }

protected:
  std::shared_ptr<LevenbergMarquardtOptimizer>
  CreateLMOptimizer(const NonlinearFactorGraph &graph, const Values &values,
                    const bool store_lm_details,
                    const LevenbergMarquardtParams &lm_params) const;

  /// For each LM iteration, retrieve the values and number of inner iterations.
  std::pair<std::vector<Values>, std::vector<size_t>> RetrieveLMItersValues(
      std::shared_ptr<LevenbergMarquardtOptimizer> optimizer) const;
};

} // namespace gtdynamics
