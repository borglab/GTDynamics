/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyOptimizer.h
 * @brief Penalty method optimizer for constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include <gtdynamics/constrained_optimizer/ConstrainedOptimizer.h>

namespace gtdynamics {

using gtsam::LevenbergMarquardtParams;
using gtsam::LevenbergMarquardtOptimizer;
using gtsam::NonlinearFactorGraph;
using gtsam::Values;

/// Parameters for penalty method
struct PenaltyParameters : public ConstrainedOptimizationParameters {
  using Base = ConstrainedOptimizationParameters;
  using shared_ptr = std::shared_ptr<PenaltyParameters>;
  double initial_mu;       // initial penalty parameter
  double mu_increase_rate; // increase rate of penalty parameter
  std::vector<LevenbergMarquardtParams>
      iters_lm_params; // use different lm parameters for different iterations.

  /** Constructor. */
  PenaltyParameters()
      : Base(15, LevenbergMarquardtParams()), initial_mu(1.0),
        mu_increase_rate(2.0) {}

  PenaltyParameters(const LevenbergMarquardtParams &_lm_params,
                    const size_t &_num_iterations = 15,
                    const double &_initial_mu = 1.0,
                    const double &_mu_increase_rate = 2.0)
      : Base(_num_iterations, _lm_params), initial_mu(_initial_mu),
        mu_increase_rate(_mu_increase_rate) {}
};

/// Details for each iteration.
struct PenaltyIterDetails : public ConstrainedOptIterDetails {
  double mu;
  std::vector<Values> lm_iters_values; // values after each lm iter
  std::vector<size_t> lm_inner_iters;  // lm inner iters in each iter
};
typedef std::vector<PenaltyIterDetails> PenaltyItersDetails;

/// Penalty method only considering equality constraints.
class PenaltyOptimizer : public ConstrainedOptimizer {
protected:
  PenaltyParameters::shared_ptr p_;
  std::shared_ptr<PenaltyItersDetails> details_;

public:
  /// Default constructor.
  PenaltyOptimizer(
      PenaltyParameters::shared_ptr p = std::make_shared<PenaltyParameters>())
      : p_(p), details_(std::make_shared<PenaltyItersDetails>()) {}

  /// Run optimization with equality constraints only.
  Values optimize(const NonlinearFactorGraph &cost,
                  const EqualityConstraints &constraints,
                  const Values &initial_values) const override;

  /// Run optimization with equality and inequality constraints.
  Values optimize(const NonlinearFactorGraph &cost,
                  const EqualityConstraints &e_constraints,
                  const InequalityConstraints &i_constraints,
                  const Values &initial_values) const override;

  /// Return details of iterations.
  const PenaltyItersDetails &details() const { return *details_; }

  /// Merit function in the form  m(x) = f(x) + 0.5 mu * ||h(x)||^2
  static NonlinearFactorGraph
  MeritFunction(const NonlinearFactorGraph &cost,
                const EqualityConstraints &constraints, const double mu);

  /// Merit function in the form
  ///  m(x) = f(x) + 0.5 mu_e * ||h(x)||^2 + 0.5 mu_i ||g(x)_-||^2
  static NonlinearFactorGraph
  MeritFunction(const NonlinearFactorGraph &cost,
                const EqualityConstraints &e_constraints,
                const InequalityConstraints &i_constraints, const double mu_e,
                const double mu_i);

  /// Construct LM optimizer for the iteration.
  std::shared_ptr<LevenbergMarquardtOptimizer>
  CreateIterLMOptimizer(const NonlinearFactorGraph &graph, const Values &values,
                        const int i) const;

  PenaltyIterDetails
  RetrieveIterDetails(std::shared_ptr<LevenbergMarquardtOptimizer> optimizer,
                      const Values &values) const;
};

} // namespace gtdynamics
