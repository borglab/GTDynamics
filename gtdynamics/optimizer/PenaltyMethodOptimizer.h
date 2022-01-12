/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PenaltyMethodOptimizer.h
 * @brief Penalty method optimizer for constrained optimization.
 * @author Yetong Zhang
 */

#pragma once

#include "gtdynamics/optimizer/ConstrainedOptimizer.h"

namespace gtdynamics {

/// Parameters for penalty method
struct PenaltyMethodParameters : public ConstrainedOptimizationParameters {
  using Base = ConstrainedOptimizationParameters;
  size_t num_iterations;
  double initial_mu;        // initial penalty parameter
  double mu_increase_rate;  // increase rate of penalty parameter

  /** Constructor. */
  PenaltyMethodParameters()
      : Base(gtsam::LevenbergMarquardtParams()),
        num_iterations(15),
        initial_mu(1.0),
        mu_increase_rate(2.0) {}

  PenaltyMethodParameters(const gtsam::LevenbergMarquardtParams& _lm_parameters,
                          const size_t& _num_iterations = 15,
                          const double& _initial_mu = 1.0,
                          const double& _mu_increase_rate = 2.0)
      : Base(_lm_parameters),
        num_iterations(_num_iterations),
        initial_mu(_initial_mu),
        mu_increase_rate(_mu_increase_rate) {}
};

/// Penalty method only considering equality constraints.
class PenaltyMethodOptimizer : public ConstrainedOptimizer {
 protected:
  const PenaltyMethodParameters p_;

 public:
  /** Default constructor. */
  PenaltyMethodOptimizer() : p_(PenaltyMethodParameters()) {}

  /**
   * Construct from parameters.
   */
  PenaltyMethodOptimizer(const PenaltyMethodParameters& parameters)
      : p_(parameters) {}

  /// Run optimization.
  gtsam::Values optimize(
      const gtsam::NonlinearFactorGraph& graph,
      const EqualityConstraints& constraints,
      const gtsam::Values& initial_values,
      ConstrainedOptResult* intermediate_result = nullptr) const override;
};

}  // namespace gtdynamics
