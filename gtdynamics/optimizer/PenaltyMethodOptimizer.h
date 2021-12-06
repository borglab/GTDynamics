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
  double initial_mu;
  double mu_increase_rate;

  /** Constructor. */
  PenaltyMethodParameters()
      : Base(gtsam::LevenbergMarquardtParams()),
        num_iterations(15),
        initial_mu(1.0),
        mu_increase_rate(2.0) {}

  PenaltyMethodParameters(const gtsam::LevenbergMarquardtParams& _lm_parameters)
      : Base(_lm_parameters),
        num_iterations(15),
        initial_mu(1.0),
        mu_increase_rate(2.0) {}
};

/// Penalty method only considering equality constraints.
class PenaltyMethodOptimizer : public ConstrainedOptimizer {
 protected:
  boost::shared_ptr<const PenaltyMethodParameters> p_;

 public:
  /** Default constructor. */
  PenaltyMethodOptimizer()
      : p_(boost::make_shared<const PenaltyMethodParameters>()) {}

  /**
   * Construct from parameters.
   */
  PenaltyMethodOptimizer(
      const boost::shared_ptr<const PenaltyMethodParameters>& parameters)
      : p_(parameters) {}

  /// Run optimization.
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                         const EqualityConstraints& constraints,
                         const gtsam::Values& initial_values) const override;
};

}  // namespace gtdynamics
