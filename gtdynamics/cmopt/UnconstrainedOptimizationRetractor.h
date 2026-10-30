/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file UnconstrainedOptimizationRetractor.h
 * @brief UnconstrainedOptimizationRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmopt/Retractor.h>

namespace gtdynamics {

/**
 * Retractor using unconstrained optimization on the penalty graph.
 *
 * This corresponds to approximate metric projection behavior used in the
 * tutorial.
 *
 * @see README.md#retraction
 */
class UnconstrainedOptimizationRetractor : public Retractor {
protected:
  MutableLMOptimizer optimizer_;

public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   */
  UnconstrainedOptimizationRetractor(
      const NonlinearEqualityConstraints::shared_ptr &constraints,
      const RetractionParams::shared_ptr &params =
          std::make_shared<RetractionParams>());

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

  /// Retraction Inplace
  Values retractConstraints(Values &&values) override;
};

/**
 * Factory for `UnconstrainedOptimizationRetractor`.
 *
 * @see README.md#retraction
 */
class UnconstrainedOptimizationRetractorCreator : public RetractorCreator {
public:
  UnconstrainedOptimizationRetractorCreator(
      RetractionParams::shared_ptr params =
          std::make_shared<RetractionParams>())
      : RetractorCreator(params) {}

  virtual ~UnconstrainedOptimizationRetractorCreator() {}

  Retractor::shared_ptr create(const NonlinearEqualityConstraints::shared_ptr
                                   constraints) const override {
    return std::make_shared<UnconstrainedOptimizationRetractor>(constraints,
                                                                params_);
  }
};

} // namespace gtdynamics
