/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file BasisRetractor.h
 * @brief BasisRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmopt/Retractor.h>

namespace gtdynamics {

/**
 * Retractor that fixes basis variables and solves for the remaining variables.
 *
 * This matches the basis-variable retraction idea in the tutorial and paper.
 *
 * @see README.md#retraction
 * @see README.md#tangent-basis
 */
class BasisRetractor : public Retractor {
protected:
  NonlinearFactorGraph merit_graph_;
  KeyVector basis_keys_;
  MutableLMOptimizer optimizer_;
  std::vector<std::shared_ptr<ConstVarFactor>> factors_with_fixed_vars_;

public:
  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Basis keys held fixed during inner solve.
   */
  BasisRetractor(const NonlinearEqualityConstraints::shared_ptr &constraints,
                 const RetractionParams::shared_ptr &params,
                 const KeyVector &basis_keys);

  virtual ~BasisRetractor() {}

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

protected:
  /**
   * Build reduced retraction graph with basis keys treated as fixed.
   * @param constraints Equality constraints for the component.
   * @param basis_keys Basis keys held fixed during inner solve.
   */
  void
  constructGraph(const NonlinearEqualityConstraints::shared_ptr &constraints,
                 const KeyVector &basis_keys);
};

/**
 * Factory for `BasisRetractor`.
 *
 * @see README.md#retraction
 * @see README.md#tangent-basis
 */
class BasisRetractorCreator : public RetractorCreator {
public:
  BasisKeyFunction basisKeyFunction;

public:
  BasisRetractorCreator(RetractionParams::shared_ptr params =
                            std::make_shared<RetractionParams>())
      : RetractorCreator(params) {}

  /**
   * Constructor with custom basis-key selector.
   * @param basisKeyFunction Callback selecting basis keys.
   * @param params Retraction parameters.
   */
  BasisRetractorCreator(BasisKeyFunction basisKeyFunction,
                        RetractionParams::shared_ptr params =
                            std::make_shared<RetractionParams>())
      : RetractorCreator(params), basisKeyFunction(basisKeyFunction) {}

  virtual ~BasisRetractorCreator() {}

  Retractor::shared_ptr create(const NonlinearEqualityConstraints::shared_ptr
                                   constraints) const override {
    KeyVector basis_keys = basisKeyFunction(constraints->keyVector());
    return std::make_shared<BasisRetractor>(constraints, params_, basis_keys);
  }
};

} // namespace gtdynamics
