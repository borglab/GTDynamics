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
  using Retractor::retract;
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<BasisRetractor>(*this);
  }

  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Basis keys held fixed during inner solve.
   */
  BasisRetractor(const NonlinearEqualityConstraints::shared_ptr &constraints,
                 const RetractionParams::shared_ptr &params,
                 const KeyVector &basis_keys);

  /// A workspace copy owns independent mutable projection state.
  BasisRetractor(const BasisRetractor &other);

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

  using RetractorCreator::create;
  IERetractorCreator::shared_ptr clone() const override {
    return std::make_shared<BasisRetractorCreator>(*this);
  }

  /// Conditioning removes fixed slots from the independent-key selection.
  IERetractorCreator::shared_ptr cloneFor(const KeySet &keys) const override {
    auto result = std::make_shared<BasisRetractorCreator>(*this);
    result->params_ = IERetractorParams::snapshot(params_, &keys);
    result->scopeOrdering(keys);
    if (result->basisKeyFunction) {
      // Resolve external callbacks once; shared types retain only local slots.
      const KeyVector scope(keys.begin(), keys.end());
      KeyVector selected;
      for (Key key : result->basisKeyFunction(scope))
        if (keys.count(key)) selected.push_back(key);
      result->basisKeyFunction = [selected](const KeyVector &available) {
        const KeySet candidates(available.begin(), available.end());
        KeyVector retained;
        for (Key key : selected)
          if (candidates.count(key)) retained.push_back(key);
        return retained;
      };
    }
    return result;
  }

  IERetractor::shared_ptr create(const IECMPoint &point) const override;

  Retractor::shared_ptr create(const NonlinearEqualityConstraints::shared_ptr
                                   constraints) const override {
    KeyVector basis_keys = basisKeyFunction(constraints->keyVector());
    return std::make_shared<BasisRetractor>(constraints, equalityParams_, basis_keys);
  }
};

} // namespace gtdynamics
