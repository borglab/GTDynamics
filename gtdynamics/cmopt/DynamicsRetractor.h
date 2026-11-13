/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsRetractor.h
 * @brief DynamicsRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmopt/Retractor.h>

namespace gtdynamics {

/**
 * Customized hierarchical retractor for kinodynamics components.
 *
 * Variables/factors are split into levels (q, v, and acceleration/dynamics)
 * and solved in sequence with level-specific optimizers and priors.
 *
 * @see README.md#retraction
 */
class DynamicsRetractor : public Retractor {
protected:
  NonlinearFactorGraph merit_graph_;
  MutableLMOptimizer optimizer_wp_q_, optimizer_wp_v_, optimizer_wp_ad_;
  MutableLMOptimizer optimizer_np_q_, optimizer_np_v_, optimizer_np_ad_;
  KeySet basis_q_keys_, basis_v_keys_, basis_ad_keys_;
  std::vector<std::shared_ptr<ConstVarFactor>> const_var_factors_v_,
      const_var_factors_ad_;

public:
  using Retractor::retract;
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<DynamicsRetractor>(*this);
  }

  /**
   * Constructor.
   * @param constraints Equality constraints for the component.
   * @param params Retraction parameters.
   * @param basis_keys Optional basis keys.
   */
  DynamicsRetractor(const NonlinearEqualityConstraints::shared_ptr &constraints,
                    const RetractionParams::shared_ptr &params,
                    std::optional<const KeyVector> basis_keys = {});

  /// A workspace copy owns independent mutable projection state.
  DynamicsRetractor(const DynamicsRetractor &other);

  /// Retraction operation.
  Values retractConstraints(const Values &values) override;

protected:
  /**
   * Classify keys by dynamics level.
   * @param keys Input key container.
   * @param q_keys Output position-level keys.
   * @param v_keys Output velocity-level keys.
   * @param ad_keys Output acceleration/dynamics-level keys.
   */
  template <typename CONTAINER>
  static void classifyKeys(const CONTAINER &keys, KeySet &q_keys,
                           KeySet &v_keys, KeySet &ad_keys);

  /**
   * Refresh linear priors in a graph for selected keys.
   * @param values Values used as prior means.
   * @param keys Keys whose priors are updated.
   * @param graph Graph whose trailing prior factors are replaced.
   */
  void updatePriors(const Values &values, const KeySet &keys,
                    NonlinearFactorGraph &graph);
};

/**
 * Factory for `DynamicsRetractor`.
 *
 * @see README.md#retraction
 */
class DynamicsRetractorCreator : public RetractorCreator {
protected:
  BasisKeyFunction basisKeyFunction;

public:
  DynamicsRetractorCreator(RetractionParams::shared_ptr params)
      : RetractorCreator(params) {}

  /**
   * Constructor with custom basis-key selector.
   * @param params Retraction parameters.
   * @param basisKeyFunction Callback selecting basis keys.
   */
  DynamicsRetractorCreator(RetractionParams::shared_ptr params,
                           BasisKeyFunction basisKeyFunction)
      : RetractorCreator(params), basisKeyFunction(basisKeyFunction) {}

  virtual ~DynamicsRetractorCreator() {}

  using RetractorCreator::create;
  IERetractorCreator::shared_ptr clone() const override {
    return std::make_shared<DynamicsRetractorCreator>(*this);
  }

  /// Conditioning removes fixed slots from the independent-key selection.
  IERetractorCreator::shared_ptr cloneFor(const KeySet &keys) const override {
    auto result = std::make_shared<DynamicsRetractorCreator>(*this);
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
    if (equalityParams_->useBasisKeys) {
      KeyVector basis_keys = basisKeyFunction(constraints->keyVector());
      return std::make_shared<DynamicsRetractor>(constraints, equalityParams_,
                                                 basis_keys);
    } else {
      return std::make_shared<DynamicsRetractor>(constraints, equalityParams_);
    }
  }
};

} // namespace gtdynamics
