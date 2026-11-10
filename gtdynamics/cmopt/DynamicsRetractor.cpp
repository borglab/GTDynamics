/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsRetractor.cpp
 * @brief DynamicsRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmopt/DynamicsRetractor.h>
#include <gtdynamics/cmcopt/IECMPoint.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/PenaltyOptimizer.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "factors/ConstVarFactor.h"
#include "utils/DynamicsSymbol.h"
#include "utils/values.h"

namespace gtdynamics {
using namespace gtsam;

bool isQLevel(const Key &key) {
  DynamicsSymbol symbol(key);
  return symbol.label() == "p" || symbol.label() == "q";
}

bool isVLevel(const Key &key) {
  DynamicsSymbol symbol(key);
  return symbol.label() == "V" || symbol.label() == "v";
}

/* ************************************************************************* */
template <typename CONTAINER>
void AddLinearPriors(NonlinearFactorGraph &graph, const CONTAINER &keys,
                     const Values &values = Values()) {
  for (const Key &key : keys) {
    DynamicsSymbol symbol(key);
    if (symbol.label() == "p") {
      graph.addPrior<gtsam::Pose3>(
          key,
          values.exists(key) ? values.at<gtsam::Pose3>(key) : gtsam::Pose3(),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-2));
    } else if (symbol.label() == "q" || symbol.label() == "v" ||
               symbol.label() == "a" || symbol.label() == "T") {
      graph.addPrior<double>(key,
                             values.exists(key) ? values.atDouble(key) : 0.0,
                             gtsam::noiseModel::Isotropic::Sigma(1, 1e-2));
    } else {
      gtsam::Vector6 value = values.exists(key) ? values.at<gtsam::Vector6>(key)
                                                : gtsam::Vector6::Zero();
      graph.addPrior<gtsam::Vector6>(
          key, value, gtsam::noiseModel::Isotropic::Sigma(6, 1e-2));
    }
  }
}

/* ************************************************************************* */
template <typename CONTAINER>
void DynamicsRetractor::classifyKeys(const CONTAINER &keys, KeySet &q_keys,
                                     KeySet &v_keys, KeySet &ad_keys) {
  for (const Key &key : keys) {
    if (isQLevel(key)) {
      q_keys.insert(key);
    } else if (isVLevel(key)) {
      v_keys.insert(key);
    } else {
      ad_keys.insert(key);
    }
  }
}

/* ************************************************************************* */
DynamicsRetractor::DynamicsRetractor(
    const NonlinearEqualityConstraints::shared_ptr &constraints,
    const RetractionParams::shared_ptr &params,
    std::optional<const KeyVector> basis_keys)
    : Retractor(params), merit_graph_(constraints->penaltyGraph()),
      optimizer_wp_q_(params->lmParams), optimizer_wp_v_(params->lmParams),
      optimizer_wp_ad_(params->lmParams), optimizer_np_q_(params->lmParams),
      optimizer_np_v_(params->lmParams), optimizer_np_ad_(params->lmParams) {
  /// classify keys
  KeySet q_keys, v_keys, ad_keys, qv_keys;
  classifyKeys(merit_graph_.keys(), q_keys, v_keys, ad_keys);
  qv_keys = q_keys;
  qv_keys.merge(v_keys);

  /// classify basis keys
  if (equalityParams_->useBasisKeys) {
    if (!basis_keys) {
      throw std::runtime_error("basis keys not provided for retractor.");
    }
    classifyKeys(*basis_keys, basis_q_keys_, basis_v_keys_, basis_ad_keys_);
  } else {
    basis_q_keys_ = q_keys;
    basis_v_keys_ = v_keys;
    basis_ad_keys_ = ad_keys;
  }
  basis_q_keys_ = q_keys;

  /// classify factors
  NonlinearFactorGraph graph_q, graph_v, graph_ad;
  for (const auto &factor : merit_graph_) {
    int lvl = 0;
    for (const auto &key : factor->keys()) {
      if (isQLevel(key)) {
        lvl = std::max(lvl, 0);
      } else if (isVLevel(key)) {
        lvl = std::max(lvl, 1);
      } else {
        lvl = std::max(lvl, 2);
      }
    }
    if (lvl == 0) {
      graph_q.add(factor);
    } else if (lvl == 1) {
      graph_v.add(factor);
    } else {
      graph_ad.add(factor);
    }
  }

  /// Update as ConstVarFactors
  std::tie(graph_v, const_var_factors_v_) = ConstVarGraph(graph_v, q_keys);
  std::tie(graph_ad, const_var_factors_ad_) = ConstVarGraph(graph_ad, qv_keys);

  // Set the graphs for optimizers
  setOptimizerGraph(optimizer_np_q_, graph_q);
  setOptimizerGraph(optimizer_np_v_, graph_v);
  setOptimizerGraph(optimizer_np_ad_, graph_ad);

  // Set the graphs for optimizers
  AddLinearPriors(graph_q, basis_q_keys_);
  AddLinearPriors(graph_v, basis_v_keys_);
  AddLinearPriors(graph_ad, basis_ad_keys_);
  setOptimizerGraph(optimizer_wp_q_, graph_q);
  setOptimizerGraph(optimizer_wp_v_, graph_v);
  setOptimizerGraph(optimizer_wp_ad_, graph_ad);
}

DynamicsRetractor::DynamicsRetractor(const DynamicsRetractor &other)
    : Retractor(other), merit_graph_(other.merit_graph_),
      optimizer_wp_q_(equalityParams_->lmParams),
      optimizer_wp_v_(equalityParams_->lmParams),
      optimizer_wp_ad_(equalityParams_->lmParams),
      optimizer_np_q_(equalityParams_->lmParams),
      optimizer_np_v_(equalityParams_->lmParams),
      optimizer_np_ad_(equalityParams_->lmParams),
      basis_q_keys_(other.basis_q_keys_), basis_v_keys_(other.basis_v_keys_),
      basis_ad_keys_(other.basis_ad_keys_),
      const_var_factors_v_(other.const_var_factors_v_),
      const_var_factors_ad_(other.const_var_factors_ad_) {
  // The with/without-prior graphs share each call's detached fixed factors.
  auto detachPair = [this](
      const MutableLMOptimizer &sourceWith,
      const MutableLMOptimizer &sourceWithout, MutableLMOptimizer &with,
      MutableLMOptimizer &without,
      std::vector<std::shared_ptr<ConstVarFactor>> &fixed) {
    NonlinearFactorGraph graph = sourceWith.graph();
    const size_t split = graph.size();
    graph.push_back(sourceWithout.graph());
    detachFixedFactors(graph, fixed);
    NonlinearFactorGraph withGraph, withoutGraph;
    for (size_t i = 0; i < split; ++i) withGraph.push_back(graph[i]);
    for (size_t i = split; i < graph.size(); ++i)
      withoutGraph.push_back(graph[i]);
    setOptimizerGraph(with, withGraph);
    setOptimizerGraph(without, withoutGraph);
  };
  setOptimizerGraph(optimizer_wp_q_, other.optimizer_wp_q_.graph());
  setOptimizerGraph(optimizer_np_q_, other.optimizer_np_q_.graph());
  detachPair(other.optimizer_wp_v_, other.optimizer_np_v_, optimizer_wp_v_,
             optimizer_np_v_, const_var_factors_v_);
  detachPair(other.optimizer_wp_ad_, other.optimizer_np_ad_, optimizer_wp_ad_,
             optimizer_np_ad_, const_var_factors_ad_);
}

IERetractor::shared_ptr DynamicsRetractorCreator::create(
    const IECMPoint &point) const {
  validatePointType(point);
  auto constraints =
      std::make_shared<NonlinearEqualityConstraints>(*point.eConstraints());
  if (!equalityParams_->useBasisKeys)
    return std::make_shared<DynamicsRetractor>(constraints, boundParams(point));

  KeyVector slots;
  for (const auto &[slot, key] : point.bindings()) slots.push_back(slot);
  KeyVector keys = basisKeyFunction(slots);
  for (Key &key : keys) key = point.bindings().at(key);
  return std::make_shared<DynamicsRetractor>(constraints, boundParams(point), keys);
}

/* ************************************************************************* */
void DynamicsRetractor::updatePriors(const Values &values, const KeySet &keys,
                                     NonlinearFactorGraph &graph) {
  graph.erase(graph.begin() + graph.size() - keys.size(), graph.end());
  AddGeneralPriors(values, keys, equalityParams_->sigma, graph);
}

/* ************************************************************************* */
Values DynamicsRetractor::retractConstraints(const Values &values) {
  Values known_values;

  // solve q level
  updatePriors(values, basis_q_keys_, optimizer_wp_q_.mutableGraph());
  optimizer_wp_q_.setValues(SubValues(values, optimizer_wp_q_.graph().keys()));
  auto results_q = optimizer_wp_q_.optimize();

  optimizer_np_q_.setValues(results_q);
  results_q = optimizer_np_q_.optimize();
  known_values.insert(results_q);

  // solve v level
  updatePriors(values, basis_v_keys_, optimizer_wp_v_.mutableGraph());
  for (auto &factor : const_var_factors_v_) {
    factor->setFixedValues(known_values);
  }
  optimizer_wp_v_.setValues(SubValues(values, optimizer_wp_v_.graph().keys()));
  auto results_v = optimizer_wp_v_.optimize();

  optimizer_np_v_.setValues(results_v);
  results_v = optimizer_np_v_.optimize();
  known_values.insert(results_v);

  // solve a and dynamics level
  updatePriors(values, basis_ad_keys_, optimizer_wp_ad_.mutableGraph());
  for (auto &factor : const_var_factors_ad_) {
    factor->setFixedValues(known_values);
  }
  optimizer_wp_ad_.setValues(
      SubValues(values, optimizer_wp_ad_.graph().keys()));
  auto results_ad = optimizer_wp_ad_.optimize();

  optimizer_np_ad_.setValues(results_ad);
  results_ad = optimizer_np_ad_.optimize();
  known_values.insert(results_ad);
  checkFeasible(merit_graph_, known_values);

  // NonlinearFactorGraph graph_wp_all = cc_->merit_graph_;
  // KeySet all_basis_keys = basis_q_keys_;
  // all_basis_keys.merge(basis_v_keys_);
  // all_basis_keys.merge(basis_ad_keys_);
  // AddGeneralPriors(values, all_basis_keys, equalityParams_->sigma, graph_wp_all);

  // LevenbergMarquardtParams lmParams = equalityParams_->lmParams;
  // lmParams.setMaxIterations(10);
  // LevenbergMarquardtOptimizer optimizer_wp_all(graph_wp_all, known_values,
  // equalityParams_->lmParams); auto result = optimizer_wp_all.optimize();

  // LevenbergMarquardtOptimizer optimizer_np_all(cc_->merit_graph_, result,
  // equalityParams_->lmParams); result = optimizer_np_all.optimize();
  // checkFeasible(cc_->merit_graph_, result);

  return known_values;
}

} // namespace gtdynamics
