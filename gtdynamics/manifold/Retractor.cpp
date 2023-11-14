/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  TspaceBasis.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/manifold/Retractor.h>
#include <gtdynamics/optimizer/AugmentedLagrangianOptimizer.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <stdexcept>

#include "factors/ConstVarFactor.h"
#include "utils/DynamicsSymbol.h"
#include "utils/values.h"

namespace gtsam {

/* ************************************************************************* */
void Retractor::checkFeasible(const NonlinearFactorGraph &graph,
                              const Values &values) const {
  if (params_->check_feasible) {
    if (graph.error(values) > params_->feasible_threshold) {
      std::cout << "fail: " << graph.error(values) << "\n";
    }
  }
}

/* ************************************************************************* */
UoptRetractor::UoptRetractor(const EqualityConstraints::shared_ptr &constraints,
                             const RetractParams::shared_ptr &params)
    : Retractor(params),
      optimizer_(constraints->meritGraph(), params->lm_params) {}

/* ************************************************************************* */
Values UoptRetractor::retractConstraints(const Values &values) {
  optimizer_.setValues(values);
  const Values &result = optimizer_.optimize();
  checkFeasible(optimizer_.mutableGraph(), result);
  return result;
}

/* ************************************************************************* */
Values UoptRetractor::retractConstraints(Values &&values) {
  optimizer_.setValues(values);
  auto result = optimizer_.optimize();
  checkFeasible(optimizer_.mutableGraph(), result);
  return std::move(result);
}

/* ************************************************************************* */
ProjRetractor::ProjRetractor(const EqualityConstraints::shared_ptr &constraints,
                             const RetractParams::shared_ptr &params,
                             std::optional<const KeyVector> basis_keys)
    : Retractor(params), merit_graph_(constraints->meritGraph()) {
  if (params->use_basis_keys) {
    basis_keys_ = *basis_keys;
  }
}

/* ************************************************************************* */
Values ProjRetractor::retractConstraints(const Values &values) {
  LevenbergMarquardtOptimizer optimizer(merit_graph_, values);
  return optimizer.optimize();
}

/* ************************************************************************* */
Values ProjRetractor::retract(const Values &values, const VectorValues &delta) {
  // optimize with priors
  NonlinearFactorGraph graph = merit_graph_;
  Values values_retract_base = retractBaseVariables(values, delta);
  if (params_->use_basis_keys) {
    AddGeneralPriors(values_retract_base, basis_keys_, params_->sigma, graph);
  } else {
    AddGeneralPriors(values_retract_base, params_->sigma, graph);
  }
  // const Values &init_values =
  //     params_->apply_base_retraction ? values_retract_base : values;
  const Values &init_values = values_retract_base;
  LevenbergMarquardtOptimizer optimizer_with_priors(graph, init_values,
                                                    params_->lm_params);
  const Values &result = optimizer_with_priors.optimize();
  if (params_->use_basis_keys &&
      optimizer_with_priors.error() < params_->feasible_threshold) {
    return result;
  }

  // optimize without priors
  LevenbergMarquardtOptimizer optimizer_without_priors(
      merit_graph_, result, params_->lm_params);
  const Values &final_result = optimizer_without_priors.optimize();
  checkFeasible(merit_graph_, final_result);
  return final_result;
}

/* ************************************************************************* */
BasisRetractor::BasisRetractor(const EqualityConstraints::shared_ptr &constraints,
                               const RetractParams::shared_ptr &params,
                               const KeyVector &basis_keys)
    : Retractor(params),
      merit_graph_(constraints->meritGraph()),
      basis_keys_(basis_keys),
      optimizer_(params->lm_params) {
  NonlinearFactorGraph graph;
  KeySet fixed_keys(basis_keys.begin(), basis_keys.end());
  for (const auto &factor : merit_graph_) {
    // check if the factor contains fixed keys
    bool contain_fixed_keys = false;
    for (const Key &key : factor->keys()) {
      if (fixed_keys.exists(key)) {
        contain_fixed_keys = true;
        break;
      }
    }
    // update the factor if it constains fixed keys
    if (contain_fixed_keys) {
      NoiseModelFactor::shared_ptr noise_factor =
          std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto const_var_factor =
          std::make_shared<ConstVarFactor>(noise_factor, fixed_keys);
      if (const_var_factor->checkActive()) {
        factors_with_fixed_vars_.push_back(const_var_factor);
        graph.add(const_var_factor);
      }
    } else {
      graph.add(factor);
    }
  }
  // set the graph for optimizer
  optimizer_.setGraph(graph);
}

/* ************************************************************************* */
Values BasisRetractor::retractConstraints(const Values &values) {
  // set fixed values for the const variable factors
  for (auto &factor : factors_with_fixed_vars_) {
    factor->setFixedValues(values);
  }
  // set initial values for optimizer
  Values opt_values;
  for (const Key &key : optimizer_.graph().keys()) {
    opt_values.insert(key, values.at(key));
  }
  optimizer_.setValues(std::move(opt_values));
  // optimize
  Values result = optimizer_.optimize();

  if (params_->recompute) {
    if (optimizer_.graph().error(result) > 1e-5) {
      std::cout << "recompute\n";
      for (const Key &key : basis_keys_) {
        result.insert(key, values.at(key));
      }
      auto optimizer = LevenbergMarquardtOptimizer(merit_graph_, result);
      result = optimizer.optimize();
      checkFeasible(merit_graph_, result);
      return result;
    }
  }

  // add fixed varaibles to the result
  for (const Key &key : basis_keys_) {
    result.insert(key, values.at(key));
  }
  return result;
}

bool isQLevel(const Key &key) {
  gtdynamics::DynamicsSymbol symb(key);
  return symb.label() == "p" || symb.label() == "q";
}

bool isVLevel(const Key &key) {
  gtdynamics::DynamicsSymbol symb(key);
  return symb.label() == "V" || symb.label() == "v";
}

/* ************************************************************************* */
template <typename CONTAINER>
void AddLinearPriors(NonlinearFactorGraph &graph, const CONTAINER &keys,
                     const Values &values = Values()) {
  for (const Key &key : keys) {
    gtdynamics::DynamicsSymbol symb(key);
    if (symb.label() == "p") {
      graph.addPrior<Pose3>(
          key, values.exists(key) ? values.at<Pose3>(key) : Pose3(),
          noiseModel::Isotropic::Sigma(6, 1e-2));
    } else if (symb.label() == "q" || symb.label() == "v" ||
               symb.label() == "a" || symb.label() == "T") {
      graph.addPrior<double>(key,
                             values.exists(key) ? values.atDouble(key) : 0.0,
                             noiseModel::Isotropic::Sigma(1, 1e-2));
    } else {
      Vector6 value =
          values.exists(key) ? values.at<Vector6>(key) : Vector6::Zero();
      graph.addPrior<Vector6>(key, value,
                              noiseModel::Isotropic::Sigma(6, 1e-2));
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
    const EqualityConstraints::shared_ptr &constraints,
    const RetractParams::shared_ptr &params,
    std::optional<const KeyVector> basis_keys)
    : Retractor(params),
      merit_graph_(constraints->meritGraph()),
      optimizer_wp_q_(params->lm_params),
      optimizer_wp_v_(params->lm_params),
      optimizer_wp_ad_(params->lm_params),
      optimizer_np_q_(params->lm_params),
      optimizer_np_v_(params->lm_params),
      optimizer_np_ad_(params->lm_params) {
  /// classify keys
  KeySet q_keys, v_keys, ad_keys, qv_keys;
  classifyKeys(merit_graph_.keys(), q_keys, v_keys, ad_keys);
  qv_keys = q_keys;
  qv_keys.merge(v_keys);

  /// classify basis keys
  if (params_->use_basis_keys) {
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
  optimizer_np_q_.setGraph(graph_q);
  optimizer_np_v_.setGraph(graph_v);
  optimizer_np_ad_.setGraph(graph_ad);

  // Set the graphs for optimizers
  AddLinearPriors(graph_q, basis_q_keys_);
  AddLinearPriors(graph_v, basis_v_keys_);
  AddLinearPriors(graph_ad, basis_ad_keys_);
  optimizer_wp_q_.setGraph(graph_q);
  optimizer_wp_v_.setGraph(graph_v);
  optimizer_wp_ad_.setGraph(graph_ad);
}

/* ************************************************************************* */
void DynamicsRetractor::updatePriors(const Values &values, const KeySet &keys,
                                     NonlinearFactorGraph &graph) {
  graph.erase(graph.begin() + graph.size() - keys.size(), graph.end());
  AddGeneralPriors(values, keys, params_->sigma, graph);
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
  // AddGeneralPriors(values, all_basis_keys, params_->sigma, graph_wp_all);

  // LevenbergMarquardtParams lm_parmas = params_->lm_params;
  // lm_parmas.setMaxIterations(10);
  // LevenbergMarquardtOptimizer optimizer_wp_all(graph_wp_all, known_values,
  // params_->lm_params); auto result = optimizer_wp_all.optimize();

  // LevenbergMarquardtOptimizer optimzier_np_all(cc_->merit_graph_, result,
  // params_->lm_params); result = optimzier_np_all.optimize();
  // checkFeasible(cc_->merit_graph_, result);

  return known_values;
}

}  // namespace gtsam
