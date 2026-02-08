/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IERetractor.cpp
 * @brief Retraction operation implementations for manifold with boundaries.
 * @author: Yetong Zhang
 */

#include <gtdynamics/factors/GeneralPriorFactor.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


namespace gtdynamics {
using namespace gtsam;

namespace {

/**
 * Run constrained penalty optimization for one IE retraction subproblem.
 *
 * Why this helper exists:
 * 1) Old CMCOpt code used a long-lived custom penalty optimizer interface that
 *    accepted `(cost, e_constraints, i_constraints, initial_values)` directly.
 * 2) New constrained GTSAM expects constructing a `ConstrainedOptProblem` and
 *    then running `PenaltyOptimizer(problem, initialValues, params).optimize()`.
 * 3) Several retractor paths (boundary move, standard retract) need the same
 *    "assemble + solve" sequence, and keeping it centralized avoids subtle
 *    divergence in LM/penalty parameter wiring.
 *
 * Behavioral intent (compatibility):
 * - Preserve previous flow: solve a penalty-form constrained problem using the
 *   provided cost graph and current equality/inequality constraints.
 * - Keep LM settings controlled by `IERetractorParams::lm_params` by copying
 *   them into `PenaltyOptimizerParams::lmParams` right before solve.
 *
 * Notes for review:
 * - This helper mutates `params->penalty_params->lmParams` intentionally so the
 *   retractor-level LM tuning remains the single source of truth.
 * - It is `namespace {}` local on purpose: this is a file-scoped adapter layer,
 *   not reusable public API.
 */
Values RunPenaltyOptimization(
    const NonlinearFactorGraph &cost,
    const NonlinearEqualityConstraints &e_constraints,
    const NonlinearInequalityConstraints &i_constraints,
    const Values &initial_values,
    const IERetractorParams::shared_ptr &params) {
  auto penalty_params = params->penalty_params;
  penalty_params->lmParams = params->lm_params;
  ConstrainedOptProblem problem(cost, e_constraints, i_constraints);
  PenaltyOptimizer optimizer(problem, initial_values, penalty_params);
  return optimizer.optimize();
}

}  // namespace


/* ************************************************************************* */
IEConstraintManifold
IERetractor::moveToBoundary(const IEConstraintManifold *manifold,
                            const IndexSet &blocking_indices,
                            IERetractInfo *retract_info) const {
  VectorValues delta = manifold->values().zeroVectors();
  return retract(manifold, delta, blocking_indices, retract_info);
}

/* ************************************************************************* */
IEConstraintManifold
BarrierRetractor::moveToBoundary(const IEConstraintManifold *manifold,
                                 const IndexSet &blocking_indices,
                                 IERetractInfo *retract_info) const {

  NonlinearEqualityConstraints blocking_constraints;
  for (const auto &blocking_idx : blocking_indices) {
    blocking_constraints.push_back(
        manifold->iConstraints()->at(blocking_idx)->createEqualityConstraint());
  }
  NonlinearFactorGraph cost = blocking_constraints.penaltyGraph(1.0);

  Values opt_values = RunPenaltyOptimization(
      cost, *manifold->eConstraints(), *manifold->iConstraints(),
      manifold->values(), params_);

  NonlinearFactorGraph merit_graph = cost;
  merit_graph.add(manifold->eConstraints()->penaltyGraph(1.0));
  merit_graph.add(manifold->iConstraints()->penaltyGraph(1.0));
  LevenbergMarquardtOptimizer optimizer_np(merit_graph, opt_values,
                                           params_->lm_params);
  Values result = optimizer_np.optimize();

  if (params_->check_feasible) {
    size_t k = DynamicsSymbol(manifold->values().keys().front()).time();
    std::string k_string = "(" + std::to_string(k) + ")";
    if (!CheckFeasible(merit_graph, result,
                       "penalty move to boundary" + k_string,
                       params_->feasible_threshold)) {
    }
  }

  return manifold->createWithNewValues(result, blocking_indices);
}

/* ************************************************************************* */
IEConstraintManifold
BarrierRetractor::retract(const IEConstraintManifold *manifold,
                          const VectorValues &delta,
                          const std::optional<IndexSet> &blocking_indices,
                          IERetractInfo *retract_info) const {

  // cost as priors
  Values new_values = manifold->values().retract(delta);
  KeyVector prior_keys = use_basis_keys_ ? basis_keys_ : new_values.keys();
  NonlinearFactorGraph prior_graph;
  params_->addPriors(new_values, prior_keys, prior_graph);

  // i and e constraints
  NonlinearInequalityConstraints i_constraints = *manifold->iConstraints();
  NonlinearEqualityConstraints e_constraints = *manifold->eConstraints();
  if (blocking_indices) {
    for (const auto &idx : *blocking_indices) {
      const auto &constraint = i_constraints.at(idx);
      e_constraints.push_back(constraint->createEqualityConstraint());
    }
  }

  // run penalty optimization
  const Values &init_values =
      params_->init_values_as_x ? manifold->values() : new_values;
  Values opt_values = RunPenaltyOptimization(
      prior_graph, e_constraints, i_constraints, init_values, params_);

  // collect active indices
  IndexSet active_indices;
  if (blocking_indices) {
    active_indices = *blocking_indices;
  }
  for (size_t constraint_idx = 0; constraint_idx < i_constraints.size();
       constraint_idx++) {
    if (!i_constraints.at(constraint_idx)->feasible(opt_values)) {
      active_indices.insert(constraint_idx);
    }
  }

  // final optimization without priors to make strictly feasible solution
  NonlinearEqualityConstraints active_constraints =
      *manifold->eConstraints();
  for (const auto &constraint_idx : active_indices) {
    active_constraints.push_back(
        i_constraints.at(constraint_idx)->createEqualityConstraint());
  }
  NonlinearFactorGraph graph_np = active_constraints.penaltyGraph(1.0);
  graph_np.add(i_constraints.penaltyGraph(1.0));
  LevenbergMarquardtOptimizer optimizer_np(graph_np, opt_values,
                                           params_->lm_params);
  Values result = optimizer_np.optimize();

  // check and ensure feasible
  if (params_->check_feasible) {
    size_t k = DynamicsSymbol(manifold->values().keys().front()).time();
    std::string k_string = "(" + std::to_string(k) + ")";
    if (!CheckFeasible(graph_np, result, "penalty retraction" + k_string,
                       params_->feasible_threshold)) {
      if (params_->ensure_feasible) {
        return retract(manifold, 0.7 * delta, blocking_indices, retract_info);
      }
    }
  }

  // record retraction info
  if (retract_info) {
    // retract_info->num_lm_iters =
    //     opt_info.num_iters.back() + optimizer_np.iterations();
    // TODO
  }

  return manifold->createWithNewValues(result, active_indices);
}

/* ************************************************************************* */
KinodynamicHierarchicalRetractor::KinodynamicHierarchicalRetractor(
    const IEConstraintManifold &manifold, const Params::shared_ptr &params,
    const std::optional<KeyVector> &basis_keys)
    : IERetractor(params), graph_q_(), graph_v_(), graph_ad_() {

  /// Create merit graph for e-constriants and i-constraints
  merit_graph_ = manifold.eConstraints()->penaltyGraph(1.0);
  const auto &i_constraints = *manifold.iConstraints();
  for (const auto &i_constraint : i_constraints) {
    merit_graph_.add(i_constraint->penaltyFactor(1.0));
  }

  /// Split keys into 3 levels
  KeySet q_keys, v_keys, ad_keys, qv_keys;
  ClassifyKeysByLevel(merit_graph_.keys(), q_keys, v_keys, ad_keys);
  qv_keys = q_keys;
  qv_keys.merge(v_keys);

  /// Split basis keys into 3 levels
  if (basis_keys) {
    ClassifyKeysByLevel(*basis_keys, basis_q_keys_, basis_v_keys_,
                        basis_ad_keys_);
  } else {
    basis_q_keys_ = q_keys;
    basis_v_keys_ = v_keys;
    basis_ad_keys_ = ad_keys;
  }

  /// Split merit graph into 3 levels
  for (const auto &factor : merit_graph_) {
    int lvl = IdentifyLevel(factor->keys());
    if (lvl == 0) {
      graph_q_.add(factor);
    } else if (lvl == 1) {
      graph_v_.add(factor);
    } else {
      graph_ad_.add(factor);
    }
  }

  /// Split i-constraints into 3 levels
  for (size_t i = 0; i < i_constraints.size(); i++) {
    const auto &i_constraint = i_constraints.at(i);
    int lvl = IdentifyLevel(i_constraint->keys());
    if (lvl == 0) {
      i_indices_q_.insert(i);
    } else if (lvl == 1) {
      i_indices_v_.insert(i);
    } else {
      i_indices_ad_.insert(i);
    }
  }

  /// Update factors in v, ad levels as ConstVarFactors
  std::tie(graph_v_, const_var_factors_v_) = ConstVarGraph(graph_v_, q_keys);
  std::tie(graph_ad_, const_var_factors_ad_) =
      ConstVarGraph(graph_ad_, qv_keys);
}

/* ************************************************************************* */
IEConstraintManifold KinodynamicHierarchicalRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices,
    IERetractInfo *retract_info) const {

  size_t k = DynamicsSymbol(manifold->values().keys().front()).time();
  std::string k_string = "(" + std::to_string(k) + ")";

  Values known_values;
  IndexSet active_indices;
  const Values &values = manifold->values();
  Values new_values = values.retract(delta);
  const NonlinearInequalityConstraints &i_constraints = *manifold->iConstraints();
  bool failed = false;

  // solve q level with priors
  NonlinearFactorGraph graph_np_q = graph_q_;
  NonlinearFactorGraph graph_wp_q = graph_np_q;
  params_->addPriors(new_values, basis_q_keys_, graph_wp_q);
  Values init_values_q = SubValues(values, graph_wp_q.keys());
  LevenbergMarquardtOptimizer optimizer_wp_q(graph_wp_q, init_values_q,
                                             params_->lm_params);
  Values results_q = optimizer_wp_q.optimize();

  // solve q level without priors
  for (const auto &i : i_indices_q_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_q)) {
      active_indices.insert(i);
      graph_np_q.add(i_constraints.at(i)->penaltyFactorEquality(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer_np_q(graph_np_q, results_q,
                                             params_->lm_params);
  Values new_results_q = optimizer_np_q.optimize();
  known_values.insert(new_results_q);
  if (params_->check_feasible) {
    if (!CheckFeasible(graph_np_q, new_results_q, "q-level" + k_string,
                       params_->feasible_threshold)) {
      failed = true;
    }
  }

  // solve v level with priors
  NonlinearFactorGraph graph_np_v = graph_v_;
  for (auto &factor : const_var_factors_v_) {
    factor->setFixedValues(known_values);
  }
  NonlinearFactorGraph graph_wp_v = graph_np_v;
  params_->addPriors(new_values, basis_v_keys_, graph_wp_v);
  Values init_values_v = SubValues(values, graph_wp_v.keys());
  LevenbergMarquardtOptimizer optimizer_wp_v(graph_wp_v, init_values_v,
                                             params_->lm_params);
  Values results_v = optimizer_wp_v.optimize();

  // solve v level without priors
  for (const auto &i : i_indices_v_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_v)) {
      active_indices.insert(i);
      // TODO: turn into const v graph
      graph_np_v.add(i_constraints.at(i)->penaltyFactorEquality(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer_np_v(graph_np_v, results_v,
                                             params_->lm_params);
  Values new_results_v = optimizer_np_v.optimize();
  known_values.insert(new_results_v);
  if (params_->check_feasible) {
    if (!CheckFeasible(graph_np_v, new_results_v, "v-level" + k_string,
                       params_->feasible_threshold)) {
      failed = true;
    }
  }

  // solve a and dynamics level with priors
  NonlinearFactorGraph graph_np_ad = graph_ad_;
  for (auto &factor : const_var_factors_ad_) {
    factor->setFixedValues(known_values);
  }
  NonlinearFactorGraph graph_wp_ad = graph_np_ad;
  params_->addPriors(new_values, basis_ad_keys_, graph_wp_ad);
  Values init_values_ad = SubValues(values, graph_wp_ad.keys());
  LevenbergMarquardtOptimizer optimizer_wp_ad(graph_wp_ad, init_values_ad,
                                              params_->lm_params);
  Values results_ad = optimizer_wp_ad.optimize();

  // solve a and dynamics level without priors
  for (const auto &i : i_indices_ad_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_ad)) {
      active_indices.insert(i);
      graph_np_ad.add(i_constraints.at(i)->penaltyFactorEquality(1.0));
    }
  }

  LevenbergMarquardtOptimizer optimizer_np_ad(graph_np_ad, results_ad,
                                              params_->lm_params);
  Values new_results_ad = optimizer_np_ad.optimize();
  known_values.insert(new_results_ad);
  if (params_->check_feasible) {
    if (!CheckFeasible(graph_np_ad, new_results_ad, "ad-level" + k_string,
                       params_->feasible_threshold)) {
      failed = true;
    }
  }

  if (retract_info) {
    retract_info->num_lm_iters = optimizer_wp_q.iterations();
    retract_info->num_lm_iters += optimizer_np_q.iterations();
    retract_info->num_lm_iters += optimizer_wp_v.iterations();
    retract_info->num_lm_iters += optimizer_np_v.iterations();
    retract_info->num_lm_iters += optimizer_wp_ad.iterations();
    retract_info->num_lm_iters += optimizer_np_ad.iterations();
  }

  // solve a and dynamics level without priors
  if (failed) {
    LevenbergMarquardtOptimizer optimizer_all(merit_graph_, known_values,
                                              params_->lm_params);
    known_values = optimizer_all.optimize();
    if (retract_info) {
      retract_info->num_lm_iters += optimizer_all.iterations();
    }
    if (!CheckFeasible(merit_graph_, known_values, "all-levels" + k_string,
                       params_->feasible_threshold)) {
      if (params_->ensure_feasible) {
        return *manifold;
      }
    }
  }

  return manifold->createWithNewValues(known_values, active_indices);
}

} // namespace gtdynamics
