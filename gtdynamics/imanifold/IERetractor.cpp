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
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtdynamics;

namespace gtsam {

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
BarrierRetractor::retract(const IEConstraintManifold *manifold,
                          const VectorValues &delta,
                          const std::optional<IndexSet> &blocking_indices,
                          IERetractInfo *retract_info) const {

  const gtdynamics::InequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const gtdynamics::EqualityConstraints &e_constraints =
      *manifold->eConstraints();

  NonlinearFactorGraph graph;

  // optimize with prior and barrier factors
  Values new_values = manifold->values().retract(delta);
  auto prior_noise = noiseModel::Unit::Create(1);
  KeyVector prior_keys = use_basis_keys_ ? basis_keys_ : new_values.keys();
  params_->addPriors(new_values, prior_keys, graph);

  for (const auto &constraint : e_constraints) {
    graph.add(constraint->createFactor(1.0));
  }
  for (const auto &constraint : i_constraints) {
    graph.add(constraint->createBarrierFactor(1.0));
  }
  if (blocking_indices) {
    for (const auto &idx : *blocking_indices) {
      const auto &constraint = i_constraints.at(idx);
      graph.add(constraint->createL2Factor(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer(
      graph, params_->init_values_as_x ? manifold->values() : new_values,
      params_->lm_params);
  Values opt_values = optimizer.optimize();
  if (retract_info) {
    retract_info->num_lm_iters = optimizer.iterations();
  }

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
  NonlinearFactorGraph graph_np;
  for (const auto &constraint : e_constraints) {
    graph_np.add(constraint->createFactor(1.0));
  }
  for (const auto &constraint_idx : active_indices) {
    graph_np.add(i_constraints.at(constraint_idx)
                     ->createEqualityConstraint()
                     ->createFactor(1.0));
  }
  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer_np(graph_np, opt_values,
                                           params_->lm_params);
  Values result = optimizer_np.optimize();
  if (retract_info) {
    retract_info->num_lm_iters += optimizer_np.iterations();
  }
  if (params_->check_feasible) {
    size_t k = DynamicsSymbol(manifold->values().keys().front()).time();
    std::string k_string = "(" + std::to_string(k) + ")";
    if (!CheckFeasible(graph_np, result, "barrier retraction" + k_string,
                       params_->feasible_threshold)) {
      if (params_->ensure_feasible) {
        if (retract_info) {
          retract_info->retract_delta = manifold->values().zeroVectors();
          auto vec_diff = retract_info->retract_delta - delta;
          double delta_norm = delta.norm();
          double diff_norm = vec_diff.norm();
          retract_info->deviate_rate = diff_norm / delta_norm;
        }
        return *manifold;
      }
    }
  }

  if (retract_info) {
    retract_info->retract_delta = manifold->values().localCoordinates(result);
    auto vec_diff = retract_info->retract_delta - delta;

    double delta_norm = delta.norm();
    double diff_norm = vec_diff.norm();
    retract_info->deviate_rate = diff_norm / delta_norm;
  }

  return manifold->createWithNewValues(result, active_indices);
}

/* ************************************************************************* */
KinodynamicHierarchicalRetractor::KinodynamicHierarchicalRetractor(
    const IEConstraintManifold &manifold, const Params::shared_ptr &params,
    const std::optional<KeyVector> &basis_keys)
    : IERetractor(params), graph_q_(), graph_v_(), graph_ad_() {

  /// Create merit graph for e-constriants and i-constraints
  merit_graph_ = manifold.eConstraints()->meritGraph();
  const auto &i_constraints = *manifold.iConstraints();
  for (const auto &i_constraint : i_constraints) {
    merit_graph_.add(i_constraint->createBarrierFactor(1.0));
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
  const InequalityConstraints &i_constraints = *manifold->iConstraints();
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
      graph_np_q.add(i_constraints.at(i)->createL2Factor(1.0));
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
      graph_np_v.add(i_constraints.at(i)->createL2Factor(1.0));
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
      graph_np_ad.add(i_constraints.at(i)->createL2Factor(1.0));
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
        if (retract_info) {
          retract_info->retract_delta = manifold->values().zeroVectors();
          auto vec_diff = retract_info->retract_delta - delta;
          double delta_norm = delta.norm();
          double diff_norm = vec_diff.norm();
          retract_info->deviate_rate = diff_norm / delta_norm;
        }
        return *manifold;
      }
    }
  }

  if (retract_info) {
    retract_info->retract_delta =
        manifold->values().localCoordinates(known_values);
    auto vec_diff = retract_info->retract_delta - delta;
    double delta_norm = delta.norm();
    double diff_norm = vec_diff.norm();
    retract_info->deviate_rate = diff_norm / delta_norm;
  }

  return manifold->createWithNewValues(known_values, active_indices);
}

} // namespace gtsam
