/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IEQuadrupedUtils.h>
#include <gtdynamics/manifold/GeneralPriorFactor.h>
#include <gtdynamics/utils/DebugUtils.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
Vision60Retractor::Vision60Retractor(const IEVision60Robot &robot,
                                     const IEConstraintManifold &manifold,
                                     const Params &params)
    : IERetractor(), robot_(robot), params_(params), graph_q_(), graph_v_(),
      graph_ad_() {

  /// Create merit graph for e-constriants and i-constraints
  merit_graph_ = manifold.eCC()->merit_graph_;
  const InequalityConstraints &i_constraints = *manifold.iConstraints();
  for (const auto &i_constraint : i_constraints) {
    merit_graph_.add(i_constraint->createBarrierFactor(1.0));
  }

  /// Split keys into 3 levels
  KeySet q_keys, v_keys, ad_keys, qv_keys;
  ClassifyKeysByLevel(merit_graph_.keys(), q_keys, v_keys, ad_keys);
  qv_keys = q_keys;
  qv_keys.merge(v_keys);

  /// Split basis keys into 3 levels
  if (params.use_basis_keys) {
    KeyVector basis_keys = robot_.getBasisKeyFunc()(manifold.eCC());
    ClassifyKeysByLevel(basis_keys, basis_q_keys_, basis_v_keys_,
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
IEConstraintManifold Vision60Retractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {

  Values known_values;
  IndexSet active_indices;
  const Values &values = manifold->values();
  Values new_values = values.retract(delta);
  const InequalityConstraints &i_constraints = *manifold->iConstraints();

  // solve q level with priors
  NonlinearFactorGraph graph_np_q = graph_q_;
  NonlinearFactorGraph graph_wp_q = graph_np_q;
  AddGeneralPriors(new_values, basis_q_keys_, params_.prior_sigma, graph_wp_q);
  Values init_values_q = SubValues(values, graph_wp_q.keys());
  LevenbergMarquardtOptimizer optimizer_wp_q(graph_wp_q, init_values_q,
                                             params_.lm_params);
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
                                             params_.lm_params);
  results_q = optimizer_np_q.optimize();
  known_values.insert(results_q);

  // solve v level with priors
  NonlinearFactorGraph graph_np_v = graph_v_;
  for (auto &factor : const_var_factors_v_) {
    factor->setFixedValues(known_values);
    graph_np_v.add(factor);
  }
  NonlinearFactorGraph graph_wp_v = graph_np_v;
  AddGeneralPriors(new_values, basis_v_keys_, params_.prior_sigma, graph_wp_v);
  Values init_values_v = SubValues(values, graph_wp_v.keys());
  LevenbergMarquardtOptimizer optimizer_wp_v(graph_wp_v, init_values_v,
                                             params_.lm_params);
  Values results_v = optimizer_wp_v.optimize();

  // solve v level without priors
  for (const auto &i : i_indices_v_) {
    if (blocking_indices && blocking_indices->exists(i) ||
        !i_constraints.at(i)->feasible(results_v)) {
      active_indices.insert(i);
      graph_np_v.add(i_constraints.at(i)->createL2Factor(1.0));
    }
  }
  LevenbergMarquardtOptimizer optimizer_np_v(graph_np_v, results_v,
                                             params_.lm_params);
  results_v = optimizer_np_v.optimize();
  known_values.insert(results_v);

  // solve a and dynamics level with priors
  NonlinearFactorGraph graph_np_ad = graph_ad_;
  for (auto &factor : const_var_factors_ad_) {
    factor->setFixedValues(known_values);
    graph_np_ad.add(factor);
  }
  NonlinearFactorGraph graph_wp_ad = graph_np_ad;
  AddGeneralPriors(new_values, basis_ad_keys_, params_.prior_sigma,
                   graph_wp_ad);
  Values init_values_ad = SubValues(values, graph_wp_ad.keys());
  LevenbergMarquardtOptimizer optimizer_wp_ad(graph_wp_ad, init_values_ad,
                                              params_.lm_params);
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
                                              params_.lm_params);
  results_ad = optimizer_np_ad.optimize();
  known_values.insert(results_ad);

  checkFeasible(merit_graph_, known_values);

  return manifold->createWithNewValues(known_values, active_indices);
}

/* ************************************************************************* */
void Vision60Retractor::checkFeasible(const NonlinearFactorGraph &graph,
                                      const Values &values) const {
  if (params_.check_feasible) {
    if (graph.error(values) > params_.feasible_threshold) {
      std::cout << "fail: " << graph.error(values) << "\n";
    }
  }
}

/* ************************************************************************* */
IERetractor::shared_ptr
Vision60RetractorCreator::create(const IEConstraintManifold &manifold) const {
  return std::make_shared<Vision60Retractor>(robot_, manifold, params_);
}

/* ************************************************************************* */
IERetractor::shared_ptr Vision60RetractorMultiPhaseCreator::create(
    const IEConstraintManifold &manifold) const {
  size_t k =
      gtdynamics::DynamicsSymbol(*manifold.values().keys().begin()).time();
  return std::make_shared<Vision60Retractor>(
      vision60_multi_phase_.robotAtStep(k), manifold, params_);
}

/* ************************************************************************* */
TspaceBasis::shared_ptr Vision60MultiPhaseTspaceBasisCreator::create(
    const ConnectedComponent::shared_ptr cc, const Values &values) const {
  size_t k = gtdynamics::DynamicsSymbol(*values.keys().begin()).time();
  KeyVector basis_keys =
      vision60_multi_phase_.robotAtStep(k).getBasisKeyFunc()(cc);
  size_t manifold_dim = values.dim() - cc->constraints_.dim();
  return TspaceBasis::create(params_, cc, values, basis_keys, manifold_dim);
}

} // namespace gtsam
