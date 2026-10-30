/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file BarrierRetractor.cpp
 * @brief BarrierRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <cmath>
#include <gtdynamics/cmcopt/AffineProjection.h>
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/factors/GeneralPriorFactor.h>
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
 *    then running `PenaltyOptimizer(problem, initialValues,
 * params).optimize()`. 3) Several retractor paths (boundary move, standard
 * retract) need the same "assemble + solve" sequence, and keeping it
 * centralized avoids subtle divergence in LM/penalty parameter wiring.
 *
 * Behavioral intent (compatibility):
 * - Preserve previous flow: solve a penalty-form constrained problem using the
 *   provided cost graph and current equality/inequality constraints.
 * - Keep LM settings controlled by `IERetractorParams::lmParams` by copying
 *   them into `PenaltyOptimizerParams::lmParams` right before solve.
 *
 * Notes for review:
 * - This helper mutates `params->penaltyParams->lmParams` intentionally so the
 *   retractor-level LM tuning remains the single source of truth.
 * - It is `namespace {}` local on purpose: this is a file-scoped adapter layer,
 *   not reusable public API.
 */
Values
RunPenaltyOptimization(const NonlinearFactorGraph &cost,
                       const NonlinearEqualityConstraints &e_constraints,
                       const NonlinearInequalityConstraints &i_constraints,
                       const Values &initial_values,
                       const IERetractorParams::shared_ptr &params) {
  auto penaltyParams = params->penaltyParams;
  penaltyParams->lmParams = params->lmParams;
  ConstrainedOptProblem problem(cost, e_constraints, i_constraints);
  PenaltyOptimizer optimizer(problem, initial_values, penaltyParams);
  return optimizer.optimize();
}

} // namespace

IEConstraintManifold
BarrierRetractor::moveToBoundary(const IEConstraintManifold *manifold,
                                 const IndexSet &blocking_indices,
                                 IERetractionInfo *retract_info) const {
  if (params_->useReducedCoordinates && manifold->hasAffineEqualities())
    return retractReduced(*manifold, manifold->values().zeroVectors(),
                          blocking_indices, retract_info);

  // Turning blocking inequalities into equalities picks the boundary face that
  // the next mode should land on.
  NonlinearEqualityConstraints blocking_constraints;
  for (const auto &blocking_idx : blocking_indices) {
    blocking_constraints.push_back(
        manifold->iConstraints()->at(blocking_idx)->createEqualityConstraint());
  }
  NonlinearFactorGraph cost = blocking_constraints.penaltyGraph(1.0);

  Values opt_values = RunPenaltyOptimization(cost, *manifold->eConstraints(),
                                             *manifold->iConstraints(),
                                             manifold->values(), params_);

  NonlinearFactorGraph merit_graph = cost;
  merit_graph.add(manifold->eConstraints()->penaltyGraph(1.0));
  merit_graph.add(manifold->iConstraints()->penaltyGraph(1.0));
  LevenbergMarquardtOptimizer optimizer_np(merit_graph, opt_values,
                                           params_->lmParams);
  Values result = optimizer_np.optimize();

  if (params_->checkFeasible) {
    size_t k = DynamicsSymbol(manifold->values().keys().front()).time();
    std::string k_string = "(" + std::to_string(k) + ")";
    if (!CheckFeasible(merit_graph, result,
                       "penalty move to boundary" + k_string,
                       params_->feasibleThreshold)) {
    }
  }

  return manifold->createWithNewValues(result, blocking_indices);
}

/* ************************************************************************* */
IEConstraintManifold
BarrierRetractor::retract(const IEConstraintManifold *manifold,
                          const VectorValues &delta,
                          const std::optional<IndexSet> &blocking_indices,
                          IERetractionInfo *retract_info) const {
  if (params_->useReducedCoordinates && manifold->hasAffineEqualities())
    return retractReduced(*manifold, delta,
                          blocking_indices.value_or(IndexSet{}), retract_info);

  // The prior graph is the quadratic stay-close term centered at x + delta.
  // This implements the metric-projection objective in thesis Eq. (4.26), and
  // blocking indices below turn it into the on-corner form in Eq. (4.46).
  Values new_values = manifold->values().retract(delta);
  KeyVector prior_keys = use_basis_keys_ ? basis_keys_ : new_values.keys();
  NonlinearFactorGraph prior_graph;
  params_->addPriors(new_values, prior_keys, prior_graph);

  // Promote blocked inequalities to equalities so the retraction stays on the
  // same active face instead of immediately stepping off it.
  NonlinearInequalityConstraints i_constraints = *manifold->iConstraints();
  NonlinearEqualityConstraints e_constraints = *manifold->eConstraints();
  if (blocking_indices) {
    for (const auto &idx : *blocking_indices) {
      const auto &constraint = i_constraints.at(idx);
      e_constraints.push_back(constraint->createEqualityConstraint());
    }
  }

  // First solve the soft penalty subproblem, then recover the hard active set
  // from the resulting feasible-or-nearly-feasible point.
  const Values &init_values =
      params_->initValuesAsX ? manifold->values() : new_values;
  Values opt_values = RunPenaltyOptimization(
      prior_graph, e_constraints, i_constraints, init_values, params_);

  // Positive g(x) means the penalty optimum lies outside g(x)<=0, even when
  // its violation is below the feasibility tolerance. Retain those faces for
  // cleanup and the next tangent QP; feasible() would silently discard them.
  IndexSet active_indices;
  if (blocking_indices) {
    active_indices = *blocking_indices;
  }
  for (size_t constraint_idx = 0; constraint_idx < i_constraints.size();
       constraint_idx++) {
    if (i_constraints.at(constraint_idx)->active(opt_values)) {
      active_indices.insert(constraint_idx);
    }
  }

  // Drop the stay-close priors and clean up on the final active manifold so the
  // returned point is strictly feasible in the chosen mode.
  NonlinearEqualityConstraints active_constraints = *manifold->eConstraints();
  for (const auto &constraint_idx : active_indices) {
    active_constraints.push_back(
        i_constraints.at(constraint_idx)->createEqualityConstraint());
  }
  NonlinearFactorGraph graph_np = active_constraints.penaltyGraph(1.0);
  graph_np.add(i_constraints.penaltyGraph(1.0));
  LevenbergMarquardtOptimizer optimizer_np(graph_np, opt_values,
                                           params_->lmParams);
  Values result = optimizer_np.optimize();

  // check and ensure feasible
  if (params_->checkFeasible) {
    size_t k = DynamicsSymbol(manifold->values().keys().front()).time();
    std::string k_string = "(" + std::to_string(k) + ")";
    if (!CheckFeasible(graph_np, result, "penalty retraction" + k_string,
                       params_->feasibleThreshold)) {
      if (params_->ensureFeasible) {
        return retract(manifold, 0.7 * delta, blocking_indices, retract_info);
      }
    }
  }

  // record retraction info
  if (retract_info) {
    // retract_info->numLMIterations =
    //     opt_info.num_iters.back() + optimizer_np.iterations();
    // TODO
  }

  return manifold->createWithNewValues(result, active_indices);
}

/* ************************************************************************* */
IEConstraintManifold BarrierRetractor::retractReduced(
    const IEConstraintManifold &manifold, const VectorValues &delta,
    const IndexSet &blocking, IERetractionInfo *info) const {
  const auto priorKeys =
      use_basis_keys_ ? std::optional<KeyVector>(basis_keys_) : std::nullopt;
  // Keep the original projection objective and original whitened constraint
  // residuals: extra normalization would change finite penalty iterates.
  AffineProjection projection(manifold, delta, *params_, priorKeys, blocking,
                              params_->feasibleThreshold,
                              std::max(1e-7, params_->feasibleThreshold),
                              false);
  if (const auto result = projection.feasibleProjection(blocking, info))
    return manifold.createWithNewValues(result->values, result->active);
  if (!projection.dim()) {
    const Values result = projection.recover(Vector());
    return manifold.createWithNewValues(
        result, projection.validateResult(result, blocking));
  }
  if (!params_->penaltyParams)
    throw std::invalid_argument("Missing reduced penalty parameters");
  auto penalty =
      std::make_shared<PenaltyOptimizerParams>(*params_->penaltyParams);
  penalty->lmParams = params_->lmParams;
  penalty->storeOptProgress = info != nullptr;
  const auto problem = projection.buildProblem(blocking);
  PenaltyOptimizer optimizer(
      problem, projection.initialValues(params_->initValuesAsX), penalty);
  if (info)
    ++info->numPenaltySolveCalls;
  const Values optimum = optimizer.optimize();
  if (info)
    for (const auto &state : optimizer.progress())
      info->numLMIterations += state.unconstrainedIterations;
  const Values ambient =
      projection.recover(optimum.at<Vector>(AffineProjection::kReduced));

  // Match BarrierRetractor's active-face cleanup. Equality elimination stays
  // exact throughout; the cleanup only adjusts reduced inequality coordinates.
  IndexSet active = blocking;
  for (size_t i = 0; i < manifold.iConstraints()->size(); ++i)
    if (manifold.iConstraints()->at(i)->active(ambient))
      active.insert(i);
  const auto cleanupProblem = projection.buildProblem(active);
  NonlinearFactorGraph cleanup =
      cleanupProblem.eConstraints().penaltyGraph(1.0);
  cleanup.add(cleanupProblem.iConstraints().penaltyGraph(1.0));
  Values cleaned = optimum;
  if (!cleanup.empty()) {
    LevenbergMarquardtOptimizer lm(cleanup, optimum, params_->lmParams);
    cleaned = lm.optimize();
    if (info)
      info->numLMIterations += lm.iterations();
  }
  const Values result =
      projection.recover(cleaned.at<Vector>(AffineProjection::kReduced));
  // Unlike legacy ambient cleanup, this opt-in path never silently accepts
  // infeasible results or loses a required blocking face.
  return manifold.createWithNewValues(
      result, projection.validateResult(result, active));
}

} // namespace gtdynamics
