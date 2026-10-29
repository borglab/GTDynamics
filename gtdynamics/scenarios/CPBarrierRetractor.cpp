/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CPBarrierRetractor.cpp
 * @brief CPBarrierRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/CPBarrierRetractor.h>
#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <iomanip>

namespace gtdynamics {
using namespace gtsam;

IEConstraintManifold
CPBarrierRetractor::retract(const IEConstraintManifold *manifold,
                            const VectorValues &delta,
                            const std::optional<IndexSet> &blocking_indices,
                            IERetractionInfo *retract_info) const {
  const NonlinearInequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const NonlinearEqualityConstraints &e_constraints = *manifold->eConstraints();

  NonlinearFactorGraph graph;

  // optimize penalty
  Values new_values = manifold->values().retract(delta);
  auto prior_noise = noiseModel::Unit::Create(1);
  double mu = 1e2;
  for (const Key &key : new_values.keys()) {
    Symbol symb(key);
    if (symb.chr() == 'q' || symb.chr() == 'v' || symb.chr() == 'a') {
      graph.addPrior<double>(key, new_values.atDouble(key), prior_noise);
    }
  }
  for (const auto &constraint : e_constraints) {
    graph.add(constraint->penaltyFactor(mu));
  }
  for (size_t idx = 0; idx < i_constraints.size(); idx++) {
    const auto &constraint = i_constraints.at(idx);
    if (blocking_indices && blocking_indices->exists(idx)) {
      graph.add(constraint->penaltyFactorEquality(mu));
    } else {
      graph.add(constraint->penaltyFactor(mu));
    }
  }

  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  // params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, manifold->values(), params);
  Values opt_values = optimizer.optimize();

  // collect active indices
  IndexSet active_indices;
  if (blocking_indices) {
    active_indices = *blocking_indices;
  }
  for (size_t idx = 0; idx < i_constraints.size(); idx++) {
    if (!i_constraints.at(idx)->feasible(opt_values)) {
      active_indices.insert(idx);
    }
  }

  // final optimization to make strictly feasible solution
  NonlinearFactorGraph graph1;
  for (const auto &constraint : e_constraints) {
    graph1.add(constraint->penaltyFactor(1.0));
  }
  for (const auto &constraint_idx : active_indices) {
    graph1.add(i_constraints.at(constraint_idx)->penaltyFactorEquality(1.0));
  }
  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer1(graph1, opt_values, params);
  Values result = optimizer1.optimize();

  return manifold->createWithNewValues(result, active_indices);
}

} // namespace gtdynamics
