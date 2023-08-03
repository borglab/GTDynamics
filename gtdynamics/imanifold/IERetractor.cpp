/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IERetractor.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/imanifold/IECartPoleWithFriction.h>
#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtdynamics/imanifold/IERetractor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
IEConstraintManifold
IERetractor::moveToBoundary(const IEConstraintManifold *manifold,
                            const IndexSet &blocking_indices) const {
  VectorValues delta = manifold->values().zeroVectors();
  return retract(manifold, delta, blocking_indices);
}

/* ************************************************************************* */
IEConstraintManifold BarrierRetractor::retract(
    const IEConstraintManifold *manifold, const VectorValues &delta,
    const std::optional<IndexSet> &blocking_indices) const {

  const gtdynamics::InequalityConstraints &i_constraints =
      *manifold->iConstraints();
  const gtdynamics::EqualityConstraints &e_constraints =
      manifold->eCC()->constraints_;

  NonlinearFactorGraph graph;

  // optimize barrier
  Values new_values = manifold->values().retract(delta);
  auto prior_noise = noiseModel::Unit::Create(1);
  for (const Key &key : new_values.keys()) {
    graph.addPrior<double>(key, new_values.atDouble(key), prior_noise);
  }
  for (const auto &constraint : e_constraints) {
    graph.add(constraint->createFactor(1e-1));
  }
  for (const auto &constraint : i_constraints) {
    graph.add(constraint->createBarrierFactor(1e-1));
  }

  LevenbergMarquardtParams params;
  // params.setVerbosityLM("SUMMARY");
  // params.setlambdaUpperBound(1e10);
  LevenbergMarquardtOptimizer optimizer(graph, manifold->values(), params);
  Values opt_values = optimizer.optimize();

  // collect active indices
  IndexSet active_indices;
  for (size_t constraint_idx = 0; constraint_idx < i_constraints.size();
       constraint_idx++) {
    if (!i_constraints.at(constraint_idx)->feasible(opt_values)) {
      active_indices.insert(constraint_idx);
    }
  }

  // final optimization to make strictly feasible solution
  NonlinearFactorGraph graph1;
  for (const auto &constraint : e_constraints) {
    graph1.add(constraint->createFactor(1.0));
  }
  for (const auto &constraint_idx : active_indices) {
    graph1.add(i_constraints.at(constraint_idx)
                   ->createEqualityConstraint()
                   ->createFactor(1.0));
  }
  // std::cout << "optimize for feasibility\n";
  // active_indices.print("active indices\n");
  LevenbergMarquardtOptimizer optimizer1(graph1, opt_values, params);
  Values result = optimizer1.optimize();

  return manifold->createWithNewValues(result, active_indices);
}


} // namespace gtsam
