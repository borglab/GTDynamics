/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizer.cpp
 * @brief Implementation of manifold optimizer.
 * @author: Yetong Zhang
 */

#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <stack>

#include "manifold/Retractor.h"

namespace gtsam {

/* ************************************************************************* */
ManifoldOptimizerParameters::ManifoldOptimizerParameters()
    : Base(),
      cc_params(std::make_shared<ConstraintManifold::Params>()),
      retract_init(true) {}

/* ************************************************************************* */
ConnectedComponent::shared_ptr ManifoldOptimizer::findConnectedComponent(
    const gtdynamics::EqualityConstraints& constraints,
    const gtsam::Key start_key, gtsam::KeySet& keys,
    const gtsam::VariableIndex& var_index) const {
  std::set<size_t> constraint_indices;

  std::stack<gtsam::Key> key_stack;
  key_stack.push(start_key);
  while (!key_stack.empty()) {
    gtsam::Key key = key_stack.top();
    key_stack.pop();
    // find all constraints connected to key
    for (const auto& constraint_index : var_index[key]) {
      constraint_indices.insert(constraint_index);
      // TODO: use keys() in constraint
      auto constraint_factor =
          constraints.at(constraint_index)->createFactor(1.0);
      for (const auto& neighbor_key : constraint_factor->keys()) {
        if (keys.find(neighbor_key) != keys.end()) {
          keys.erase(neighbor_key);
          key_stack.push(neighbor_key);
        }
      }
    }
  }

  gtdynamics::EqualityConstraints cc_constraints;
  for (const auto& constraint_index : constraint_indices) {
    cc_constraints.emplace_back(constraints.at(constraint_index));
  }
  return std::make_shared<ConnectedComponent>(cc_constraints);
}

/* ************************************************************************* */
std::vector<ConnectedComponent::shared_ptr>
ManifoldOptimizer::identifyConnectedComponents(
    const gtdynamics::EqualityConstraints& constraints) const {
  // Get all the keys in constraints.
  // TODO(yetong): create VariableIndex from EqualityConstraints
  gtsam::NonlinearFactorGraph constraint_graph;
  for (const auto& constraint : constraints) {
    constraint_graph.add(constraint->createFactor(1.0));
  }
  gtsam::VariableIndex constraint_var_index =
      gtsam::VariableIndex(constraint_graph);
  gtsam::KeySet constraint_keys;
  for (const auto& it : constraint_var_index) {
    constraint_keys.insert(it.first);
  }

  // Find connected component using DFS algorithm.
  std::vector<ConnectedComponent::shared_ptr> components;
  while (!constraint_keys.empty()) {
    Key key = *constraint_keys.begin();
    constraint_keys.erase(key);
    components.emplace_back(findConnectedComponent(
        constraints, key, constraint_keys, constraint_var_index));
  }
  return components;
}

}  // namespace gtsam
