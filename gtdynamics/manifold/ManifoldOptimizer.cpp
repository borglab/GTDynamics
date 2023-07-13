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
#include <gtdynamics/manifold/SubstituteFactor.h>
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
ConnectedComponent::shared_ptr ManifoldOptimizer::IdentifyConnectedComponent(
    const gtdynamics::EqualityConstraints& constraints,
    const gtsam::Key start_key, gtsam::KeySet& keys,
    const gtsam::VariableIndex& var_index) {
  std::set<size_t> constraint_indices;

  std::stack<gtsam::Key> key_stack;
  key_stack.push(start_key);
  while (!key_stack.empty()) {
    gtsam::Key key = key_stack.top();
    key_stack.pop();
    // find all constraints connected to key
    for (const auto& constraint_index : var_index[key]) {
      constraint_indices.insert(constraint_index);
      for (const auto &neighbor_key :
           constraints.at(constraint_index)->keys()) {
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
ManifoldOptimizer::IdentifyConnectedComponents(
    const gtdynamics::EqualityConstraints& constraints) {
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
    components.emplace_back(IdentifyConnectedComponent(
        constraints, key, constraint_keys, constraint_var_index));
  }
  return components;
}

NonlinearFactorGraph
ManifoldOptimizer::ManifoldGraph(const NonlinearFactorGraph &graph,
                                 const std::map<Key, Key> &var2man_keymap,
                                 const Values& fc_manifolds) {
  NonlinearFactorGraph manifold_graph;
  for (const auto &factor : graph) {
    std::map<Key, Key> replacement_map;
    for (const Key &key : factor->keys()) {
      if (var2man_keymap.find(key) != var2man_keymap.end()) {
        replacement_map[key] = var2man_keymap.at(key);
      }
    }
    if (replacement_map.size() > 0) {
      NoiseModelFactor::shared_ptr noise_factor =
          std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto subs_factor = std::make_shared<SubstituteFactor>(
          noise_factor, replacement_map, fc_manifolds);
      if (subs_factor->checkActive()) {
        manifold_graph.add(subs_factor);
      }
    } else {
      manifold_graph.add(factor);
    }
  }
  return manifold_graph;
}

}  // namespace gtsam
