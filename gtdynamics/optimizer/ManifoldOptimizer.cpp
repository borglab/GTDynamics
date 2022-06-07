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

#include <gtdynamics/optimizer/ManifoldOptimizer.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>

#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/linear/VectorValues.h>

#include <stack>

namespace gtsam {

/* ************************************************************************* */
ManifoldOptimizer::Params::Params()
    : cc_params(boost::make_shared<ConstraintManifold::Params>()),
      retract_init(true) {}

/* ************************************************************************* */
ConnectedComponent::shared_ptr ManifoldOptimizer::dfsFindConnectedComponent(
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
          constraints_.at(constraint_index)->createFactor(1.0);
      for (const auto& neighbor_key : constraint_factor->keys()) {
        if (keys.find(neighbor_key) != keys.end()) {
          keys.erase(neighbor_key);
          key_stack.push(neighbor_key);
        }
      }
    }
  }

  gtdynamics::EqualityConstraints constraints;
  for (const auto& constraint_index : constraint_indices) {
    constraints.emplace_back(constraints_.at(constraint_index));
  }
  return boost::make_shared<ConnectedComponent>(constraints);
}

/* ************************************************************************* */
void ManifoldOptimizer::identifyConnectedComponents() {
  // Get all the keys in constraints.
  // TODO(yetong): create VariableIndex from EqualityConstraints
  gtsam::NonlinearFactorGraph constraint_graph;
  for (const auto& constraint : constraints_) {
    constraint_graph.add(constraint->createFactor(1.0));
  }
  gtsam::VariableIndex constraint_var_index =
      gtsam::VariableIndex(constraint_graph);
  gtsam::KeySet constraint_keys;
  for (const auto& it : constraint_var_index) {
    constraint_keys.insert(it.first);
  }

  // Find connected component using algorithm from
  // https://www.geeksforgeeks.org/connected-components-in-an-undirected-graph/
  while (!constraint_keys.empty()) {
    Key key = *constraint_keys.begin();
    constraint_keys.erase(key);
    components_.emplace_back(dfsFindConnectedComponent(
        key, constraint_keys, constraint_var_index));
  }
}

void ManifoldOptimizer::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "found " << components_.size() << " components:\n";
  for (size_t i=0; i<components_.size(); i++) {
    const auto& component = components_.at(i);
    std::cout << "component " << i << ":";
    for (const auto& key : component->keys) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
}

}  // namespace gtsam
