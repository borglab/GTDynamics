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

#include <gtdynamics/cmopt/ManifoldOptimizer.h>
#include <gtdynamics/factors/SubstituteFactor.h>
#include <gtdynamics/utils/GraphUtils.h>

#include <stack>

namespace gtdynamics {

/* ************************************************************************* */
Values ManifoldOptimizationProblem::unconstrainedValues() const {
  return SubValues(values, unconstrainedKeys);
}

/* ************************************************************************* */
EManifoldValues ManifoldOptimizationProblem::manifolds() const {
  EManifoldValues equalityManifolds;
  for (const Key& key : manifoldKeys) {
    equalityManifolds.insert({key, values.at<ConstraintManifold>(key)});
  }
  return equalityManifolds;
}

/* ************************************************************************* */
EManifoldValues ManifoldOptimizationProblem::constManifolds() const {
  EManifoldValues equalityManifolds;
  for (const Key& key : fixedManifolds.keys()) {
    equalityManifolds.insert({key, fixedManifolds.at<ConstraintManifold>(key)});
  }
  return equalityManifolds;
}

/* ************************************************************************* */
std::pair<size_t, size_t> ManifoldOptimizationProblem::problemDimension() const {
  size_t values_dim = values.dim();
  size_t graph_dim = 0;
  for (const auto& factor : graph) {
    graph_dim += factor->dim();
  }
  return std::make_pair(graph_dim, values_dim);
}

/* ************************************************************************* */
void ManifoldOptimizationProblem::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "found " << components.size()
            << " components: " << manifoldKeys.size() << " free, "
            << fixedManifolds.size() << " fixed\n";
  for (const Key& cm_key : manifoldKeys) {
    const auto& cm = values.at(cm_key).cast<ConstraintManifold>();
    std::cout << "component " << keyFormatter(cm_key) << ":\tdimension "
              << cm.dim() << "\n";
    for (const auto& key : cm.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
  std::cout << "fully constrained manifolds:\n";
  for (const auto& cm_key : fixedManifolds.keys()) {
    const auto& cm = fixedManifolds.at(cm_key).cast<ConstraintManifold>();
    for (const auto& key : cm.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
}

/* ************************************************************************* */
ManifoldOptimizerParameters::ManifoldOptimizerParameters()
    : Base(),
      constraintManifoldParams(std::make_shared<ConstraintManifold::Params>()),
      retractInitialValues(true) {}

/* ************************************************************************* */
NonlinearEqualityConstraints::shared_ptr ManifoldOptimizer::identifyConnectedComponent(
    const NonlinearEqualityConstraints& constraints, const gtsam::Key start_key,
    gtsam::KeySet& keys, const gtsam::VariableIndex& var_index) {
  std::set<size_t> constraint_indices;

  std::stack<gtsam::Key> key_stack;
  key_stack.push(start_key);
  while (!key_stack.empty()) {
    gtsam::Key key = key_stack.top();
    key_stack.pop();
    // find all constraints connected to key
    for (const auto& constraint_index : var_index[key]) {
      constraint_indices.insert(constraint_index);
      for (const auto& neighbor_key :
           constraints.at(constraint_index)->keys()) {
        if (keys.find(neighbor_key) != keys.end()) {
          keys.erase(neighbor_key);
          key_stack.push(neighbor_key);
        }
      }
    }
  }

  auto cc_constraints = std::make_shared<NonlinearEqualityConstraints>();
  for (const auto& constraint_index : constraint_indices) {
    cc_constraints->push_back(constraints.at(constraint_index));
  }
  return cc_constraints;
}

/* ************************************************************************* */
std::vector<NonlinearEqualityConstraints::shared_ptr>
ManifoldOptimizer::identifyConnectedComponents(
    const NonlinearEqualityConstraints& constraints) {
  // Get all the keys in constraints.
  gtsam::VariableIndex constraint_var_index(constraints);
  gtsam::KeySet constraint_keys = constraints.keys();

  // Find connected component using DFS algorithm.
  std::vector<NonlinearEqualityConstraints::shared_ptr> components;
  while (!constraint_keys.empty()) {
    Key key = *constraint_keys.begin();
    constraint_keys.erase(key);
    components.emplace_back(identifyConnectedComponent(
        constraints, key, constraint_keys, constraint_var_index));
  }
  return components;
}

/* ************************************************************************* */
NonlinearFactorGraph ManifoldOptimizer::manifoldGraph(
    const NonlinearFactorGraph& graph, const std::map<Key, Key>& var2man_keymap,
    const Values& fc_manifolds) {
  NonlinearFactorGraph manifold_graph;
  for (const auto& factor : graph) {
    std::map<Key, Key> replacement_map;
    for (const Key& key : factor->keys()) {
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

/* ************************************************************************* */
ManifoldOptimizationProblem ManifoldOptimizer::transformProblem(
    const EConsOptProblem& equalityConstrainedProblem) const {
  ManifoldOptimizationProblem mopt_problem;
  mopt_problem.components =
      identifyConnectedComponents(equalityConstrainedProblem.e_constraints_);
  constructManifoldOptimizationValues(equalityConstrainedProblem, mopt_problem);
  constructManifoldOptimizationGraph(equalityConstrainedProblem, mopt_problem);
  return mopt_problem;
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldOptimizationValues(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) const {
  constructManifoldValues(equalityConstrainedProblem, mopt_problem);
  constructUnconstrainedValues(equalityConstrainedProblem, mopt_problem);
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldValues(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) const {
  for (size_t i = 0; i < mopt_problem.components.size(); i++) {
    // Find the values of variables in the component.
    const auto& component = mopt_problem.components.at(i);
    const KeySet component_keys = component->keys();
    if (component_keys.empty()) continue;
    const Key component_key = *component_keys.begin();
    Values component_values;
    for (const Key& key : component_keys) {
      component_values.insert(key, equalityConstrainedProblem.values_.at(key));
    }
    // Construct manifold value
    auto constraint_manifold = ConstraintManifold(
        component, component_values, p_.constraintManifoldParams, p_.retractInitialValues);
    // check if the manifold is fully constrained
    if (constraint_manifold.dim() > 0) {
      mopt_problem.values.insert(component_key, constraint_manifold);
      mopt_problem.manifoldKeys.insert(component_key);
    } else {
      mopt_problem.fixedManifolds.insert(component_key, constraint_manifold);
    }
  }
}

/* ************************************************************************* */
void ManifoldOptimizer::constructUnconstrainedValues(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) {
  // Find out which variables are unconstrained
  mopt_problem.unconstrainedKeys = equalityConstrainedProblem.costs_.keys();
  KeySet& unconstrainedKeys = mopt_problem.unconstrainedKeys;
  for (const auto& component : mopt_problem.components) {
    for (const Key& key : component->keys()) {
      if (unconstrainedKeys.find(key) != unconstrainedKeys.end()) {
        unconstrainedKeys.erase(key);
      }
    }
  }
  // Copy the values of unconstrained variables
  for (const Key& key : unconstrainedKeys) {
    mopt_problem.values.insert(key, equalityConstrainedProblem.values_.at(key));
  }
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldOptimizationGraph(
    const EConsOptProblem& equalityConstrainedProblem,
    ManifoldOptimizationProblem& mopt_problem) {
  // Construct base key to component map.
  std::map<Key, Key> key_component_map;
  for (const Key& cm_key : mopt_problem.manifoldKeys) {
    const auto& cm = mopt_problem.values.at(cm_key).cast<ConstraintManifold>();
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }
  for (const Key& cm_key : mopt_problem.fixedManifolds.keys()) {
    const auto& cm =
        mopt_problem.fixedManifolds.at(cm_key).cast<ConstraintManifold>();
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }

  // Turn factors involved with constraint variables into SubstituteFactor.
  for (const auto& factor : equalityConstrainedProblem.costs_) {
    std::map<Key, Key> replacement_map;
    for (const Key& key : factor->keys()) {
      if (key_component_map.find(key) != key_component_map.end()) {
        replacement_map[key] = key_component_map.at(key);
      }
    }
    if (replacement_map.size() > 0) {
      NoiseModelFactor::shared_ptr noise_factor =
          std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto subs_factor = std::make_shared<SubstituteFactor>(
          noise_factor, replacement_map, mopt_problem.fixedManifolds);
      if (subs_factor->checkActive()) {
        mopt_problem.graph.add(subs_factor);
      }
    } else {
      mopt_problem.graph.add(factor);
    }
  }
}

/* ************************************************************************* */
Values ManifoldOptimizer::baseValues(const ManifoldOptimizationProblem& mopt_problem,
                                     const Values& nopt_values) const {
  Values base_values;
  for (const Key& key : mopt_problem.fixedManifolds.keys()) {
    base_values.insert(mopt_problem.fixedManifolds.at(key)
                           .cast<ConstraintManifold>()
                           .values());
  }
  for (const Key& key : mopt_problem.unconstrainedKeys) {
    base_values.insert(key, nopt_values.at(key));
  }
  for (const Key& key : mopt_problem.manifoldKeys) {
    if (p_.retractFinalValues) {
      base_values.insert(
          nopt_values.at(key).cast<ConstraintManifold>().feasibleValues());
    } else {
      base_values.insert(
          nopt_values.at(key).cast<ConstraintManifold>().values());
    }
  }
  return base_values;
}

/* ************************************************************************* */
VectorValues ManifoldOptimizer::baseTangentVector(
    const ManifoldOptimizationProblem& mopt_problem, const Values& values,
    const VectorValues& delta) const {
  VectorValues tangentVector;
  for (const Key& key : mopt_problem.unconstrainedKeys) {
    tangentVector.insert(key, delta.at(key));
  }
  for (const Key& key : mopt_problem.manifoldKeys) {
    tangentVector.insert(
        values.at(key).cast<ConstraintManifold>().basis()->computeTangentVector(
            delta.at(key)));
  }
  return tangentVector;
}

/* ************************************************************************* */
ManifoldOptimizationProblem ManifoldOptimizer::initializeManifoldOptimizationProblem(
    const gtsam::NonlinearFactorGraph& costs,
    const NonlinearEqualityConstraints& constraints,
    const gtsam::Values& init_values) const {
  EConsOptProblem equalityConstrainedProblem(costs, constraints, init_values);
  return transformProblem(equalityConstrainedProblem);
}

}  // namespace gtdynamics
