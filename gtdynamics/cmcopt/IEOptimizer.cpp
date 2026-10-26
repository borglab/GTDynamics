/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  IEManifoldOptimizer.cpp
 * @brief Tagent space basis implementations.
 * @author: Yetong Zhang
 */

#include <utils/DynamicsSymbol.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <stack>

namespace gtdynamics {
using namespace gtsam;


/* ************************************************************************* */
std::map<Key, Key>
IEOptimizer::varToManifoldKeyMap(const IEManifoldValues &manifolds) {
  std::map<Key, Key> keymap_var2manifold;
  for (const auto &it : manifolds) {
    for (const Key &variable_key : it.second.values().keys()) {
      keymap_var2manifold.emplace(variable_key, it.first);
    }
  }
  return keymap_var2manifold;
}

struct ComponentInfo {
  IndexSet e_indices;
  IndexSet i_indices;
  KeySet keys;
};

/* ************************************************************************* */
ComponentInfo identifyConnectedComponent(
    const NonlinearEqualityConstraints &e_constraints,
    const NonlinearInequalityConstraints &i_constraints,
    const VariableIndex &e_var_index,
    const VariableIndex &i_var_index, const Key start_key) {
  ComponentInfo component_info;

  // Flood-fill the bipartite variable/constraint graph to recover one
  // manifold-with-boundary component.
  std::stack<Key> key_stack;
  key_stack.push(start_key);
  while (!key_stack.empty()) {
    Key key = key_stack.top();
    key_stack.pop();
    // find all constraints connected to key
    if (e_var_index.find(key) != e_var_index.end()) {
      for (const auto &constraint_index : e_var_index[key]) {
        component_info.e_indices.insert(constraint_index);
        for (const auto &neighbor_key :
            e_constraints.at(constraint_index)->keys()) {
          if (!component_info.keys.exists(neighbor_key)) {
            component_info.keys.insert(neighbor_key);
            key_stack.push(neighbor_key);
          }
        }
      }
    }
    if (i_var_index.find(key) != i_var_index.end()) {
      for (const auto &constraint_index : i_var_index[key]) {
        component_info.i_indices.insert(constraint_index);
        for (const auto &neighbor_key :
            i_constraints.at(constraint_index)->keys()) {
          if (!component_info.keys.exists(neighbor_key)) {
            component_info.keys.insert(neighbor_key);
            key_stack.push(neighbor_key);
          }
        }
      }
    }
  }
  return component_info;
}

/* ************************************************************************* */
  std::vector<std::pair<NonlinearEqualityConstraints::shared_ptr,
                      NonlinearInequalityConstraints::shared_ptr>>
identifyConnectedComponents(
    const NonlinearEqualityConstraints &e_constraints,
    const NonlinearInequalityConstraints &i_constraints) {

  VariableIndex e_var_index(e_constraints);
  VariableIndex i_var_index(i_constraints);

  // Get all the keys in constraints.
  KeySet keys = e_constraints.keys();
  keys.merge(i_constraints.keys());

  std::vector<std::pair<NonlinearEqualityConstraints::shared_ptr,
                        NonlinearInequalityConstraints::shared_ptr>>
      components;
  while (!keys.empty()) {
    auto component_info = identifyConnectedComponent(
        e_constraints, i_constraints, e_var_index, i_var_index, *keys.begin());
    for (const Key &key : component_info.keys) {
      keys.erase(key);
    }

    // e_constraints
    auto component_e_constraints =
        std::make_shared<NonlinearEqualityConstraints>();
    for (const auto &idx : component_info.e_indices) {
      component_e_constraints->push_back(e_constraints.at(idx));
    }
    // i_constraints
    auto component_i_constraints =
        std::make_shared<NonlinearInequalityConstraints>();
    for (const auto &idx : component_info.i_indices) {
      component_i_constraints->push_back(i_constraints.at(idx));
    }

    components.emplace_back(component_e_constraints, component_i_constraints);
  }
  return components;
}

/* ************************************************************************* */
IEManifoldValues IEOptimizer::identifyManifolds(
    const NonlinearEqualityConstraints &e_constraints,
    const NonlinearInequalityConstraints &i_constraints,
    const Values &values,
    const IEConstraintManifold::Params::shared_ptr &iecm_params) {
  // Each thesis Eq. (4.33) connected component becomes one Eq. (4.34)
  // IEConstraintManifold with its own active set, basis, and tangent cone.
  auto components = identifyConnectedComponents(e_constraints, i_constraints);

  IEManifoldValues ie_manifolds;
  for (const auto &[component_e_constraints, component_i_constraints] : components) {
    KeySet keys = component_e_constraints->keys();
    keys.merge(component_i_constraints->keys());
    Key component_key = *keys.begin();
    Values component_values;
    for (const Key &key : keys) {
      component_values.insert(key, values.at(key));
    }
    ie_manifolds.emplace(
        component_key, IEConstraintManifold(iecm_params, component_e_constraints,
                                            component_i_constraints,
                                            component_values));
  }
  return ie_manifolds;
}

/* ************************************************************************* */
Values IEOptimizer::identifyUnconstrainedValues(
      const NonlinearEqualityConstraints &e_constraints,
      const NonlinearInequalityConstraints &i_constraints,
      const Values &values) {
  Values unconstrainedValues;
  KeySet constrained_keys = e_constraints.keys();
  constrained_keys.merge(i_constraints.keys());
  for (const Key& key: values.keys()) {
    if (!constrained_keys.exists(key)) {
      unconstrainedValues.insert(key, values.at(key));
    }
  }
  return unconstrainedValues;
}

/* ************************************************************************* */
VectorValues
IEOptimizer::computeTangentVector(const IEManifoldValues &manifolds,
                                  const VectorValues &delta) {
  VectorValues tangentVector;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const Vector &xi = delta.at(key);
    VectorValues tv = manifolds.at(key).eBasis()->computeTangentVector(xi);
    tangentVector.insert(tv);
  }
  return tangentVector;
}

/* ************************************************************************* */
std::pair<IndexSetMap, VectorValues>
IEOptimizer::projectTangentCone(const IEManifoldValues &manifolds,
                                const VectorValues &v) {
  VectorValues proj_v;
  IndexSetMap active_indices_all;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const Vector &xi = v.at(key);
    IndexSet active_indices;
    Vector proj_xi;
    // Project independently on each component cone C_x as in thesis Eq. (4.42)
    // because the connected components have separated the coupling structure.
    std::tie(active_indices, proj_xi) = it.second.projectTangentCone(xi);
    proj_v.insert(key, proj_xi);
    active_indices_all.emplace(key, active_indices);
  }
  return std::make_pair(active_indices_all, proj_v);
}

/* ************************************************************************* */
IEManifoldValues
IEOptimizer::retractManifolds(const IEManifoldValues &manifolds,
                              const VectorValues &delta) {
  IEManifoldValues newManifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const Vector &xi = delta.at(key);
    newManifolds.emplace(key, it.second.retract(xi));
  }
  return newManifolds;
}

/* ************************************************************************* */
Values IEOptimizer::equalityManifolds(const IEManifoldValues &manifolds) {
  Values equalityManifolds;
  for (const auto &it : manifolds) {
    equalityManifolds.insert(it.first, it.second.eConstraintManifold());
  }
  return equalityManifolds;
}

/* ************************************************************************* */
std::pair<Values, Values>
IEOptimizer::equalityManifolds(const IEManifoldValues &manifolds,
                        const IndexSetMap &active_indices) {
  Values equalityManifolds;
  Values const_e_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    ConstraintManifold e_manifold =
        it.second.eConstraintManifold(active_indices.at(key));
    if (e_manifold.dim() > 0) {
      equalityManifolds.insert(key, e_manifold);
    } else {
      const_e_manifolds.insert(key, e_manifold);
    }
  }
  return std::make_pair(equalityManifolds, const_e_manifolds);
}

/* ************************************************************************* */
bool IEOptimizer::isSameMode(const IEManifoldValues &manifolds1,
                             const IEManifoldValues &manifolds2) {
  for (const auto &it : manifolds1) {
    const IndexSet &indices1 = it.second.activeIndices();
    const IndexSet &indices2 = manifolds2.at(it.first).activeIndices();
    if (indices1.size() != indices2.size()) {
      return false;
    }
    if (!std::equal(indices1.begin(), indices1.end(), indices2.begin())) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
IndexSetMap
IEOptimizer::identifyChangeIndices(const IEManifoldValues &manifolds,
                                   const IEManifoldValues &newManifolds) {
  IndexSetMap change_indices_map;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const IndexSet &indices = it.second.activeIndices();
    const IndexSet &new_indices = newManifolds.at(key).activeIndices();
    IndexSet change_indices;
    for (const auto &idx : new_indices) {
      if (indices.find(idx) == indices.end()) {
        change_indices.insert(idx);
      }
    }
    if (change_indices.size() > 0) {
      change_indices_map.insert({key, change_indices});
    }
  }
  return change_indices_map;
}

/* ************************************************************************* */
IndexSetMap
IEOptimizer::identifyApproachingIndices(const IEManifoldValues &manifolds,
                                        const IEManifoldValues &newManifolds,
                                        const IndexSetMap &change_indices_map,
                                        const double& approach_rate_threshold) {
  IndexSetMap approach_indices_map;
  for (const auto &it : change_indices_map) {
    const Key &key = it.first;
    const IndexSet &change_indices = it.second;
    const IEConstraintManifold &manifold = manifolds.at(key);
    const IEConstraintManifold &new_manifold = newManifolds.at(key);
    auto i_constraints = manifolds.at(key).iConstraints();

    double max_approach_rate = -1;
    size_t max_approach_idx;
    for (const auto &idx : change_indices) {
      const auto &constraint = i_constraints->at(idx);
      double eval = constraint->unwhitenedExpr(manifold.values())(0);
      double new_eval = constraint->unwhitenedExpr(new_manifold.values())(0);
      // Large positive eval/new_eval means the step is rapidly driving this
      // inactive inequality toward a new active boundary/mode.
      double approach_rate = eval / new_eval;
      // std::cout << "eval: " << eval << "\n";
      // std::cout << "new_eval: " << new_eval << "\n";
      // std::cout << "approach_rate: " << approach_rate << "\n";
      if (approach_rate > approach_rate_threshold && approach_rate > max_approach_rate) {
        max_approach_idx = idx;
        max_approach_rate = approach_rate;
      }
    }
    if (max_approach_rate > 0) {
      IndexSet approach_indices;
      approach_indices.insert(max_approach_idx);
      approach_indices_map.insert({key, approach_indices});
    }
  }

  return approach_indices_map;
}

/* ************************************************************************* */
std::string IEOptimizer::indicesString(const IndexSetMap &indices_map,
                                    const KeyFormatter &keyFormatter) {
  std::string str;
  for (const auto &it : indices_map) {
    if (it.second.size() > 0) {
      str += "(" + keyFormatter(it.first) + ":";
      for (const auto &idx : it.second) {
        str += " " + std::to_string((idx));
      }
      str += ")\t";
    }
  }
  return str;
}

/* ************************************************************************* */
std::string IEOptimizer::indicesString(const IEManifoldValues &manifolds,
                                    const KeyFormatter &keyFormatter) {
  std::string str;
  for (const auto &it : manifolds) {
    if (it.second.activeIndices().size() > 0) {
      str += "(" + keyFormatter(it.first) + ":";
      for (const auto &idx : it.second.activeIndices()) {
        str += " " + std::to_string((idx));
      }
      str += ")\t";
    }
  }
  return str;
}

} // namespace gtdynamics
