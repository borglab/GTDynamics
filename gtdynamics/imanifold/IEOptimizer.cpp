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

#include "utils/DynamicsSymbol.h"
#include <gtdynamics/imanifold/IEOptimizer.h>

namespace gtsam {

/* ************************************************************************* */
std::map<Key, Key>
IEOptimizer::Var2ManifoldKeyMap(const IEManifoldValues &manifolds) {
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
ComponentInfo IdentifyConnectedComponent(
    const gtdynamics::EqualityConstraints &e_constraints,
    const gtdynamics::InequalityConstraints &i_constraints,
    const gtsam::VariableIndex &e_var_index,
    const gtsam::VariableIndex &i_var_index, const gtsam::Key start_key) {
  ComponentInfo component_info;

  std::stack<gtsam::Key> key_stack;
  key_stack.push(start_key);
  while (!key_stack.empty()) {
    gtsam::Key key = key_stack.top();
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
std::vector<std::pair<gtdynamics::EqualityConstraints::shared_ptr,
                      gtdynamics::InequalityConstraints::shared_ptr>>
IdentifyConnectedComponents(
    const gtdynamics::EqualityConstraints &e_constraints,
    const gtdynamics::InequalityConstraints &i_constraints) {

  gtsam::VariableIndex e_var_index = e_constraints.varIndex();
  gtsam::VariableIndex i_var_index = i_constraints.varIndex();

  // Get all the keys in constraints.
  KeySet keys = e_constraints.keys();
  keys.merge(i_constraints.keys());

  std::vector<std::pair<gtdynamics::EqualityConstraints::shared_ptr,
                        gtdynamics::InequalityConstraints::shared_ptr>>
      components;
  while (!keys.empty()) {
    auto component_info = IdentifyConnectedComponent(
        e_constraints, i_constraints, e_var_index, i_var_index, *keys.begin());
    for (const Key &key : component_info.keys) {
      keys.erase(key);
    }

    // e_constraints
    auto component_e_constraints =
        std::make_shared<gtdynamics::EqualityConstraints>();
    for (const auto &idx : component_info.e_indices) {
      component_e_constraints->emplace_back(e_constraints.at(idx));
    }
    // i_constraints
    auto component_i_constraints =
        std::make_shared<gtdynamics::InequalityConstraints>();
    for (const auto &idx : component_info.i_indices) {
      component_i_constraints->emplace_back(i_constraints.at(idx));
    }

    components.emplace_back(component_e_constraints, component_i_constraints);
  }
  return components;
}

/* ************************************************************************* */
IEManifoldValues IEOptimizer::IdentifyManifolds(
    const gtdynamics::EqualityConstraints &e_constraints,
    const gtdynamics::InequalityConstraints &i_constraints,
    const gtsam::Values &values,
    const IEConstraintManifold::Params::shared_ptr &iecm_params) {
  // find connected components by equality constraints
  auto components = IdentifyConnectedComponents(e_constraints, i_constraints);

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
Values IEOptimizer::IdentifyUnconstrainedValues(
      const gtdynamics::EqualityConstraints &e_constraints,
      const gtdynamics::InequalityConstraints &i_constraints,
      const gtsam::Values &values) {
  Values unconstrained_values;
  KeySet constrained_keys = e_constraints.keys();
  constrained_keys.merge(i_constraints.keys());
  for (const Key& key: values.keys()) {
    if (!constrained_keys.exists(key)) {
      unconstrained_values.insert(key, values.at(key));
    }
  }
  return unconstrained_values;
}

/* ************************************************************************* */
VectorValues
IEOptimizer::ComputeTangentVector(const IEManifoldValues &manifolds,
                                  const VectorValues &delta) {
  VectorValues tangent_vector;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const Vector &xi = delta.at(key);
    VectorValues tv = manifolds.at(key).eBasis()->computeTangentVector(xi);
    tangent_vector.insert(tv);
  }
  return tangent_vector;
}

/* ************************************************************************* */
std::pair<IndexSetMap, VectorValues>
IEOptimizer::ProjectTangentCone(const IEManifoldValues &manifolds,
                                const VectorValues &v) {
  VectorValues proj_v;
  IndexSetMap active_indices_all;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const Vector &xi = v.at(key);
    IndexSet active_indices;
    Vector proj_xi;
    std::tie(active_indices, proj_xi) = it.second.projectTangentCone(xi);
    proj_v.insert(key, proj_xi);
    active_indices_all.emplace(key, active_indices);
  }
  return std::make_pair(active_indices_all, proj_v);
}

/* ************************************************************************* */
IEManifoldValues
IEOptimizer::RetractManifolds(const IEManifoldValues &manifolds,
                              const VectorValues &delta) {
  IEManifoldValues new_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const Vector &xi = delta.at(key);
    new_manifolds.emplace(key, it.second.retract(xi));
  }
  return new_manifolds;
}

/* ************************************************************************* */
Values IEOptimizer::EManifolds(const IEManifoldValues &manifolds) {
  Values e_manifolds;
  for (const auto &it : manifolds) {
    e_manifolds.insert(it.first, it.second.eConstraintManifold());
  }
  return e_manifolds;
}

/* ************************************************************************* */
std::pair<Values, Values>
IEOptimizer::EManifolds(const IEManifoldValues &manifolds,
                        const IndexSetMap &active_indices) {
  Values e_manifolds;
  Values const_e_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    ConstraintManifold e_manifold =
        it.second.eConstraintManifold(active_indices.at(key));
    if (e_manifold.dim() > 0) {
      e_manifolds.insert(key, e_manifold);
    } else {
      const_e_manifolds.insert(key, e_manifold);
    }
  }
  return std::make_pair(e_manifolds, const_e_manifolds);
}

/* ************************************************************************* */
bool IEOptimizer::IsSameMode(const IEManifoldValues &manifolds1,
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
IEOptimizer::IdentifyChangeIndices(const IEManifoldValues &manifolds,
                                   const IEManifoldValues &new_manifolds) {
  IndexSetMap change_indices_map;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const IndexSet &indices = it.second.activeIndices();
    const IndexSet &new_indices = new_manifolds.at(key).activeIndices();
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
IEOptimizer::IdentifyApproachingIndices(const IEManifoldValues &manifolds,
                                        const IEManifoldValues &new_manifolds,
                                        const IndexSetMap &change_indices_map,
                                        const double& approach_rate_threshold) {
  IndexSetMap approach_indices_map;
  for (const auto &it : change_indices_map) {
    const Key &key = it.first;
    const IndexSet &change_indices = it.second;
    const IEConstraintManifold &manifold = manifolds.at(key);
    const IEConstraintManifold &new_manifold = new_manifolds.at(key);
    auto i_constraints = manifolds.at(key).iConstraints();

    double max_approach_rate = -1;
    size_t max_approach_idx;
    for (const auto &idx : change_indices) {
      const auto &constraint = i_constraints->at(idx);
      double eval = (*constraint)(manifold.values());
      double new_eval = (*constraint)(new_manifold.values());
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
std::string IEOptimizer::IndicesStr(const IndexSetMap &indices_map,
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
std::string IEOptimizer::IndicesStr(const IEManifoldValues &manifolds,
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

} // namespace gtsam
