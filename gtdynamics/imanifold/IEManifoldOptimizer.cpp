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

#include "imanifold/IERetractor.h"
#include <gtdynamics/imanifold/IEManifoldOptimizer.h>

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

/* ************************************************************************* */
IEManifoldValues IEOptimizer::IdentifyManifolds(
    const gtdynamics::EqualityConstraints &e_constraints,
    const gtdynamics::InequalityConstraints &i_constraints,
    const gtsam::Values &values,
    const IEConstraintManifold::Params::shared_ptr &iecm_params) {
  // find connected components by equality constraints
  auto components =
      ManifoldOptimizer::IdentifyConnectedComponents(e_constraints);

  // construct variable key to manifold key map
  std::map<Key, Key> var2manifold_keymap;
  for (const auto &component : components) {
    Key component_key = *component->keys_.begin();
    for (const Key &key : component->keys_) {
      var2manifold_keymap.emplace(key, component_key);
    }
  }

  // assign inequality constraints to components
  std::map<Key, gtdynamics::InequalityConstraints::shared_ptr> cc_i_constraints;
  for (const auto &component : components) {
    Key component_key = *component->keys_.begin();
    cc_i_constraints.emplace(
        component_key, std::make_shared<gtdynamics::InequalityConstraints>());
  }
  for (const auto &constraint : i_constraints) {
    KeySet component_keys;
    for (const Key &key : constraint->keys()) {
      component_keys.emplace(var2manifold_keymap.at(key));
    }
    if (component_keys.size() > 1) {
      throw std::runtime_error(
          "i_constraint connect to more than 1 component.\n");
    }
    Key component_key = *component_keys.begin();
    cc_i_constraints.at(component_key)->emplace_back(constraint);
  }

  // create ie_manifolds
  IEManifoldValues ie_manifolds;
  for (const auto &component : components) {
    Key component_key = *component->keys_.begin();
    Values component_values;
    for (const Key &key : component->keys_) {
      component_values.insert(key, values.at(key));
    }
    ie_manifolds.emplace(
        component_key, IEConstraintManifold(iecm_params, component,
                                            cc_i_constraints.at(component_key),
                                            component_values));
  }
  return ie_manifolds;
}

/* ************************************************************************* */
Values IEOptimizer::CollectManifoldValues(const IEManifoldValues &manifolds) {
  Values values;
  for (const auto &it : manifolds) {
    values.insert(it.second.values());
  }
  return values;
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
                                        const IndexSetMap &change_indices_map) {
  IndexSetMap approach_indices_map;
  for (const auto &it : change_indices_map) {
    const Key &key = it.first;
    const IndexSet &change_indices = it.second;
    const IEConstraintManifold &manifold = manifolds.at(key);
    const IEConstraintManifold &new_manifold = new_manifolds.at(key);
    auto i_constraints = manifolds.at(key).iConstraints();
    IndexSet approach_indices;

    for (const auto &idx : change_indices) {
      const auto &constraint = i_constraints->at(idx);
      double eval = (*constraint)(manifold.values());
      double new_eval = (*constraint)(new_manifold.values());
      double approach_rate = eval / new_eval;
      std::cout << "eval: " << eval << "\n";
      std::cout << "new_eval: " << new_eval << "\n";
      std::cout << "approach_rate: " << approach_rate << "\n";
      if (approach_rate > 3) {
        approach_indices.insert(idx);
      }
    }
    if (approach_indices.size() > 0) {
      approach_indices_map.insert({key, change_indices});
    }
  }
  return approach_indices_map;
}

/* ************************************************************************* */
IEManifoldValues
IEOptimizer::MoveToBoundaries(const IEManifoldValues &manifolds,
                              const IndexSetMap &approach_indices_map) {
  IEManifoldValues new_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const auto &manifold = it.second;
    if (approach_indices_map.find(key) == approach_indices_map.end()) {
      new_manifolds.insert({key, manifold});
    } else {
      new_manifolds.insert(
          {key, manifold.moveToBoundary(approach_indices_map.at(key))});
    }
  }
  return new_manifolds;
}

} // namespace gtsam
