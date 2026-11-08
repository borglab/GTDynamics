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

#include <gtdynamics/constrained/CMCSubstituteFactor.h>
#include <utils/DynamicsSymbol.h>
#include <gtsam/inference/Symbol.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>
#include <stack>

namespace gtdynamics {
using namespace gtsam;


/* ************************************************************************* */
std::map<Key, Key>
IEOptimizer::varToManifoldKeyMap(const IECMPoints &points) {
  std::map<Key, Key> ambientToPoint;
  for (const auto &it : points) {
    for (const Key &variable_key : it.second.values().keys()) {
      ambientToPoint.emplace(variable_key, it.first);
    }
  }
  return ambientToPoint;
}

NonlinearFactorGraph IEOptimizer::pointGraph(const NonlinearFactorGraph &graph,
                                             const IECMPoints &points) {
  Values fixedPoints;
  for (const auto& [key, point] : points)
    if (!point.dim()) fixedPoints.insert(key, point);
  return pointSubstitutionGraph(graph, varToManifoldKeyMap(points), fixedPoints,
                                CMCSubstituteFactor::PointType::Inequality);
}
void IEOptimizer::validatePoints(const IECMPoints &points,
                                 const Values &unconstrained) {
  KeySet ambient(unconstrained.keys());
  for (const auto &[key, point] : points) {
    if (!point.manifold()) throw std::invalid_argument("Empty IECMPoint");
    for (Key slot : point.values().keys())
      if (!ambient.insert(slot).second)
        throw std::invalid_argument("Overlapping IECMPoint ambient ownership");
  }
  for (const auto &[key, point] : points)
    if (ambient.count(key))
      throw std::invalid_argument(
          "Component point key collides with ambient key");
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
IEOptimizer::Components IEOptimizer::identifyConnectedComponents(
    const NonlinearEqualityConstraints &e_constraints,
    const NonlinearInequalityConstraints &i_constraints,
    const Values &values,
    const IEConstraintManifold::Params::shared_ptr &iecm_params,
    const TypeFactory& typeFactory, const ValueInitializer& initialize) {
  if (!iecm_params) throw std::invalid_argument("Missing CMC type parameters");
  VariableIndex e_var_index(e_constraints);
  VariableIndex i_var_index(i_constraints);

  // Only constraint factors determine connectivity; costs may couple points.
  KeySet keys = e_constraints.keys();
  keys.merge(i_constraints.keys());
  Components result;
  while (!keys.empty()) {
    auto component_info = identifyConnectedComponent(
        e_constraints, i_constraints, e_var_index, i_var_index, *keys.begin());
    for (const Key &key : component_info.keys) keys.erase(key);

    NonlinearEqualityConstraints equalities;
    for (const auto &idx : component_info.e_indices)
      equalities.push_back(e_constraints.at(idx));
    NonlinearInequalityConstraints inequalities;
    for (const auto &idx : component_info.i_indices)
      inequalities.push_back(i_constraints.at(idx));

    Values componentValues;
    for (const Key &key : component_info.keys)
      componentValues.insert(key, values.at(key));
    if (initialize) componentValues = initialize(equalities, componentValues);
    const auto type = typeFactory
        ? typeFactory(equalities, inequalities, componentValues, iecm_params)
        : std::make_shared<IEConstraintManifold>(equalities, inequalities,
                                                componentValues, *iecm_params);

    // Point keys are independent of both ambient keys and the type collection.
    size_t index = result.points.size();
    Key pointKey = Symbol('c', index);
    while (values.exists(pointKey) || result.points.count(pointKey))
      pointKey = Symbol('c', ++index);
    result.types.push_back(type);
    result.points.emplace(pointKey, type->point(componentValues));
  }
  return result;
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
IEOptimizer::computeTangentVector(const IECMPoints &points,
                                  const VectorValues &delta) {
  VectorValues tangentVector;
  for (const auto &it : points) {
    const Key &key = it.first;
    const Vector &xi = delta.at(key);
    VectorValues tv = points.at(key).toAmbient(xi);
    tangentVector.insert(tv);
  }
  return tangentVector;
}

/* ************************************************************************* */
std::pair<IndexSetMap, VectorValues>
IEOptimizer::projectTangentCone(const IECMPoints &points,
                                const VectorValues &v) {
  VectorValues proj_v;
  IndexSetMap active_indices_all;
  for (const auto &it : points) {
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
IECMPoints
IEOptimizer::retractPoints(const IECMPoints &points,
                              const VectorValues &delta) {
  IECMPoints newPoints;
  for (const auto &it : points) {
    const Key &key = it.first;
    const Vector &xi = delta.at(key);
    newPoints.emplace(key, it.second.retract(xi));
  }
  return newPoints;
}

/* ************************************************************************* */
bool IEOptimizer::isSameMode(const IECMPoints &points1,
                             const IECMPoints &points2) {
  for (const auto &it : points1) {
    const IndexSet &indices1 = it.second.activeIndices();
    const IndexSet &indices2 = points2.at(it.first).activeIndices();
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
IEOptimizer::identifyChangeIndices(const IECMPoints &points,
                                   const IECMPoints &newPoints) {
  IndexSetMap change_indices_map;
  for (const auto &it : points) {
    const Key &key = it.first;
    const IndexSet &indices = it.second.activeIndices();
    const IndexSet &new_indices = newPoints.at(key).activeIndices();
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
IEOptimizer::identifyApproachingIndices(const IECMPoints &points,
                                        const IECMPoints &newPoints,
                                        const IndexSetMap &change_indices_map,
                                        const double& approach_rate_threshold) {
  IndexSetMap approach_indices_map;
  for (const auto &it : change_indices_map) {
    const Key &key = it.first;
    const IndexSet &change_indices = it.second;
    const IECMPoint &point = points.at(key);
    const IECMPoint &newPoint = newPoints.at(key);

    double max_approach_rate = -1;
    size_t max_approach_idx;
    for (const auto &idx : change_indices) {
      double eval = point.inequalityResidual(idx)(0);
      double new_eval = newPoint.inequalityResidual(idx)(0);
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
std::string IEOptimizer::indicesString(const IECMPoints &points,
                                    const KeyFormatter &keyFormatter) {
  std::string str;
  for (const auto &it : points) {
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
