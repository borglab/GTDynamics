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
#include <gtdynamics/utils/GraphUtils.h>

namespace gtsam {
/* ************************************************************************* */
Values ManifoldOptProblem::unconstrainedValues() const {
  return SubValues(values_, unconstrained_keys_);
}

/* ************************************************************************* */
EManifoldValues ManifoldOptProblem::manifolds() const {
  EManifoldValues e_manifolds;
  for (const Key &key : manifold_keys_) {
    e_manifolds.insert({key, values_.at<ConstraintManifold>(key)});
  }
  return e_manifolds;
}

/* ************************************************************************* */
EManifoldValues ManifoldOptProblem::constManifolds() const {
  EManifoldValues e_manifolds;
  for (const Key &key : fixed_manifolds_.keys()) {
    e_manifolds.insert({key, fixed_manifolds_.at<ConstraintManifold>(key)});
  }
  return e_manifolds;
}

/* ************************************************************************* */
std::pair<size_t, size_t> ManifoldOptProblem::problemDimension() const {
  size_t values_dim = values_.dim();
  size_t graph_dim = 0;
  for (const auto& factor : graph_) {
    graph_dim += factor->dim();
  }
  return std::make_pair(graph_dim, values_dim);
}

/* ************************************************************************* */
void ManifoldOptProblem::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "found " << components_.size()
            << " components: " << manifold_keys_.size() << " free, "
            << fixed_manifolds_.size() << " fixed\n";
  for (const Key& cm_key : manifold_keys_) {
    const auto& cm = values_.at(cm_key).cast<ConstraintManifold>();
    std::cout << "component " << keyFormatter(cm_key) << ":\tdimension "
              << cm.dim() << "\n";
    for (const auto& key : cm.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
  std::cout << "fully constrained manifolds:\n";
  for (const auto& cm_key : fixed_manifolds_.keys()) {
    const auto& cm = fixed_manifolds_.at(cm_key).cast<ConstraintManifold>();
    for (const auto& key : cm.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
}

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
  auto constraint_var_index = constraints.varIndex();
  gtsam::KeySet constraint_keys = constraints.keys();

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

/* ************************************************************************* */
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

/* ************************************************************************* */
ManifoldOptProblem ManifoldOptimizer::problemTransform(
    const EqConsOptProblem& ecopt_problem) const {
  ManifoldOptProblem mopt_problem;
  mopt_problem.components_ =
      IdentifyConnectedComponents(ecopt_problem.constraints_);
  constructMoptValues(ecopt_problem, mopt_problem);
  constructMoptGraph(ecopt_problem, mopt_problem);
  return mopt_problem;
}

/* ************************************************************************* */
void ManifoldOptimizer::constructMoptValues(
    const EqConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) const {
  constructManifoldValues(ecopt_problem, mopt_problem);
  constructUnconstrainedValues(ecopt_problem, mopt_problem);
}

/* ************************************************************************* */
void ManifoldOptimizer::constructManifoldValues(
    const EqConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) const {
  for (size_t i = 0; i < mopt_problem.components_.size(); i++) {
    // Find the values of variables in the component.
    const auto& component = mopt_problem.components_.at(i);
    const Key& component_key = *component->keys_.begin();
    Values component_values;
    for (const Key& key : component->keys_) {
      component_values.insert(key, ecopt_problem.values_.at(key));
    }
    // Construct manifold value
    auto constraint_manifold = ConstraintManifold(
        component, component_values, p_.cc_params, p_.retract_init);
    // check if the manifold is fully constrained
    if (constraint_manifold.dim() > 0) {
      mopt_problem.values_.insert(component_key, constraint_manifold);
      mopt_problem.manifold_keys_.insert(component_key);
    } else {
      mopt_problem.fixed_manifolds_.insert(component_key, constraint_manifold);
    }
  }
}

/* ************************************************************************* */
void ManifoldOptimizer::constructUnconstrainedValues(
    const EqConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) {
  // Find out which variables are unconstrained
  mopt_problem.unconstrained_keys_ = ecopt_problem.costs_.keys();
  KeySet& unconstrained_keys = mopt_problem.unconstrained_keys_;
  for (const auto& component : mopt_problem.components_) {
    for (const Key& key : component->keys_) {
      if (unconstrained_keys.find(key) != unconstrained_keys.end()) {
        unconstrained_keys.erase(key);
      }
    }
  }
  // Copy the values of unconstrained variables
  for (const Key& key : unconstrained_keys) {
    mopt_problem.values_.insert(key, ecopt_problem.values_.at(key));
  }
}

/* ************************************************************************* */
void ManifoldOptimizer::constructMoptGraph(
    const EqConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) {
  // Construct base key to component map.
  std::map<Key, Key> key_component_map;
  for (const Key& cm_key : mopt_problem.manifold_keys_) {
    const auto& cm = mopt_problem.values_.at(cm_key).cast<ConstraintManifold>();
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }
  for (const Key& cm_key : mopt_problem.fixed_manifolds_.keys()) {
    const auto& cm =
        mopt_problem.fixed_manifolds_.at(cm_key).cast<ConstraintManifold>();
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }

  // Turn factors involved with constraint variables into SubstituteFactor.
  for (const auto& factor : ecopt_problem.costs_) {
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
          noise_factor, replacement_map, mopt_problem.fixed_manifolds_);
      if (subs_factor->checkActive()) {
        mopt_problem.graph_.add(subs_factor);
      }
    } else {
      mopt_problem.graph_.add(factor);
    }
  }
}


/* ************************************************************************* */
Values ManifoldOptimizer::baseValues(
    const ManifoldOptProblem& mopt_problem, const Values& nopt_values) const {
  Values base_values;
  for (const Key& key : mopt_problem.fixed_manifolds_.keys()) {
    base_values.insert(mopt_problem.fixed_manifolds_.at(key)
                           .cast<ConstraintManifold>()
                           .values());
  }
  for (const Key& key : mopt_problem.unconstrained_keys_) {
    base_values.insert(key, nopt_values.at(key));
  }
  for (const Key& key : mopt_problem.manifold_keys_) {
    if (p_.retract_final) {
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
VectorValues
ManifoldOptimizer::baseTangentVector(const ManifoldOptProblem &mopt_problem,
                                     const Values &values,
                                     const VectorValues &delta) const {
  VectorValues tangent_vector;
  for (const Key &key : mopt_problem.unconstrained_keys_) {
    tangent_vector.insert(key, delta.at(key));
  }
  for (const Key &key : mopt_problem.manifold_keys_) {
    tangent_vector.insert(
        values.at(key).cast<ConstraintManifold>().basis()->computeTangentVector(
            delta.at(key)));
  }
  return tangent_vector;
}

/* ************************************************************************* */
ManifoldOptProblem ManifoldOptimizer::initializeMoptProblem(
    const gtsam::NonlinearFactorGraph& costs,
    const gtdynamics::EqualityConstraints& constraints,
    const gtsam::Values& init_values) const {
  EqConsOptProblem ecopt_problem(costs, constraints, init_values);
  return problemTransform(ecopt_problem);
}

}  // namespace gtsam
