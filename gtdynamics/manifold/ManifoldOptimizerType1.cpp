/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ManifoldOptimizerType1.cpp
 * @brief Manifold optimizer implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/manifold/ManifoldOptimizerType1.h>
#include <gtdynamics/manifold/SubstituteFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "manifold/Retractor.h"
#include "manifold/TspaceBasis.h"

namespace gtsam {

/* ************************************************************************* */
Values ManifoldOptimizerType1::optimize(
    const NonlinearFactorGraph& costs,
    const gtdynamics::EqualityConstraints& constraints,
    const Values& init_values,
    gtdynamics::ConstrainedOptResult* intermediate_result) const {
  auto mopt_problem = initializeMoptProblem(costs, constraints, init_values);
  return optimize(mopt_problem, intermediate_result);
}

/* ************************************************************************* */
Values ManifoldOptimizerType1::optimize(
    const ManifoldOptProblem& mopt_problem,
    gtdynamics::ConstrainedOptResult* intermediate_result) const {
  auto nonlinear_optimizer = constructNonlinearOptimizer(mopt_problem);
  auto nopt_values = nonlinear_optimizer->optimize();
  if (intermediate_result) {
    intermediate_result->num_iters.push_back(
        std::dynamic_pointer_cast<LevenbergMarquardtOptimizer>(
            nonlinear_optimizer)
            ->getInnerIterations());
  }
  return baseValues(mopt_problem, nopt_values);
}

/* ************************************************************************* */
Values ManifoldOptimizerType1::baseValues(
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
ManifoldOptProblem ManifoldOptimizerType1::initializeMoptProblem(
    const gtsam::NonlinearFactorGraph& costs,
    const gtdynamics::EqualityConstraints& constraints,
    const gtsam::Values& init_values) const {
  EConsOptProblem ecopt_problem(costs, constraints, init_values);
  return problemTransform(ecopt_problem);
}

/* ************************************************************************* */
ManifoldOptProblem ManifoldOptimizerType1::problemTransform(
    const EConsOptProblem& ecopt_problem) const {
  ManifoldOptProblem mopt_problem;
  mopt_problem.components_ =
      identifyConnectedComponents(ecopt_problem.constraints());
  constructMoptValues(ecopt_problem, mopt_problem);
  constructMoptGraph(ecopt_problem, mopt_problem);
  return mopt_problem;
}

/* ************************************************************************* */
void ManifoldOptimizerType1::constructMoptValues(
    const EConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) const {
  constructManifoldValues(ecopt_problem, mopt_problem);
  constructUnconstrainedValues(ecopt_problem, mopt_problem);
}

/* ************************************************************************* */
void ManifoldOptimizerType1::constructManifoldValues(
    const EConsOptProblem& ecopt_problem,
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
void ManifoldOptimizerType1::constructUnconstrainedValues(
    const EConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) const {
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
void ManifoldOptimizerType1::constructMoptGraph(
    const EConsOptProblem& ecopt_problem,
    ManifoldOptProblem& mopt_problem) const {
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
std::shared_ptr<NonlinearOptimizer>
ManifoldOptimizerType1::constructNonlinearOptimizer(
    const ManifoldOptProblem& mopt_problem) const {
  if (std::holds_alternative<GaussNewtonParams>(nopt_params_)) {
    return std::make_shared<GaussNewtonOptimizer>(
        mopt_problem.graph_, mopt_problem.values_,
        std::get<GaussNewtonParams>(nopt_params_));
  } else if (std::holds_alternative<LevenbergMarquardtParams>(nopt_params_)) {
    return std::make_shared<LevenbergMarquardtOptimizer>(
        mopt_problem.graph_, mopt_problem.values_,
        std::get<LevenbergMarquardtParams>(nopt_params_));
  } else if (std::holds_alternative<DoglegParams>(nopt_params_)) {
    return std::make_shared<DoglegOptimizer>(
        mopt_problem.graph_, mopt_problem.values_,
        std::get<DoglegParams>(nopt_params_));
  } else {
    return std::make_shared<LevenbergMarquardtOptimizer>(mopt_problem.graph_,
                                                         mopt_problem.values_);
  }
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

}  // namespace gtsam
