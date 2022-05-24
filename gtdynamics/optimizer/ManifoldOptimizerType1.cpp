#include <gtdynamics/optimizer/ManifoldOptimizerType1.h>

#include <gtdynamics/optimizer/ConstraintManifold.h>
#include <gtdynamics/optimizer/SubstituteFactor.h>

namespace gtsam {

/* ************************************************************************* */
const gtsam::Values& ManifoldOptimizerType1::optimize() {
  nonlinear_optimizer_->optimize();
  return base_values();
}

/* ************************************************************************* */
const Values& ManifoldOptimizerType1::base_values() {
  base_values_ = Values();
  for (const Key& key : fc_manifolds_.keys()) {
    auto constraint_manifold = fc_manifolds_.at<ConstraintManifold>(key);
    base_values_.insert(constraint_manifold.values());
  }
  const Values& values = nonlinear_optimizer_->values();
  for (const Key& key : unconstrained_keys_) {
    base_values_.insert(key, values.at(key));
  }
  for (const Key& key : component_key_vec_) {
    base_values_.insert(values.at<ConstraintManifold>(key).values());
  }
  return base_values_;
}

/* ************************************************************************* */
Values ManifoldOptimizerType1::construct_manifold_values(
    const Values& init_values) {
  Values manifold_values;
  component_key_vec_ = KeyVector();

  // Construct values for constraint manifold.
  for (size_t i = 0; i < components_.size(); i++) {
    const auto& component = components_.at(i);
    const Key& component_key = *component->keys.begin();
    Values component_values;
    for (const Key& key : component->keys) {
      component_values.insert(key, init_values.at(key));
    }
    KeyVector basis_keys;
    if (params_->cc_params->retract_type ==
            ConstraintManifold::Params::RetractType::PARTIAL_PROJ ||
        params_->cc_params->basis_type ==
            ConstraintManifold::Params::BasisType::SPECIFY_VARIABLES) {
      basis_keys = (*basis_key_func_)(component);
    }
    auto constraint_manifold =
        ConstraintManifold(component, component_values, params_->cc_params,
                           params_->retract_init, true, basis_keys);
    if (constraint_manifold.dim() > 0) {
      component_key_vec_.push_back(component_key);
      manifold_values.insert(component_key, constraint_manifold);
    } else {
      fc_manifolds_.insert(component_key, constraint_manifold);
    }
  }

  // Add values for unconstrained keys.
  KeySet unconstrained_keys_ = costs_.keys();
  for (const auto& component : components_) {
    for (const Key& key : component->keys) {
      if (unconstrained_keys_.find(key) != unconstrained_keys_.end()) {
        unconstrained_keys_.erase(key);
      }
    }
  }
  for (const Key& key : unconstrained_keys_) {
    manifold_values.insert(key, init_values.at(key));
  }
  return manifold_values;
}

/* ************************************************************************* */
NonlinearFactorGraph ManifoldOptimizerType1::construct_manifold_graph(
    const Values& manifold_values) {
  // Construct base key to component map.
  std::map<Key, Key> key_component_map;
  for (const Key& cm_key : component_key_vec_) {
    auto cm = manifold_values.at<ConstraintManifold>(cm_key);
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }
  for (const Key& cm_key : fc_manifolds_.keys()) {
    auto cm = fc_manifolds_.at<ConstraintManifold>(cm_key);
    for (const Key& base_key : cm.values().keys()) {
      key_component_map[base_key] = cm_key;
    }
  }

  // Turn factors involved with constraint varibles into SubstituteFactor.
  NonlinearFactorGraph manifold_graph;
  for (const auto& factor : costs_) {
    std::map<Key, Key> replacement_map;
    for (const Key& key : factor->keys()) {
      if (key_component_map.find(key) != key_component_map.end()) {
        replacement_map[key] = key_component_map.at(key);
      }
    }
    if (replacement_map.size() > 0) {
      NoiseModelFactor::shared_ptr noise_factor =
          boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto subs_factor = boost::make_shared<SubstituteFactor>(
          noise_factor, replacement_map, fc_manifolds_);
      if (subs_factor->check_active()) {
        manifold_graph.add(subs_factor);
      }
    } else {
      manifold_graph.add(factor);
    }
  }

  return manifold_graph;
}

/* ************************************************************************* */
void ManifoldOptimizerType1::construct_nonlinear_optimizer(
    const Values& init_values, const NonlinearOptParamsVariant& params) {
  Values manifold_values = construct_manifold_values(init_values);
  NonlinearFactorGraph manifold_graph =
      construct_manifold_graph(manifold_values);
  if (params.type() == typeid(GaussNewtonParams)) {
    nonlinear_optimizer_ = boost::make_shared<GaussNewtonOptimizer>(
        manifold_graph, manifold_values, boost::get<GaussNewtonParams>(params));
  } else if (params.type() == typeid(LevenbergMarquardtParams)) {
    nonlinear_optimizer_ = boost::make_shared<LevenbergMarquardtOptimizer>(
        manifold_graph, manifold_values,
        boost::get<LevenbergMarquardtParams>(params));
  } else if (params.type() == typeid(DoglegParams)) {
    nonlinear_optimizer_ = boost::make_shared<DoglegOptimizer>(
        manifold_graph, manifold_values, boost::get<DoglegParams>(params));
  }
}

/* ************************************************************************* */
int ManifoldOptimizerType1::getInnerIterations() const {
  if (const auto lm_pointer =
          boost::dynamic_pointer_cast<LevenbergMarquardtOptimizer>(
              nonlinear_optimizer_)) {
    return lm_pointer->getInnerIterations();
  } else {
    return -1;
  }
}

/* ************************************************************************* */
std::pair<size_t, size_t> ManifoldOptimizerType1::problem_dimension() const {
  size_t values_dim = nonlinear_optimizer_->values().dim();
  size_t graph_dim = 0;
  for (const auto& factor : nonlinear_optimizer_->graph()) {
    graph_dim += factor->dim();
  }
  return std::make_pair(graph_dim, values_dim);
}

/* ************************************************************************* */
void ManifoldOptimizerType1::print(const std::string& s,
                                   const KeyFormatter& keyFormatter) const {
  std::cout << s;
  std::cout << "found " << components_.size()
            << " components: " << component_key_vec_.size() << " free, "
            << fc_manifolds_.size() << " fixed\n";
  size_t i = 0;
  for (size_t i = 0; i < component_key_vec_.size(); i++) {
    Key cm_key = component_key_vec_.at(i);
    auto cm = nonlinear_optimizer_->values().at<ConstraintManifold>(cm_key);
    std::cout << "component " << i << ":\tdimension " << cm.dim() << "\n";
    for (const auto& key : cm.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
  std::cout << "fully constrained manifolds:\n";
  for (const auto& cm_key : fc_manifolds_.keys()) {
    auto cm = fc_manifolds_.at<ConstraintManifold>(cm_key);
    for (const auto& key : cm.values().keys()) {
      std::cout << "\t" << keyFormatter(key);
    }
    std::cout << "\n";
  }
}

}  // namespace gtsam
