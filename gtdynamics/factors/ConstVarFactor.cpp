#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {

/* ************************************************************************* */
KeyVector ConstVarFactor::computeNewKeys(const Base::shared_ptr& base_factor,
                                         const KeySet& fixed_keys) {
  KeyVector new_keys;
  for (const Key& key : base_factor->keys()) {
    if (!fixed_keys.exists(key)) {
      new_keys.push_back(key);
    }
  }
  return new_keys;
}

/* ************************************************************************* */
KeySet ConstVarFactor::computeFixedKeys(const Base::shared_ptr& base_factor,
                                        const KeySet& fixed_keys) {
  KeySet fixed_keys_in_factor;
  for (const Key& key : base_factor->keys()) {
    if (fixed_keys.exists(key)) {
      fixed_keys_in_factor.insert(key);
    }
  }
  return fixed_keys_in_factor;
}

/* ************************************************************************* */
std::vector<size_t> ConstVarFactor::computeBaseKeyIndex(
    const Base::shared_ptr& base_factor, const KeySet& fixed_keys) {
  std::vector<size_t> base_key_index;
  const KeyVector& base_keys = base_factor->keys();
  for (size_t i = 0; i < base_factor->size(); i++) {
    if (!fixed_keys.exists(base_keys.at(i))) {
      base_key_index.emplace_back(i);
    }
  }
  return base_key_index;
}

/* ************************************************************************* */
void ConstVarFactor::setFixedValues(const Values& values) {
  for (const Key& key : values.keys()) {
    if (fixed_keys_.exists(key)) {
      if (fixed_values_.exists(key)) {
        fixed_values_.update(key, values.at(key));
      } else {
        fixed_values_.insert(key, values.at(key));
      }
    }
  }
}

/* ************************************************************************* */
Vector ConstVarFactor::unwhitenedError(const Values& x,
                                       gtsam::OptionalMatrixVecType H) const {
  // Construct values for base factor.
  Values base_x = x;
  base_x.insert(fixed_values_);

  // compute jacobian
  if (H) {
    // Compute Jacobian and error using base factor.
    std::vector<gtsam::Matrix> base_H(base_factor_->size());
    Vector unwhitened_error = base_factor_->unwhitenedError(base_x, base_H);
    // Compute Jacobian for new variables
    for (size_t variable_idx = 0; variable_idx < keys().size();
         variable_idx++) {
      (*H)[variable_idx] = base_H.at(base_key_index_.at(variable_idx));
    }
    return unwhitened_error;
  } else {
    return base_factor_->unwhitenedError(base_x);
  }
}

std::pair<NonlinearFactorGraph, ConstVarFactors> ConstVarGraph(
    const NonlinearFactorGraph& graph, const KeySet& fixed_keys) {
  ConstVarFactors const_var_factors;
  NonlinearFactorGraph new_graph;
  for (const auto& factor : graph) {
    // check if the factor contains fixed keys
    bool contain_fixed_keys = false;
    for (const Key& key : factor->keys()) {
      if (fixed_keys.exists(key)) {
        contain_fixed_keys = true;
        break;
      }
    }
    // update the factor if it constains fixed keys
    if (contain_fixed_keys) {
      NoiseModelFactor::shared_ptr noise_factor =
          std::dynamic_pointer_cast<NoiseModelFactor>(factor);
      auto const_var_factor =
          std::make_shared<ConstVarFactor>(noise_factor, fixed_keys);
      if (const_var_factor->checkActive()) {
        new_graph.add(const_var_factor);
        const_var_factors.push_back(const_var_factor);
      }
    } else {
      new_graph.add(factor);
    }
  }
  return std::make_pair(new_graph, const_var_factors);
}

NonlinearFactorGraph ConstVarGraph(const NonlinearFactorGraph& graph,
                                   const Values& fixed_values) {
  ConstVarFactors const_var_factors;
  NonlinearFactorGraph new_graph;
  KeyVector key_vec = fixed_values.keys();
  KeySet key_set(key_vec.begin(), key_vec.end());
  std::tie(new_graph, const_var_factors) = ConstVarGraph(graph, key_set);

  for (auto& factor : const_var_factors) {
    factor->setFixedValues(fixed_values);
  }
  return new_graph;
}

}  // namespace gtdynamics
