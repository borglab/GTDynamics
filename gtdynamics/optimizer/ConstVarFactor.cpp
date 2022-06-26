#include <gtdynamics/optimizer/ConstVarFactor.h>

namespace gtsam {

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
Vector ConstVarFactor::unwhitenedError(
    const Values& x, boost::optional<std::vector<Matrix>&> H) const {
  // Construct values for base factor.
  Values base_x = x;
  base_x.insert(fixed_values_);

  // compute jacobian
  if (H) {
    // Compute Jacobian and error using base factor.
    std::vector<Matrix> base_H(base_factor_->size());
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

}  // namespace gtsam
