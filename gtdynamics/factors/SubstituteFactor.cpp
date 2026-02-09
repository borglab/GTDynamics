/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SubstituteFactor.cpp
 * @brief Substitute factor implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/cmopt/ConstraintManifold.h>
#include <gtdynamics/factors/SubstituteFactor.h>

namespace gtdynamics {

/* ************************************************************************* */
KeyVector SubstituteFactor::computeNewKeys(
    const Base::shared_ptr& base_factor,
    const std::map<Key, Key>& replacement_map, const Values& fc_manifolds) {
  const KeyVector& base_keys = base_factor->keys();
  KeySet cmanifold_keys;
  KeyVector new_keys;
  for (const Key& base_key : base_keys) {
    if (replacement_map.find(base_key) != replacement_map.end()) {
      Key new_key = replacement_map.at(base_key);
      if (!fc_manifolds.exists(new_key)) {
        if (!cmanifold_keys.exists(new_key)) {
          new_keys.push_back(new_key);
          cmanifold_keys.insert(new_key);
        }
      }
    } else {
      new_keys.push_back(base_key);
    }
  }
  return new_keys;
}

/* ************************************************************************* */
void SubstituteFactor::computeBaseKeyIndex() {
  const KeyVector& base_keys = base_factor_->keys();
  for (size_t key_index = 0; key_index < base_keys.size(); key_index++) {
    const Key& base_key = base_keys.at(key_index);
    base_key_index_[base_key] = key_index;
  }
}

/* ************************************************************************* */
void SubstituteFactor::classifyKeys(const Values& fc_manifolds) {
  const KeyVector& base_keys = base_factor_->keys();
  for (const Key& base_key : base_keys) {
    if (isReplaced(base_key)) {
      Key new_key = replacement_map_.at(base_key);
      if (fc_manifolds.exists(new_key)) {
        fc_values_.insert(
            base_key,
            fc_manifolds.at(new_key).cast<ConstraintManifold>().recover(base_key));
      } else {
        cmanifold_keys_.insert(new_key);
      }
    } else {
      unconstrained_keys_.insert(base_key);
    }
  }
}

/* ************************************************************************* */
/** Insert values (for key in keys) to output_values */
void InsertSelected(const Values& values, const KeyVector& keys,
                    Values& output_values) {
  for (const Key& key : keys) {
    if (values.exists(key)) {
      output_values.insert(key, values.at(key));
    }
  }
}

/* ************************************************************************* */
Vector SubstituteFactor::unwhitenedError(
    const Values& x, gtsam::OptionalMatrixVecType H) const {
  // Construct values for base factor.
  Values base_x = fc_values_;
  for (const Key& key : unconstrained_keys_) {
    base_x.insert(key, x.at(key));
  }
  for (const Key& key : cmanifold_keys_) {
    const auto& cmanifold = x.at(key).cast<ConstraintManifold>();
    InsertSelected(cmanifold.values(), base_factor_->keys(), base_x);
  }

  // compute jacobian
  if (H) {
    // Compute Jacobian and error using base factor.
    std::vector<Matrix> base_H(base_x.size());
    Vector unwhitened_error = base_factor_->unwhitenedError(base_x, base_H);

    // Compute Jacobian for new variables
    for (size_t variable_idx = 0; variable_idx < keys().size();
         variable_idx++) {
      const Key& key = keys().at(variable_idx);
      if (unconstrained_keys_.exists(key)) {
        (*H)[variable_idx] = base_H.at(base_key_index_.at(key));
      } else {
        const auto& cmanifold = x.at(key).cast<ConstraintManifold>();
        bool H_initialized = false;
        for (const Key& base_key : cmanifold.values().keys()) {
          if (base_key_index_.find(base_key) != base_key_index_.end()) {
            const size_t& base_key_index = base_key_index_.at(base_key);
            Matrix H_recover;
            cmanifold.recover(base_key, H_recover);
            if (!H_initialized) {
              (*H)[variable_idx] = base_H.at(base_key_index) * H_recover;
              H_initialized = true;
              continue;
            }
            (*H)[variable_idx] += base_H.at(base_key_index) * H_recover;
          }
        }
      }
    }
    return unwhitened_error;
  } else {
    return base_factor_->unwhitenedError(base_x);
  }
}

}  // namespace gtdynamics
