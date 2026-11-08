/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CMCSubstituteFactor.cpp
 * @brief Substitute factor implementations.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/constrained/CMCSubstituteFactor.h>
#include <gtdynamics/cmopt/ECMPoint.h>

namespace gtdynamics {

/* ************************************************************************* */
KeyVector CMCSubstituteFactor::computeNewKeys(
    const Base::shared_ptr& base_factor,
    const std::map<Key, Key>& replacement_map, const Values& fixedPoints) {
  const KeyVector& base_keys = base_factor->keys();
  KeySet pointKeys;
  KeyVector new_keys;
  for (const Key& base_key : base_keys) {
    if (replacement_map.find(base_key) != replacement_map.end()) {
      Key new_key = replacement_map.at(base_key);
      if (!fixedPoints.exists(new_key)) {
        if (!pointKeys.exists(new_key)) {
          new_keys.push_back(new_key);
          pointKeys.insert(new_key);
        }
      }
    } else {
      new_keys.push_back(base_key);
    }
  }

  return new_keys;
}

/* ************************************************************************* */
void CMCSubstituteFactor::computeBaseKeyIndex() {
  const KeyVector& base_keys = base_factor_->keys();
  for (size_t key_index = 0; key_index < base_keys.size(); key_index++) {
    const Key& base_key = base_keys.at(key_index);
    base_key_index_[base_key] = key_index;
  }
}

/* ************************************************************************* */
void CMCSubstituteFactor::classifyKeys(const Values& fixedPoints) {
  const KeyVector& base_keys = base_factor_->keys();
  for (const Key& base_key : base_keys) {
    if (isReplaced(base_key)) {
      Key new_key = replacement_map_.at(base_key);
      if (fixedPoints.exists(new_key)) {
        fc_values_.insert(
            base_key,
            (pointType_ == PointType::Equality
                ? static_cast<const CMPoint&>(fixedPoints.at(new_key).cast<ECMPoint>())
                : static_cast<const CMPoint&>(fixedPoints.at(new_key).cast<IECMPoint>())).recover(base_key));
      } else {
        pointKeys_.insert(new_key);
      }
    } else {
      unconstrained_keys_.insert(base_key);
    }
  }
}

/* ************************************************************************* */
/** Insert values (for key in keys) to output_values */
void InsertCMSelected(const Values& values, const KeyVector& keys,
                      Values& output_values) {
  for (const Key& key : keys) {
    if (values.exists(key)) {
      output_values.insert(key, values.at(key));
    }
  }
}

/* ************************************************************************* */
Vector CMCSubstituteFactor::unwhitenedError(
    const Values& x, gtsam::OptionalMatrixVecType H) const {
  // Construct values for base factor.
  Values base_x = fc_values_;
  for (const Key& key : unconstrained_keys_) {
    base_x.insert(key, x.at(key));
  }

  for (const Key& key : pointKeys_) {
    const CMPoint& point = pointType_ == PointType::Equality
        ? static_cast<const CMPoint&>(x.at(key).cast<ECMPoint>())
        : static_cast<const CMPoint&>(x.at(key).cast<IECMPoint>());
    InsertCMSelected(point.values(), base_factor_->keys(), base_x);
  }

  // Pull the original factor's Jacobians back to point coordinates.
  if (H) {
    // Compute Jacobian and error using base factor.
    std::vector<Matrix> base_H(base_x.size());
    Vector unwhitened_error = base_factor_->unwhitenedError(base_x, base_H);

    // Sum contributions when several ambient keys belong to the same point.
    for (size_t variable_idx = 0; variable_idx < keys().size();
         variable_idx++) {
      const Key& key = keys().at(variable_idx);
      if (unconstrained_keys_.exists(key)) {
        (*H)[variable_idx] = base_H.at(base_key_index_.at(key));
      } else {
        const CMPoint& point = pointType_ == PointType::Equality
        ? static_cast<const CMPoint&>(x.at(key).cast<ECMPoint>())
        : static_cast<const CMPoint&>(x.at(key).cast<IECMPoint>());
        bool H_initialized = false;
        for (const Key& base_key : point.values().keys()) {
          if (base_key_index_.find(base_key) != base_key_index_.end()) {
            const size_t& base_key_index = base_key_index_.at(base_key);
            Matrix H_recover;
            point.recover(base_key, H_recover);
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

gtsam::NonlinearFactorGraph pointSubstitutionGraph(
    const gtsam::NonlinearFactorGraph& graph,
    const std::map<Key, Key>& ownership, const Values& fixedPoints,
    CMCSubstituteFactor::PointType pointType) {
  gtsam::NonlinearFactorGraph result;
  for (const auto& factor : graph) {
    if (!factor) continue;
    std::map<Key, Key> local;
    for (Key key : factor->keys()) {
      const auto found = ownership.find(key);
      if (found != ownership.end()) local.emplace(*found);
    }
    if (local.empty()) {
      result.push_back(factor);
      continue;
    }
    const auto noise = std::dynamic_pointer_cast<NoiseModelFactor>(factor);
    if (!noise)
      throw std::invalid_argument("Point substitution requires noise-model costs");
    auto transformed = std::make_shared<CMCSubstituteFactor>(
        noise, local, fixedPoints, pointType);
    // Fully fixed costs stay in the ambient objective, outside Gaussian variables.
    if (transformed->checkActive()) result.push_back(transformed);
  }
  return result;
}

}  // namespace gtdynamics
