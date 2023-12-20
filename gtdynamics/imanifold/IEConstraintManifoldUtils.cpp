#include <gtdynamics/imanifold/IEConstraintManifold.h>

namespace gtsam {
/* ************************************************************************* */
Values IEManifoldValues::baseValues() const {
  Values values;
  for (const auto &it : *this) {
    values.insert(it.second.values());
  }
  return values;
}

/* ************************************************************************* */
KeyVector IEManifoldValues::keys() const {
  KeyVector key_vector;
  for (const auto &it : *this) {
    key_vector.push_back(it.first);
  }
  return key_vector;
}

/* ************************************************************************* */
IEManifoldValues IEManifoldValues::moveToBoundaries(
    const IndexSetMap &approach_indices_map) const {
  IEManifoldValues new_manifolds;
  for (const auto &[key, manifold] : *this) {
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
