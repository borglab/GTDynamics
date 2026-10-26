#include <gtdynamics/cmcopt/IEConstraintManifold.h>

namespace gtdynamics {
using namespace gtsam;

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
  IEManifoldValues newManifolds;
  for (const auto &[key, manifold] : *this) {
    if (approach_indices_map.find(key) == approach_indices_map.end()) {
      newManifolds.insert({key, manifold});
    } else {
      newManifolds.insert(
          {key, manifold.moveToBoundary(approach_indices_map.at(key))});
    }
  }
  return newManifolds;
}

} // namespace gtdynamics
