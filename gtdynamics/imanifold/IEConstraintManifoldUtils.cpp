#include <gtdynamics/imanifold/IEConstraintManifold.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {
/* ************************************************************************* */
Values CollectManifoldValues(const IEManifoldValues &manifolds) {
  Values values;
  for (const auto &it : manifolds) {
    values.insert(it.second.values());
  }
  return values;
}

/* ************************************************************************* */
IEManifoldValues MoveToBoundaries(const IEManifoldValues &manifolds,
                                  const IndexSetMap &approach_indices_map) {
  IEManifoldValues new_manifolds;
  for (const auto &it : manifolds) {
    const Key &key = it.first;
    const auto &manifold = it.second;
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
