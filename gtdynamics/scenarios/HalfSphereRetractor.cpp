/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file HalfSphereRetractor.cpp
 * @brief HalfSphereRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/HalfSphereRetractor.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>

namespace gtdynamics {

HalfSphereRetractor::HalfSphereRetractor(const IEHalfSphere &half_sphere)
    : IERetractor(), r_(half_sphere.radius_) {}

IEConstraintManifold
HalfSphereRetractor::retract(const IEConstraintManifold *manifold,
                             const VectorValues &delta,
                             const std::optional<IndexSet> &blocking_indices,
                             IERetractionInfo *retract_info) const {
  Key key = manifold->values().keys().front();
  Point3 p = manifold->values().at<Point3>(key);
  Vector3 v = delta.at(key);
  Point3 new_p = p + v;

  if (blocking_indices && blocking_indices->size() > 0) {
    new_p.z() = 0;
  }
  new_p.z() = std::max(0.0, new_p.z());
  new_p = normalize(new_p) * r_;

  Values new_values;
  new_values.insert(key, new_p);
  return manifold->createWithNewValues(new_values);
}

} // namespace gtdynamics
