/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file SphereRetractor.cpp
 * @brief SphereRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtdynamics/scenarios/SphereRetractor.h>

namespace gtdynamics {

SphereRetractor::SphereRetractor(const IEHalfSphere &half_sphere)
    : IERetractor(), r_(half_sphere.radius_) {}

IEConstraintManifold
SphereRetractor::retract(const IEConstraintManifold *manifold,
                         const VectorValues &delta,
                         const std::optional<IndexSet> &blocking_indices,
                         IERetractionInfo *retract_info) const {
  Key key = manifold->values().keys().front();
  Point3 p = manifold->values().at<Point3>(key);
  Vector3 v = delta.at(key);
  Point3 new_p = p + v;

  new_p = normalize(new_p) * r_;
  Values new_values;
  new_values.insert(key, new_p);
  return manifold->createWithNewValues(new_values);
}

} // namespace gtdynamics
