/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DomeRetractor.cpp
 * @brief DomeRetractor implementation.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/DomeRetractor.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>

namespace gtdynamics {

DomeRetractor::DomeRetractor(const IEHalfSphere &half_sphere)
    : IERetractor(), r_(half_sphere.radius_) {}

IEConstraintManifold
DomeRetractor::retract(const IEConstraintManifold *manifold,
                       const VectorValues &delta,
                       const std::optional<IndexSet> &blocking_indices,
                       IERetractionInfo *retract_info) const {
  Key key = manifold->values().keys().front();
  Point3 p = manifold->values().at<Point3>(key);
  Vector3 v = delta.at(key);
  Point3 new_p = p + v;

  new_p.z() = std::max(0.0, new_p.z());
  if (new_p.norm() > r_) {
    new_p = r_ / new_p.norm() * new_p;
  }

  Values new_values;
  new_values.insert(key, new_p);
  return manifold->createWithNewValues(new_values);
}

IEConstraintManifold
DomeRetractor::moveToBoundary(const IEConstraintManifold *manifold,
                              const IndexSet &blocking_indices,
                              IERetractionInfo *retract_info) const {
  Key key = manifold->values().keys().front();
  Point3 p = manifold->values().at<Point3>(key);

  // std::cout << "old p: \n" << p << "\n";

  bool is_sphere = false;
  bool is_positive_z = false;

  IndexSet active_indices = blocking_indices;
  for (const auto &constraint_idx : manifold->activeIndices()) {
    active_indices.insert(constraint_idx);
  }

  for (const auto &constraint_idx : active_indices) {
    if (constraint_idx == 0) {
      is_sphere = true;
    }
    if (constraint_idx == 1) {
      is_positive_z = true;
    }
  }

  if (is_positive_z) {
    p.z() = 0;
  }

  if (is_sphere) {
    p = r_ / p.norm() * p;
  }

  // std::cout << "new p: \n" << p << "\n";

  Values new_values;
  new_values.insert(key, p);
  return manifold->createWithNewValues(new_values);
}

} // namespace gtdynamics
