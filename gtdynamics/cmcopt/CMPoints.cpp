/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CMPoints.cpp
 * @brief Ambient recovery and boundary operations for collections of points.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmcopt/CMPoint.h>

namespace gtdynamics {
using namespace gtsam;

/* ************************************************************************* */
Values ambientValues(const CMPoints& points) {
  Values values;
  for (const auto& it : points) {
    values.insert(it.second.values());
  }

  return values;
}

/* ************************************************************************* */
KeyVector pointKeys(const CMPoints& points) {
  KeyVector key_vector;
  for (const auto& it : points) {
    key_vector.push_back(it.first);
  }

  return key_vector;
}

/* ************************************************************************* */
CMPoints movePointsToBoundaries(const CMPoints& points,
                                const IndexSetMap& approach_indices_map,
                                const VectorValues* metricSigmas) {
  CMPoints newPoints;
  // Preserve each type's metric policy while forcing the requested boundaries.
  for (const auto& [key, point] : points) {
    if (approach_indices_map.find(key) == approach_indices_map.end()) {
      newPoints.insert({key, point});
    } else {
      newPoints.insert(
          {key, point.moveToBoundary(
                    approach_indices_map.at(key), nullptr,
                    metricSigmas && point.manifold()->usesVaryingMetric()
                        ? metricSigmas
                        : nullptr)});
    }
  }

  return newPoints;
}

}  // namespace gtdynamics
