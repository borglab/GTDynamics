/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file HalfSphereRetractor.h
 * @brief HalfSphereRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

class IEHalfSphere;

/** Manually defined retractor for half sphere manifold. */
class HalfSphereRetractor : public IERetractor {
  /// Sphere radius.
  double r_;

public:
  /** Constructor. */
  HalfSphereRetractor(const IEHalfSphere &half_sphere);

  /** Retract a tangent update and project back to the upper half-sphere. */
  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
