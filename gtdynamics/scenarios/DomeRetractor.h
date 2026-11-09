/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DomeRetractor.h
 * @brief DomeRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

class IEHalfSphere;

/** Manually defined retractor for the dome manifold. */
class DomeRetractor : public IERetractor {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<DomeRetractor>(*this);
  }

  /// Sphere radius.
  double r_;

public:
  /** Constructor. */
  DomeRetractor(const IEHalfSphere &half_sphere);

  /** Retract a tangent update and clamp to the dome boundary if needed. */
  CMPoint
  retract(const CMPoint *point, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;

  /** Move the current point onto active dome boundary constraints. */
  CMPoint
  moveToBoundary(const CMPoint *point,
                 const IndexSet &blocking_indices,
                 IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
