/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CPBarrierRetractor.h
 * @brief CPBarrierRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

class IECartPoleWithFriction;

/** Barrier-based retractor for the cart-pole friction cone. */
class CPBarrierRetractor : public IERetractor {
protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithFriction &cp_;

public:
  /** Constructor. */
  CPBarrierRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update using a barrier-style local solve. */
  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
