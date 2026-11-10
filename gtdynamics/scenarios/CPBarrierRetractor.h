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
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<CPBarrierRetractor>(*this);
  }

protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithFriction &cp_;

public:
  /** Constructor. */
  CPBarrierRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update using a barrier-style local solve. */
  IECMPoint
  retract(const IECMPoint *point, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
