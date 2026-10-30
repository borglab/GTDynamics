/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CartPoleWithFrictionRetractor.h
 * @brief CartPoleWithFrictionRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

class IECartPoleWithFriction;

/** Retraction rule specialized for the cart-pole friction cone. */
class CartPoleWithFrictionRetractor : public IERetractor {
protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithFriction &cp_;

public:
  /** Constructor. */
  CartPoleWithFrictionRetractor(const IECartPoleWithFriction &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update and enforce active friction constraints. */
  IEConstraintManifold
  retract(const IEConstraintManifold *manifold, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;

  /** Fallback penalty-based retraction for boundary corner cases. */
  IEConstraintManifold retract1(const IEConstraintManifold *manifold,
                                const VectorValues &delta) const;
};

} // namespace gtdynamics
