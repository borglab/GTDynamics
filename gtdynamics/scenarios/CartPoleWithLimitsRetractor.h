/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CartPoleWithLimitsRetractor.h
 * @brief CartPoleWithLimitsRetractor declaration.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>

namespace gtdynamics {

class IECartPoleWithLimits;

/** Retraction rule that clamps cart position and force to their limits. */
class CartPoleWithLimitsRetractor : public IERetractor {
 public:
  /// Isolated copy used by runtime manifold types.
  IERetractor::shared_ptr clone() const override {
    return std::make_shared<CartPoleWithLimitsRetractor>(*this);
  }

protected:
  /// Cart-pole problem parameters.
  const IECartPoleWithLimits &cp_;

public:
  /** Constructor. */
  CartPoleWithLimitsRetractor(const IECartPoleWithLimits &cp)
      : IERetractor(), cp_(cp) {}

  /** Retract a tangent update and return a feasible constrained manifold. */
  CMPoint
  retract(const CMPoint *point, const VectorValues &delta,
          const std::optional<IndexSet> &blocking_indices = {},
          IERetractionInfo *retract_info = nullptr) const override;
};

} // namespace gtdynamics
