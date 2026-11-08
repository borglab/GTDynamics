/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file CompiledEConstraintManifold.h
 * @brief Equality-only compiled GTSAM manifold adapter.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/constrained/internal/CompiledChart.h>

namespace gtdynamics::internal {
/// Equality charts need only recovery/Jacobians and ordinary GTSAM traits.
template <class T, class Recovery>
class CompiledEConstraintManifold final
    : public CompiledChart<T, Recovery, EConstraintManifold> {
  using Chart = CompiledChart<T, Recovery, EConstraintManifold>;

 public:
  using Chart::Chart;
};

}  // namespace gtdynamics::internal
