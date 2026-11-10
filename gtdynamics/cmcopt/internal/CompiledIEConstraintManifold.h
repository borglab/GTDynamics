/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CompiledIEConstraintManifold.h
 * @brief Adapt ordinary GTSAM manifold types to CMC ambient recovery.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/constrained/internal/CompiledChart.h>

namespace gtdynamics::internal {
/** A compiled chart with native inequality and boundary operations. */
template <class T, class Recovery>
class CompiledIEConstraintManifold final
    : public CompiledChart<T, Recovery, IEConstraintManifold> {
  using Chart = CompiledChart<T, Recovery, IEConstraintManifold>;

 protected:
  std::vector<NativeBoundary> nativeBoundaries(const T& value) const override {
    return this->recovery_.boundaries(value);
  }
  T applyBoundary(const T& value, const IndexSet& active) const override {
    return this->recovery_.boundary(value, active);
  }

 public:
  using Chart::Chart;

  /// Select tight native boundaries, preserving complete block indices.
  IndexSet identifyActive(
      const IECMPoint& point,
      const std::optional<IndexSet>& supplied = {}) const override {
    const auto boundaries =
        this->recovery_.boundaries(this->recovery_.fromAmbient(this->local(point)));
    if (supplied) {
      for (size_t index : *supplied)
        if (index >= boundaries.size())
          throw std::invalid_argument("Invalid native boundary index");
      return *supplied;
    }
    IndexSet active;
    for (size_t i = 0; i < boundaries.size(); ++i)
      if ((boundaries[i].expression.array() >= 0).any()) active.insert(i);
    return active;
  }

  /// Native block counts are independent of factor storage.
  size_t inequalityCount(const IECMPoint& point) const override {
    return this->recovery_.boundaries(this->recovery_.fromAmbient(this->local(point))).size();
  }

  /// Return the raw native inequality expression.
  Vector inequalityResidual(const IECMPoint& point, size_t index) const override {
    return this->recovery_.boundaries(this->recovery_.fromAmbient(this->local(point)))
        .at(index)
        .expression;
  }

  /// Measure positive normalized boundary residuals.
  double inequalityViolation(const IECMPoint& point) const override {
    double squared = 0;
    for (const auto& block :
         this->recovery_.boundaries(this->recovery_.fromAmbient(this->local(point))))
      squared += block.expression.cwiseMax(0)
                     .cwiseQuotient(block.sigmas)
                     .squaredNorm();
    return std::sqrt(squared);
  }

};

}  // namespace gtdynamics::internal
