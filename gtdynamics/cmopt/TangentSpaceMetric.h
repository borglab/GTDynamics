/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file TangentSpaceMetric.h
 * @brief Cache recovery maps and a positive definite metric on a tangent basis.
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmopt/TangentSpaceBasis.h>

namespace gtdynamics {

/** Immutable metric on one fixed tangent basis.
 * For ambient diagonal precision W, N = B R^-1 and R'R = B'WB.
 * Thus N'WN = I, projection coordinates are N'W delta, and recovery is N u.
 * This linear algebra is valid for any tangent space; exact finite recovery
 * additionally requires affine equalities in Euclidean variables.
 */
class TangentSpaceMetric {
 public:
  using shared_ptr = std::shared_ptr<const TangentSpaceMetric>;
  TangentSpaceMetric(const TangentSpaceBasis &basis, const Values &values,
                     const VectorValues &precisions);
  const std::map<Key, Matrix> &maps() const { return maps_; }
  /// Exact comparison: changed weights or measured keys invalidate the cache.
  bool matches(const VectorValues &precisions) const;
  Vector project(const VectorValues &delta) const;
  VectorValues recover(const Vector &u) const;

 private:
  VectorValues precisions_;
  std::map<Key, Matrix> maps_;
  size_t dim_;
};

}  // namespace gtdynamics
