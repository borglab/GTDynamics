/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CMTangentSpace.h
 * @brief Immutable active-constraint geometry for one IE manifold state.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/constrained/TangentCone.h>
#include <gtdynamics/constrained/TangentSpaceMetric.h>

#include <mutex>

namespace gtdynamics {
/** Active inequalities in one component's equality-tangent coordinates.
 * Each block represents Dg_i(x) B delta <= 0. Indices always refer to the
 * original nonlinear constraint blocks; a block can contain multiple rows.
 * Raw rows preserve directional tolerances; whitened rows preserve the noise
 * metric used for QP assembly and face checks. Geometry belongs to a
 * single state and is shared read-only across LM trials and manifold copies.
 */
class CMTangentSpace {
 public:
  /** Copy homogeneous unary tangent factors and optional ambient factors.
   * Input keys are replaced by the local projection key. Nonlocal, nonfinite,
   * nonzero-RHS, or hard-noise tangent factors throw invalid_argument.
   */
  explicit CMTangentSpace(const LinearIConstraintMap& tangentConstraints,
                          const LinearIConstraintMap& ambientConstraints = {});

  /// Complete point geometry with a read-only equality basis and fresh metrics.
  CMTangentSpace(const std::shared_ptr<const TangentSpaceBasis>& basis,
                 const LinearIConstraintMap& tangentConstraints,
                 const LinearIConstraintMap& ambientConstraints);

  /// Numerical equality basis at this point.
  const std::shared_ptr<const TangentSpaceBasis>& basis() const {
    return basis_;
  }

  /// Point-local projection metric; different effective weights never alias.
  TangentSpaceMetric::shared_ptr projectionMetric(
      const Values& values, const VectorValues& precisions, bool whiten) const;

  /// Original nonlinear indices of the cached active blocks.
  const IndexSet& indices() const { return indices_; }

  /// Immutable raw or whitened rows for one block, used by QP assembly.
  const Matrix& rows(size_t index, bool whitened = false) const;

  /// Raw directional violations, using the existing block-discovery tolerance.
  IndexSet violated(const Vector& delta, double tolerance = 1e-5) const;

  /// Selected whole blocks must vanish in their original whitened metric.
  bool active(const Vector& delta, const IndexSet& indices,
              double tolerance = 1e-5) const;

  /// Project onto the tangent cone; return original nonlinear block indices.
  std::pair<IndexSet, Vector> project(const Vector& delta) const;

  /// Check ambient directions, including directions outside the equality
  /// tangent.
  IndexSet blockingIndices(const VectorValues& direction) const;

 private:
  struct MetricCache {
    std::mutex mutex;
    TangentSpaceMetric::shared_ptr metric;
  };
  std::shared_ptr<const TangentSpaceBasis> basis_;
  std::shared_ptr<MetricCache> metricCache_ = std::make_shared<MetricCache>();
  struct Block {
    Matrix raw;
    Matrix whitened;
    SharedDiagonal noise;
    JacobianFactor::shared_ptr ambient;
  };
  std::map<size_t, Block> blocks_;
  IndexSet indices_;
  TangentCone::shared_ptr cone_;
};

}  // namespace gtdynamics
