/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IEManifoldLinearization.h
 * @brief Immutable active-constraint geometry for one IE manifold state.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtdynamics/cmcopt/TangentCone.h>

namespace gtdynamics {
/** Active inequalities in one component's equality-tangent coordinates.
 * Each block represents Dg_i(x) B delta <= 0. Indices always refer to the
 * original nonlinear constraint blocks; a block can contain multiple rows.
 * Raw rows preserve directional tolerances; whitened rows preserve the noise
 * metric used for QP assembly and face checks. Geometry belongs to a
 * single state and is shared read-only across LM trials and manifold copies.
 */
class IEManifoldLinearization {
 public:
  /** Copy homogeneous unary tangent factors and optional ambient factors.
   * Input keys are replaced by the local projection key. Nonlocal, nonfinite,
   * nonzero-RHS, or hard-noise tangent factors throw invalid_argument.
   */
  explicit IEManifoldLinearization(
      const LinearIConstraintMap& tangentConstraints,
      const LinearIConstraintMap& ambientConstraints = {});

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
