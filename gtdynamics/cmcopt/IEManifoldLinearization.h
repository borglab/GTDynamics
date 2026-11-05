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
#include <gtdynamics/cmcopt/WorkingFace.h>

#include <optional>

namespace gtdynamics {
/** Active inequalities in one component's equality-tangent coordinates.
 * Each block represents Dg_i(x) B delta <= 0. Indices always refer to the
 * original nonlinear constraint blocks; a block can contain multiple rows.
 * Raw rows preserve directional tolerances; whitened rows preserve the noise
 * metric used for working-face checks and refinement. Geometry belongs to a
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

  /// Immutable raw or whitened rows for one block, used by row-wise refinement.
  const Matrix& rows(size_t index, bool whitened = false) const;

  /// Raw directional violations, using the existing block-discovery tolerance.
  IndexSet violated(const Vector& delta, double tolerance = 1e-5) const;

  /// Selected whole blocks must vanish in their original whitened metric.
  bool active(const Vector& delta, const IndexSet& indices,
              double tolerance = 1e-5) const;

  /// Project onto the tangent cone; return original nonlinear block indices.
  std::pair<IndexSet, Vector> project(const Vector& delta) const;

  /// A row is identified by its original nonlinear block and row within it.
  using Row = std::pair<size_t, size_t>;
  using RowSet = std::set<Row>;
  /// Build a chart for selected whole blocks with their original raw scaling.
  internal::WorkingFace workingFace(const IndexSet& indices) const;
  /// Build a chart for individually selected rows, using whitened scaling.
  internal::WorkingFace workingFace(const RowSet& rows) const;
  /// Expand whole blocks to rows; reject an infeasible QP warm start at 1e-7.
  RowSet initialRows(const IndexSet& indices, const Vector& initial) const;
  /// Whole blocks whose every row is selected; partial blocks stay
  /// inequalities.
  IndexSet completeBlocks(const RowSet& rows) const;
  /// Return the first entering row and step fraction, or {maximumStep,
  /// nullopt}.
  std::pair<double, std::optional<Row>> boundaryStep(
      const Vector& current, const Vector& direction, const RowSet& active,
      double maximumStep = 1.0) const;
  /** Solve local dual equations and return the most negative multiplier below
   * -1e-7. The supplied gradient includes all coupled objective terms.
   */
  std::pair<double, std::optional<Row>> leavingRow(const Vector& gradient,
                                                   const RowSet& active) const;

  /// Check ambient directions, including directions outside the equality
  /// tangent.
  IndexSet blockingIndices(const VectorValues& direction) const;

 private:
  /** Stack selected blocks in index order, retaining their raw or whitened
   * scaling. The returned matrix defines the selected local working face.
   */
  Matrix workingRows(const IndexSet& indices, bool whitened = false) const;

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
