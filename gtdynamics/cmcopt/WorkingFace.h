/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file WorkingFace.h
 * @brief Parameterize component-local working faces for unconstrained LM.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

#include <Eigen/SparseCore>
#include <map>

namespace gtdynamics {
namespace internal {
/** Coordinates for one component's selected homogeneous faces A delta = 0.
 * Unit row normalization and FullPivLU preserve rank decisions across scales.
 * The equality tangent basis B belongs to the manifold; this chart only owns N.
 */
class WorkingFace {
 public:
  /// Construct N spanning ker(rows), including zero-dimensional kernels.
  explicit WorkingFace(gtsam::Matrix rows);
  /// Number of original equality-tangent coordinates.
  size_t dimension() const { return basis_.rows(); }
  /// Number of remaining coordinates on the selected face.
  size_t freeDimension() const { return basis_.cols(); }
  /// Substitute N into one objective Jacobian; preserve every nonzero entry.
  gtsam::Matrix reduce(const gtsam::Matrix& jacobian) const;
  /// Recover N eta and check normalized face residuals at tolerance 1e-7.
  gtsam::Vector recover(const gtsam::Vector& reduced) const;

 private:
  gtsam::Matrix rows_, basis_;
  Eigen::SparseMatrix<double> sparseBasis_;
};
using WorkingFaces = std::map<gtsam::Key, WorkingFace>;
/** Substitute local charts into soft Gaussian costs, omitting fixed constants.
 * Missing keys keep identity coordinates. Hard constraints are rejected.
 */
gtsam::GaussianFactorGraph reduceWorkingFaces(
    const gtsam::GaussianFactorGraph& cost, const WorkingFaces& faces);
/// Lift all reduced increments, restoring zero-DOF component entries.
gtsam::VectorValues recoverWorkingFaces(
    const gtsam::VectorValues& reduced,
    const std::map<gtsam::Key, size_t>& dimensions, const WorkingFaces& faces);
/** Solve the coupled soft objective in component charts using GTSAM.
 * Damping must already be included; filter zero-DOF keys from explicit
 * ordering.
 */
gtsam::VectorValues solveWorkingFaces(
    const gtsam::GaussianFactorGraph& cost, const WorkingFaces& faces,
    const gtsam::NonlinearOptimizerParams& params);
/// Stack costs with identical key scopes without changing whitening or order.
gtsam::GaussianFactorGraph compactGaussianCosts(
    const gtsam::GaussianFactorGraph& cost);
}  // namespace internal
}  // namespace gtdynamics
