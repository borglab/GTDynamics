/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ComponentTangentSpace.h
 * @brief Parameterize component-local working faces for unconstrained LM.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

#include <map>

namespace gtdynamics {
namespace internal {
/** Stack soft Jacobian costs with identical key scopes and original noise.
 * First-occurrence order and singleton pointers are preserved. Other Gaussian
 * representations remain unchanged; this operation does not eliminate keys.
 */
gtsam::GaussianFactorGraph compactGaussianCosts(
    const gtsam::GaussianFactorGraph& cost);

/** Product of local null spaces for one CMC working set.
 * Keys denote equality-manifold components, with increments delta_i. Each
 * supplied matrix defines homogeneous working faces A_i delta_i = 0. The
 * object stores N_i spanning ker(A_i), so delta_i = N_i eta_i. Missing keys
 * retain their original coordinates; full-rank blocks have zero free DOF.
 * No global constraint graph is constructed. Dependent and zero rows are
 * allowed. This is an internal implementation API, not a general QP solver.
 */
class ComponentTangentSpace {
 public:
  /** Construct component bases. Reject nonfinite rows or zero-column blocks. */
  explicit ComponentTangentSpace(
      const std::map<gtsam::Key, gtsam::Matrix>& faces);

  /** Substitute local bases into soft costs, including any existing damping.
   * Constant-only factors are omitted from the solve. Invalid key dimensions
   * and constrained noise throw invalid_argument; Hessian costs use GTSAM's
   * native Jacobian conversion. The original graph is not modified.
   */
  gtsam::GaussianFactorGraph reduce(
      const gtsam::GaussianFactorGraph& cost) const;

  /** Recover original component increments, including exact zero-DOF blocks.
   * Reject nonfinite results or unit-row face residuals exceeding 1e-7.
   * This tolerance is independent of the supplied row scales. Dimensions
   * describe the original cost graph, before this additional reduction.
   */
  gtsam::VectorValues recover(
      const gtsam::VectorValues& reduced,
      const std::map<gtsam::Key, size_t>& dimensions) const;

  /** Reduce, solve an unconstrained Gaussian graph, then recover increments.
   * Preserve the requested solver and ordering, removing zero-DOF keys.
   * Numerical failures propagate to LM trial rejection; there is no fallback
   * to global constrained elimination and no extra objective regularization.
   */
  gtsam::VectorValues solve(
      const gtsam::GaussianFactorGraph& cost,
      const gtsam::NonlinearOptimizerParams& params) const;

 private:
  /// Unit-normalized face rows; identically zero rows remain zero.
  std::map<gtsam::Key, gtsam::Matrix> faces_;
  std::map<gtsam::Key, gtsam::Matrix> bases_;
};
}  // namespace internal
}  // namespace gtdynamics
