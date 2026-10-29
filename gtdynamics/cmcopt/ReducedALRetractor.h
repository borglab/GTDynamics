/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ReducedALRetractor.h
 * @brief Declare the reduced augmented Lagrangian retractor.
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmopt/TangentSpaceMetric.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>

namespace gtdynamics {

/** Metric projection on an affine equality manifold with the original
 * inequalities.
 *
 * Requires equalities that are affine in Euclidean variables. The retractor
 * uses the manifold's own equality basis B, with
 * x = x0 + B z; no separate dynamics reduction is supplied by the caller.
 * A Cholesky change of coordinates makes the projection metric the identity;
 * a common trial-length scaling then keeps the target norm at most one.
 * Original scalar inequalities are evaluated at x0 + B z, with Jacobians
 * chained through B, and solved by GTSAM's AL optimizer. No quadratic model
 * callback is needed; a constraint may depend on multiple ambient keys.
 *
 * This is not a retractor for arbitrary nonlinear equality manifolds. Supported
 * Values entries are double, Vector, and fixed column vectors of size 1..6.
 * The fixed-size limit is the implementation's runtime type whitelist, not a
 * mathematical limitation; dynamic Vector entries can have any dimension.
 * Affine equalities are required because a tangent basis describes a nonlinear
 * equality manifold only locally, not exactly as x = x0 + B z.
 * The manifold owns reusable basis/metric geometry. An already feasible trial
 * (or feasible unconstrained metric projection) is returned without AL. Original
 * constraints and required blocking faces are checked before either shortcut.
 * Failed convergence or original-constraint validation throws an exception;
 * feasibility alone is not accepted as convergence. For general nonconvex
 * problems, AL convergence tests local stationarity, not a global projection.
 * Nonsmooth inequalities retain their original Jacobian convention; AL may
 * fail at a cone apex. No smoothness or global convergence is asserted.
 */
class ReducedALRetractor : public IERetractor {
 public:
  struct Settings {
    /// BCL AL settings. Stationarity is measured in normalized coordinates.
    gtsam::AugmentedLagrangianParams al;
    /// Infinity norm of ORIGINAL whitened equality/inequality residuals.
    double feasibilityTolerance = 1e-8;
    /// Near-tight original inequalities retained in the next tangent cone.
    double activeTolerance = 1e-7;
    /** Configure BCL AL and its inner LM solver. */
    Settings();
  };

  /** Project using the manifold basis and the specified ambient prior metric.
   */
  explicit ReducedALRetractor(
      const IERetractorParams::shared_ptr &params,
      const Settings &settings = Settings(),
      const std::optional<KeyVector> &priorKeys = {});

  /** Project a trial, imposing selected blocking inequalities as equalities.
   * The non-owning manifold pointer follows the virtual IERetractor API and
   * must be nonnull; a null pointer throws std::invalid_argument.
   */
  IEConstraintManifold retract(
      const IEConstraintManifold *manifold, const VectorValues &delta,
      const std::optional<IndexSet> &blockingIndices = {},
      IERetractionInfo *info = nullptr) const override;

 private:
  /** Coordinates anchored at the manifold's current values.
   * Cholesky whitening and a common length scale make the projection metric
   * proportional to the identity, with a target norm at most one.
   */
  struct Reduction {
    /// Per-variable displacement maps: x[key] = x0[key] + maps[key] * u.
    TangentSpaceMetric::shared_ptr metric;
    double lengthScale = 1.0;
    /// Unconstrained projection target in the normalized u coordinates.
    Vector target;
  };
  Settings settings_;
  std::optional<KeyVector> priorKeys_;

  /** Test original constraints, including any required boundary faces. */
  std::optional<IndexSet> feasibleResult(const IEConstraintManifold &manifold,
      const Values &values, const IndexSet &blocking) const;
  /** Pull back original inequalities to the reduced variables. */
  gtsam::ConstrainedOptProblem buildProblem(
      const IEConstraintManifold &manifold, const Reduction &reduction,
      const IndexSet &blocking) const;
  /** Run AL and require both stationarity and feasibility convergence. */
  Vector solveProblem(const gtsam::ConstrainedOptProblem &problem,
                      const Reduction &reduction, IERetractionInfo *info) const;
  /** Validate in original coordinates and retain near-tight faces. */
  IndexSet validateResult(const IEConstraintManifold &manifold,
                          const Values &values, const IndexSet &blocking) const;
};

}  // namespace gtdynamics
