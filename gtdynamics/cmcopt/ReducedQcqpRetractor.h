/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ReducedQcqpRetractor.h
 * @brief Declare the reduced QCQP retractor.
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>

#include <functional>

namespace gtdynamics {

/** Metric projection on an affine equality manifold with QCQP inequalities.
 *
 * The caller explicitly certifies that the equalities are affine in Euclidean
 * variables. The retractor uses the manifold's own equality basis B, with
 * x = x0 + B z; no separate dynamics reduction is supplied by the caller.
 * A Cholesky change of coordinates makes the projection metric the identity;
 * a common trial-length scaling then keeps the target norm at most one.
 * Both linear and quadratic inequalities are solved by GTSAM's AL optimizer.
 *
 * This is not a retractor for arbitrary nonlinear equality manifolds. Supported
 * Values entries are double, Vector, and fixed column vectors of size 1..6.
 * The fixed-size limit is the implementation's runtime type whitelist, not a
 * QCQP limitation; dynamic Vector entries can have any dimension. Affine
 * equalities are required because a tangent basis describes a nonlinear
 * equality manifold only locally, not exactly as x = x0 + B z.
 * Failed convergence or original-constraint validation throws an exception;
 * feasibility alone is not accepted as convergence. For general nonconvex
 * QCQPs, AL convergence certifies local stationarity, not a global projection.
 */
class ReducedQcqpRetractor : public IERetractor {
 public:
  /// Explicit assertion by the caller, not an inferred Jacobian property.
  struct AffineEqualities {};

  /** Exact scalar inequality model in its original variable coordinates.
   * The returned unary quadratic must have the same feasible set AND boundary
   * as the original constraint (in conjunction with the other inequalities).
   * For a squared SOC, a separate normal >= 0 constraint is essential. A zero
   * quadratic matrix represents a linear constraint. The outer CMC constraint
   * is left unchanged, including its Jacobian and index in the active set.
   */
  using InequalityModel = std::function<gtsam::QuadraticConstraint(
      const gtsam::NonlinearInequalityConstraint&)>;

  struct Settings {
    /// BCL AL settings. Stationarity is measured in normalized coordinates.
    gtsam::AugmentedLagrangianParams al;
    /// Infinity norm of ORIGINAL whitened equality/inequality residuals.
    double feasibilityTolerance = 1e-8;
    /// Near-tight original inequalities retained in the next tangent cone.
    double activeTolerance = 1e-7;
    Settings();
  };

  ReducedQcqpRetractor(AffineEqualities, InequalityModel model,
                       const IERetractorParams::shared_ptr& params,
                       const Settings& settings = Settings(),
                       const std::optional<KeyVector>& priorKeys = {});

  IEConstraintManifold retract(
      const IEConstraintManifold* manifold, const VectorValues& delta,
      const std::optional<IndexSet>& blockingIndices = {},
      IERetractionInfo* info = nullptr) const override;

 private:
  struct Reduction {
    std::map<Key, Matrix> maps;  // dx/du, with unit metric in u coordinates
    Vector target;
  };
  InequalityModel model_;
  Settings settings_;
  std::optional<KeyVector> priorKeys_;

  Reduction buildReduction(const IEConstraintManifold& manifold,
                           const Values& target) const;
  gtsam::QcqpProblem buildProblem(const IEConstraintManifold& manifold,
                                 const Reduction& reduction,
                                 const IndexSet& blocking) const;
  Vector solveProblem(const gtsam::QcqpProblem& problem,
                      const Reduction& reduction,
                      IERetractionInfo* info) const;
  IndexSet validateResult(const IEConstraintManifold& manifold,
                          const Values& values,
                          const IndexSet& blocking) const;
};

}  // namespace gtdynamics
