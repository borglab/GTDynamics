/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file AffineProjection.h
 * @brief Share affine projection geometry, constraints, and feasibility checks.
 * @author Frank Dellaert
 */

#pragma once
#include <gtdynamics/cmopt/TangentSpaceMetric.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/constrained/ConstrainedOptProblem.h>
#include <optional>

namespace gtdynamics {
class IEConstraintManifold;
struct IERetractorParams;
struct IERetractionInfo;

/** A shared projection problem on an exact affine equality manifold.
 * The manifold owns the basis and metric cache. This short-lived object owns
 * only the trial coordinates and builds constraints on recovered original keys.
 * Values must be Euclidean scalars/vectors; no nonlinear recovery is implied.
 * Its manifold reference must outlive it. Built factors own their anchor/maps.
 */
class AffineProjection {
public:
  static constexpr Key kReduced = 0;
  /** Normalize AL coordinates/rows when requested. Penalty methods must disable
   * normalization to preserve their original cost-versus-penalty weighting.
   * Whitening is independently controlled by params.whitenAffineCoordinates.
   */
  AffineProjection(const IEConstraintManifold &manifold,
                   const VectorValues &delta, const IERetractorParams &params,
                   const std::optional<KeyVector> &priorKeys,
                   const IndexSet &blocking, double feasibilityTolerance,
                   double activeTolerance, bool normalize = true);
  size_t dim() const { return target_.size(); }
  /// v is measured relative to the unconstrained equality projection.
  Values recover(const Vector &v) const;
  Values initialValues(bool atAnchor) const;
  gtsam::ConstrainedOptProblem buildProblem(const IndexSet &blocking) const;
  /// Return a proven projection minimizer without invoking a constrained
  /// solver.
  struct Result {
    Values values;
    IndexSet active;
  };
  std::optional<Result> feasibleProjection(const IndexSet &blocking,
                                           IERetractionInfo *info) const;
  std::optional<IndexSet> feasibleResult(const Values &values,
                                         const IndexSet &blocking) const;
  IndexSet validateResult(const Values &values, const IndexSet &blocking) const;

private:
  const IEConstraintManifold &manifold_;
  TangentSpaceMetric::shared_ptr metric_;
  Values trial_;
  Vector target_;
  double length_scale_ = 1.0;
  double feasibility_tolerance_, active_tolerance_;
  bool normalize_;
};
} // namespace gtdynamics
