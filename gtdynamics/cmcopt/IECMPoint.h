/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IECMPoint.h
 * @brief Values of runtime constraint-manifold types.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/constrained/CMPoint.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/constrained/TangentSpaceMetric.h>
#include <gtsam/base/Manifold.h>

namespace gtdynamics {
class ECMPoint;
/** An immutable point with ambient identities and ordinary vector tangents.
 * Copies share read-only geometry. Successors preserve the runtime type and
 * bindings, and rebuild nonlinear geometry. Retraction work is never shared.
 */
class IECMPoint : public CMPoint {
 public:
  enum { dimension = Eigen::Dynamic };
  using shared_ptr = std::shared_ptr<IECMPoint>;
  using Bindings = IEConstraintManifold::Bindings;

  /// Empty value for GTSAM concept/default construction, not an optimizable
  /// point.
  IECMPoint() = default;

  /// Runtime type identity, including immutable conditioning.
  IEConstraintManifold::shared_ptr manifold() const {
    return std::static_pointer_cast<const IEConstraintManifold>(manifold_);
  }
  const IndexSet& activeIndices() const { return active_indices_; }

  /// Explicitly relax inequalities into a distinct equality-only type.
  ECMPoint equalityView() const;

  /// GTSAM chart operation. At corners, admissibility remains CMC's concern.
  IECMPoint retract(const Vector& delta) const;

  /// Retraction with explicit boundary and trial-metric context.
  IECMPoint retract(const Vector& delta, const std::optional<IndexSet>& blocking,
                  IERetractionInfo* info = nullptr,
                  const VectorValues* metricSigmas = nullptr) const;

  /// Ambient-update entry point for existing feasibility algorithms.
  IECMPoint retract(const VectorValues& delta,
                  const std::optional<IndexSet>& blocking = {},
                  IERetractionInfo* info = nullptr,
                  const VectorValues* metricSigmas = nullptr) const;

  /// Chart coordinates require the same type identity and bindings.
  Vector localCoordinates(const IECMPoint& other) const;

  /// Compute cone-feasible reduced direction and original block indices.
  std::pair<IndexSet, Vector> projectTangentCone(const Vector& delta) const;

  /// Ambient version of cone projection.
  std::pair<IndexSet, VectorValues> projectTangentCone(
      const VectorValues& delta) const;

  /// Current inequalities blocking an ambient direction.
  IndexSet blockingIndices(const VectorValues& direction) const;

  /// Move to a selected boundary using the type's chosen policy.
  IECMPoint moveToBoundary(const IndexSet& active,
                         IERetractionInfo* info = nullptr,
                         const VectorValues* metricSigmas = nullptr) const;

  /// Expression of a complete inequality block, including inactive rows.
  Vector inequalityResidual(size_t index) const;

  /// Bound factor views for feasibility implementations; definitions stay
  /// local.
  std::shared_ptr<const NonlinearEqualityConstraints> eConstraints() const {
    return equalities_;
  }

  /// Bound inequality view with preserved block ordering and sigmas.
  std::shared_ptr<const NonlinearInequalityConstraints> iConstraints() const {
    return inequalities_;
  }

  /// Whitened nonlinear equality violation.
  double evaluateEqualityViolation() const;

  /// Whitened nonlinear inequality violation.
  double evaluateInequalityViolation() const;

  /// Metric cache belongs to this point's geometry, not its shared type.
  TangentSpaceMetric::shared_ptr projectionMetric(
      const IERetractorParams& params,
      const std::optional<KeyVector>& priorKeys = {}) const;

  /// Create an isolated retraction workspace, never a shared policy instance.
  IERetractor::shared_ptr retractor(
      const VectorValues* metricSigmas = nullptr) const;

 private:
  friend class IEConstraintManifold;
  IndexSet active_indices_;
  NonlinearEqualityConstraints::shared_ptr equalities_;
  NonlinearInequalityConstraints::shared_ptr inequalities_;
};

/// Component-point keys are independent of ambient keys.
using IECMPoints = std::map<Key, IECMPoint>;
/// Gather the authoritative ambient values of disjoint points.
Values ambientValues(const IECMPoints& points);
/// Component-point keys in deterministic order.
KeyVector pointKeys(const IECMPoints& points);
/// Move selected points to their boundaries without changing the originals.
IECMPoints movePointsToBoundaries(const IECMPoints& points,
                                const IndexSetMap& indices,
                                const VectorValues* metricSigmas = nullptr);
}  // namespace gtdynamics
namespace gtsam {
template <>
struct traits<gtdynamics::IECMPoint> : internal::Manifold<gtdynamics::IECMPoint> {};
}  // namespace gtsam
