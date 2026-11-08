/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file CMPoint.h
 * @brief Values of runtime constraint-manifold types.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/CMTangentSpace.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmopt/TangentSpaceMetric.h>
#include <gtsam/base/Manifold.h>

namespace gtdynamics {
/** An immutable point with ambient identities and ordinary vector tangents.
 * Copies share read-only geometry. Successors preserve the runtime type and
 * bindings, and rebuild nonlinear geometry. Retraction work is never shared.
 */
class CMPoint {
 public:
  enum { dimension = Eigen::Dynamic };
  using shared_ptr = std::shared_ptr<CMPoint>;
  using Bindings = IEConstraintManifold::Bindings;

  /// Empty value for GTSAM concept/default construction, not an optimizable
  /// point.
  CMPoint() = default;

  /** Convenience construction of one factor-defined type and its initial point.
   * Reuse a IEConstraintManifold explicitly when several points share a
   * definition.
   */
  CMPoint(
      const IEConstraintManifold::Params::shared_ptr& params,
      const std::shared_ptr<const NonlinearEqualityConstraints>& equalities,
      const std::shared_ptr<const NonlinearInequalityConstraints>& inequalities,
      const Values& values, const std::optional<IndexSet>& active = {});

  /// Rebuild a value of the same type without modifying its origin.
  CMPoint createWithNewValues(const Values& values,
                              const std::optional<IndexSet>& active = {}) const;

  /// Ambient numerical state is authoritative.
  const Values& values() const { return values_; }
  /// Runtime type identity, including immutable conditioning.
  const IEConstraintManifold::shared_ptr& manifold() const { return manifold_; }
  /// Local-slot to ambient-key assignment.
  const Bindings& bindings() const { return bindings_; }
  /// Current tight constraint blocks, never stored on the shared type.
  const IndexSet& activeIndices() const { return active_indices_; }
  /// Geometry at this point, shared read-only across trials.
  const std::shared_ptr<const CMTangentSpace>& tangentSpace() const {
    return tangentSpace_;
  }

  /// Equality basis in ambient-key coordinates.
  const std::shared_ptr<const TangentSpaceBasis>& eBasis() const;

  /// Captured affine contract.
  bool hasAffineEqualities() const { return manifold_->affineEqualities(); }
  /// Intrinsic equality dimension, independent of the active boundary.
  size_t dim() const { return manifold_ ? manifold_->dim() : 0; }
  /// Map reduced increments to ambient tangents.
  VectorValues toAmbient(const Vector& delta) const;

  /// Map ambient tangents to reduced increments.
  Vector toCoordinates(const VectorValues& delta) const;

  /// GTSAM chart operation. At corners, admissibility remains CMC's concern.
  CMPoint retract(const Vector& delta) const;

  /// Retraction with explicit boundary and trial-metric context.
  CMPoint retract(const Vector& delta, const std::optional<IndexSet>& blocking,
                  IERetractionInfo* info = nullptr,
                  const VectorValues* metricSigmas = nullptr) const;

  /// Ambient-update entry point for existing feasibility algorithms.
  CMPoint retract(const VectorValues& delta,
                  const std::optional<IndexSet>& blocking = {},
                  IERetractionInfo* info = nullptr,
                  const VectorValues* metricSigmas = nullptr) const;

  /// Chart coordinates require the same type identity and bindings.
  Vector localCoordinates(const CMPoint& other) const;

  /// Numerical equality includes type identity and ambient bindings.
  bool equals(const CMPoint& other, double tolerance = 1e-8) const;

  /// Print this point and its ambient numerical state.
  void print(const std::string& prefix = "") const;

  /// Recover an ambient value, optionally with its reduced recovery Jacobian.
  const Value& recover(
      Key key, OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) const;

  /// Compute cone-feasible reduced direction and original block indices.
  std::pair<IndexSet, Vector> projectTangentCone(const Vector& delta) const;

  /// Ambient version of cone projection.
  std::pair<IndexSet, VectorValues> projectTangentCone(
      const VectorValues& delta) const;

  /// Current inequalities blocking an ambient direction.
  IndexSet blockingIndices(const VectorValues& direction) const;

  /// Move to a selected boundary using the type's chosen policy.
  CMPoint moveToBoundary(const IndexSet& active,
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
  IEConstraintManifold::shared_ptr manifold_;
  Bindings bindings_;
  Values values_;
  IndexSet active_indices_;
  NonlinearEqualityConstraints::shared_ptr equalities_;
  NonlinearInequalityConstraints::shared_ptr inequalities_;
  std::shared_ptr<const CMTangentSpace> tangentSpace_;
  // Frozen preparation only; every invocation clones/detaches mutable work.
  std::shared_ptr<const IERetractor> retractionPreparation_;
  CMPoint(IEConstraintManifold::shared_ptr manifold, const Bindings& bindings,
          const Values& values, const std::optional<IndexSet>& active,
          const CMPoint* previous = nullptr);
};

/// Component-point keys are independent of ambient keys.
using CMPoints = std::map<Key, CMPoint>;
/// Gather the authoritative ambient values of disjoint points.
Values ambientValues(const CMPoints& points);
/// Component-point keys in deterministic order.
KeyVector pointKeys(const CMPoints& points);
/// Move selected points to their boundaries without changing the originals.
CMPoints movePointsToBoundaries(const CMPoints& points,
                                const IndexSetMap& indices,
                                const VectorValues* metricSigmas = nullptr);
}  // namespace gtdynamics
namespace gtsam {
template <>
struct traits<gtdynamics::CMPoint> : internal::Manifold<gtdynamics::CMPoint> {};
}  // namespace gtsam
