/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file IEConstraintManifold.h
 * @brief Immutable runtime definitions of constraint manifolds with corners.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmopt/Retractor.h>
#include <gtdynamics/cmopt/TangentSpaceBasis.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>

#include <typeindex>
#include <mutex>

namespace gtdynamics {
namespace internal {
struct LegacyCMAccess;
}

class CMPoint;
class CMTangentSpace;

/** A runtime type, independent of any trajectory keys or numerical point.
 * Factor keys name local slots. Bindings on CMPoint assign ambient identities.
 * Constraint factors and captured configuration must remain immutable.
 */
class IEConstraintManifold : public std::enable_shared_from_this<IEConstraintManifold> {
 public:
  using shared_ptr = std::shared_ptr<const IEConstraintManifold>;
  using Bindings = std::map<Key, Key>;

  /** Construction choices, copied when the definition is created. */
  struct Params {
    using shared_ptr = std::shared_ptr<Params>;
    TangentSpaceBasisCreator::shared_ptr equalityBasisCreator =
        std::make_shared<OrthonormalBasisCreator>();
    /// Feasibility retraction policy, frozen when the runtime type is built.
    IERetractorCreator::shared_ptr retractorCreator;
    /// Temporary equality-only compatibility adapter policy.
    RetractorCreator::shared_ptr equalityRetractionCreator;
    /// Rebuild augmented equality geometry in the legacy compatibility adapter.
    bool equalityBasisBuildFromScratch = true;
    /// Explicit certification of constant affine equalities in Euclidean slots.
    bool affineEqualities = false;
  };

  /** Construct a factor-defined type. prototype supplies slot types/dimensions;
   * its numerical coordinates are not retained. Empty inequalities are valid.
   */
  IEConstraintManifold(const NonlinearEqualityConstraints& equalities,
            const NonlinearInequalityConstraints& inequalities,
            const Values& prototype, const Params& params);

  /// Construct with default basis and retraction policies.
  IEConstraintManifold(const NonlinearEqualityConstraints& equalities,
            const NonlinearInequalityConstraints& inequalities,
            const Values& prototype);
  virtual ~IEConstraintManifold() = default;

  /// Construct a point with complete one-to-one bindings and ambient values.
  CMPoint point(const Bindings& bindings, const Values& values,
                const std::optional<IndexSet>& active = {}) const;

  /// Construct a point whose ambient keys coincide with the local slots.
  CMPoint point(const Values& values,
                const std::optional<IndexSet>& active = {}) const;

  /// Fix local slots immutably, producing a distinct member of the type family.
  shared_ptr condition(const Values& fixed) const;

  /// Compute geometry at a point; no point-dependent data is stored in the
  /// type.
  std::shared_ptr<const CMTangentSpace> linearize(const CMPoint& point) const;

  /// Apply this type's retraction, with isolated mutable projection work.
  virtual CMPoint retract(const CMPoint& point, const Vector& delta,
                          const std::optional<IndexSet>& blocking = {},
                          IERetractionInfo* info = nullptr,
                          const VectorValues* metricSigmas = nullptr) const;

  /// Ambient-update path retained by feasibility projection policies.
  virtual CMPoint retractAmbient(
      const CMPoint& point, const VectorValues& delta,
      const std::optional<IndexSet>& blocking = {},
      IERetractionInfo* info = nullptr,
      const VectorValues* metricSigmas = nullptr) const;

  /// Explicit boundary move, distinct from the ordinary GTSAM chart operation.
  virtual CMPoint moveToBoundary(
      const CMPoint& point, const IndexSet& active,
      IERetractionInfo* info = nullptr,
      const VectorValues* metricSigmas = nullptr) const;

  /// Local chart coordinates; CMPoint checks type and binding compatibility.
  virtual Vector localCoordinates(const CMPoint& origin,
                                  const CMPoint& other) const;

  /// Nonlinear constraint operations also supported by compiled definitions.
  virtual IndexSet identifyActive(
      const CMPoint& point, const std::optional<IndexSet>& supplied = {}) const;

  /// Number of nonlinear inequality blocks, independent of factor storage.
  virtual size_t inequalityCount(const CMPoint& point) const;

  /// Raw complete inequality expression, independent of its active status.
  virtual Vector inequalityResidual(const CMPoint& point, size_t index) const;

  /// Whitened equality violation of a point.
  virtual double equalityViolation(const CMPoint& point) const;

  /// Whitened inequality violation of a point.
  virtual double inequalityViolation(const CMPoint& point) const;

  /// Dimension of the equality chart, unchanged by boundary activation.
  size_t dim() const { return dimension_; }
  /// Local variable identities and dimensions.
  std::map<Key, size_t> dimensions() const;

  /// Captured affine contract.
  bool affineEqualities() const { return params_.affineEqualities; }
  /// Whether the projection metric is supplied by the current LM trial.
  bool usesVaryingMetric() const;

 protected:
  /// Build active geometry using this type's prepared local equality structure.
  virtual std::shared_ptr<const CMTangentSpace> linearizeAt(
      const CMPoint& point) const;

  /// Slot metadata; no sample numerical state is retained.
  struct Slot {
    std::type_index type;
    size_t dimension;
  };
  Params params_;
  std::map<Key, Slot> slots_;
  std::shared_ptr<const NonlinearEqualityConstraints> equalities_;
  std::shared_ptr<const NonlinearInequalityConstraints> inequalities_;
  Values fixed_;
  size_t dimension_ = 0;

  /// Minimal constructor for native implementations with no factor storage.
  IEConstraintManifold(const Values& prototype, size_t dimension);

 private:
  friend class CMPoint;
  friend struct internal::LegacyCMAccess;
  struct EqualityPreparation {
    std::shared_ptr<const TangentSpaceBasis> structure;
    std::once_flag affineOnce;
    std::shared_ptr<const TangentSpaceBasis> affineBasis;
  };
  std::shared_ptr<EqualityPreparation> equalityPreparation_;
  void prepareEqualities();
  std::shared_ptr<const TangentSpaceBasis> localEqualityBasis(
      const CMPoint& point) const;
  void validate(const Bindings& bindings, const Values& values) const;
  std::pair<NonlinearEqualityConstraints::shared_ptr,
            NonlinearInequalityConstraints::shared_ptr>
  boundConstraints(const Bindings& bindings) const;
};

}  // namespace gtdynamics
