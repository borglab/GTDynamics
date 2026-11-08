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
#include <gtdynamics/constrained/ConstraintManifold.h>
#include <gtdynamics/constrained/TangentSpaceBasis.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>

#include <typeindex>
#include <mutex>

namespace gtdynamics {

class IECMPoint;
class CMTangentSpace;

/** A runtime type, independent of any trajectory keys or numerical point.
 * Factor keys name local slots. Bindings on IECMPoint assign ambient identities.
 * Constraint factors and captured configuration must remain immutable.
 */
class IEConstraintManifold : public ConstraintManifold,
    public std::enable_shared_from_this<IEConstraintManifold> {
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
  IEConstraintManifold(const IEConstraintManifold&) = default;
  // Runtime definitions are immutable, including through a base reference.
  IEConstraintManifold& operator=(const IEConstraintManifold&) = delete;
  virtual ~IEConstraintManifold() = default;

  /// Construct a point with complete one-to-one bindings and ambient values.
  IECMPoint point(const Bindings& bindings, const Values& values,
                const std::optional<IndexSet>& active = {}) const;

  /// Construct a point whose ambient keys coincide with the local slots.
  IECMPoint point(const Values& values,
                const std::optional<IndexSet>& active = {}) const;

  /// Fix local slots immutably, producing a distinct member of the type family.
  shared_ptr condition(const Values& fixed) const;

  /// Compute geometry at a point; no point-dependent data is stored in the
  /// type.
  std::shared_ptr<const CMTangentSpace> linearize(const IECMPoint& point) const;

  /// Apply this type's retraction, with isolated mutable projection work.
  virtual IECMPoint retract(const IECMPoint& point, const Vector& delta,
                          const std::optional<IndexSet>& blocking = {},
                          IERetractionInfo* info = nullptr,
                          const VectorValues* metricSigmas = nullptr) const;

  /// Ambient-update path retained by feasibility projection policies.
  virtual IECMPoint retractAmbient(
      const IECMPoint& point, const VectorValues& delta,
      const std::optional<IndexSet>& blocking = {},
      IERetractionInfo* info = nullptr,
      const VectorValues* metricSigmas = nullptr) const;

  /// Explicit boundary move, distinct from the ordinary GTSAM chart operation.
  virtual IECMPoint moveToBoundary(
      const IECMPoint& point, const IndexSet& active,
      IERetractionInfo* info = nullptr,
      const VectorValues* metricSigmas = nullptr) const;

  /// Local chart coordinates; IECMPoint checks type and binding compatibility.
  virtual Vector localCoordinates(const IECMPoint& origin,
                                  const IECMPoint& other) const;

  /// Nonlinear constraint operations also supported by compiled definitions.
  virtual IndexSet identifyActive(
      const IECMPoint& point, const std::optional<IndexSet>& supplied = {}) const;

  /// Number of nonlinear inequality blocks, independent of factor storage.
  virtual size_t inequalityCount(const IECMPoint& point) const;

  /// Raw complete inequality expression, independent of its active status.
  virtual Vector inequalityResidual(const IECMPoint& point, size_t index) const;

  /// Whitened equality violation of a point.
  virtual double equalityViolation(const IECMPoint& point) const;

  /// Whitened inequality violation of a point.
  virtual double inequalityViolation(const IECMPoint& point) const;

  /// Dimension of the equality chart, unchanged by boundary activation.
  size_t dim() const override { return dimension_; }
  /// Local variable identities and dimensions.
  std::map<Key, size_t> dimensions() const override;

  /// Captured affine contract.
  bool affineEqualities() const override { return params_.affineEqualities; }
  /// Whether the projection metric is supplied by the current LM trial.
  bool usesVaryingMetric() const;

  ConstraintManifold::shared_ptr conditionBase(const Values& fixed) const override {
    return condition(fixed);
  }
  std::shared_ptr<const CMTangentSpace> linearize(const CMPoint& point) const override;
  Values retractValues(const CMPoint& point, const Vector& delta) const override;
  /// Explicitly forget inequalities by constructing a distinct equality type.
  std::shared_ptr<const class EConstraintManifold> equalityView() const;

 protected:
  /// Equality-only CM historically treats saturated layouts as fixed charts.
  enum class EqualityLayout { Ordinary, EqualityOnly };
  IEConstraintManifold(const NonlinearEqualityConstraints& equalities,
      const NonlinearInequalityConstraints& inequalities,
      const Values& prototype, const Params& params, EqualityLayout layout);

  /// Reject direct kernel calls with points owned by a different runtime type.
  void validatePointType(const CMPoint& point) const;
  virtual std::shared_ptr<IEConstraintManifold> cloneDefinition() const;

  /// Build active geometry using this type's prepared local equality structure.
  virtual std::shared_ptr<const CMTangentSpace> linearizeAt(
      const IECMPoint& point) const;

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
  bool equalityOnlyLayout_ = false;

  /// Minimal constructor for native implementations with no factor storage.
  IEConstraintManifold(const Values& prototype, size_t dimension);

 private:
  friend class IECMPoint;
  friend class EConstraintManifold;
  struct EqualityPreparation {
    std::shared_ptr<const TangentSpaceBasis> structure;
    std::once_flag affineOnce;
    std::shared_ptr<const TangentSpaceBasis> affineBasis;
  };
  std::shared_ptr<EqualityPreparation> equalityPreparation_;
  void prepareEqualities();
  std::shared_ptr<const TangentSpaceBasis> localEqualityBasis(
      const IECMPoint& point) const;
  void validate(const Bindings& bindings, const Values& values) const;
  std::pair<NonlinearEqualityConstraints::shared_ptr,
            NonlinearInequalityConstraints::shared_ptr>
  boundConstraints(const Bindings& bindings) const;
};

}  // namespace gtdynamics
