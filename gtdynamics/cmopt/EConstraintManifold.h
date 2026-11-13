/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file EConstraintManifold.h
 * @brief Equality-only specialization of a constrained runtime type.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once
#include <gtdynamics/cmcopt/IEConstraintManifold.h>

namespace gtdynamics {
class ECMPoint;

class EConstraintManifold : public IEConstraintManifold {
 public:
  using shared_ptr = std::shared_ptr<const EConstraintManifold>;
  using Params = IEConstraintManifold::Params;
  EConstraintManifold(const NonlinearEqualityConstraints& equalities,
                       const Values& prototype, const Params& params);
  EConstraintManifold(const NonlinearEqualityConstraints& equalities,
                       const Values& prototype);
  ECMPoint point(const Bindings& bindings, const Values& values,
                 const std::optional<IndexSet>& active = {}) const;
  ECMPoint point(const Values& values,
                 const std::optional<IndexSet>& active = {}) const;
  shared_ptr condition(const Values& fixed) const;
  static shared_ptr fromEqualityView(const IEConstraintManifold& source);

  // The leaf guarantee also applies to compiled subclasses and conditioning.
  IndexSet identifyActive(
      const IECMPoint& point,
      const std::optional<IndexSet>& supplied = {}) const final;
  size_t inequalityCount(const IECMPoint&) const final { return 0; }
  Vector inequalityResidual(const IECMPoint&, size_t) const final;
  double inequalityViolation(const IECMPoint&) const final { return 0; }

 private:
  explicit EConstraintManifold(const IEConstraintManifold& source);

 protected:
  EConstraintManifold(const Values& prototype, size_t dimension)
      : IEConstraintManifold(prototype, dimension) {}
  std::shared_ptr<IEConstraintManifold> cloneDefinition() const override;
};
}  // namespace gtdynamics
