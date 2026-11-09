/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file LegacyCMAdapter.h
 * @brief Isolate the temporary equality-only CM value dependency.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/CMPoint.h>
#include <gtdynamics/cmcopt/internal/ReadOnlyBasis.h>
#include <gtdynamics/cmopt/ConstraintManifold.h>

namespace gtdynamics::internal {

/** Access to compatibility settings never exposed through public point APIs. */
struct LegacyCMAccess {
  static bool basisFromScratch(const IEConstraintManifold& type) {
    return type.params_.equalityBasisBuildFromScratch;
  }

  static ConstraintManifold::Params::shared_ptr params(
      const IEConstraintManifold& type) {
    auto result = std::make_shared<ConstraintManifold::Params>();
    result->basisCreator = type.params_.equalityBasisCreator;
    result->retractorCreator = type.params_.equalityRetractionCreator;
    return result;
  }
};

/** Equality view used only inside existing feasibility algorithms. */
inline ConstraintManifold equalityPoint(
    const CMPoint& point, const IndexSet& active = {},
    const std::optional<bool>& fromScratch = {}) {
  auto constraints =
      std::make_shared<NonlinearEqualityConstraints>(*point.eConstraints());

  // Promote only the requested complete inequality blocks to equalities.
  NonlinearEqualityConstraints extra;
  for (size_t index : active)
    extra.push_back(
        point.iConstraints()->at(index)->createEqualityConstraint());
  constraints->add(extra);

  // Preserve the point's chart unless the compatibility view adds a boundary.
  auto basis = active.empty()
                   ? std::make_shared<ReadOnlyBasis>(point.eBasis())
                   : point.eBasis()->createWithAdditionalConstraints(
                         extra, point.values(),
                         fromScratch.value_or(LegacyCMAccess::basisFromScratch(
                             *point.manifold())));
  return ConstraintManifold(constraints, point.values(),
                            LegacyCMAccess::params(*point.manifold()), false,
                            basis);
}

}  // namespace gtdynamics::internal
