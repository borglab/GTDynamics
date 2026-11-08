/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file ManualIEConstraintManifold.h
 * @brief A reusable nonnegative line, y=2x, for explicit type construction.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#pragma once

#include <gtdynamics/cmcopt/CMPoint.h>
#include <gtdynamics/cmcopt/ReducedALRetractor.h>
#include <gtsam/constrained/LinearConstraint.h>

namespace manual_ieconstraint_manifold {

using namespace gtsam;
using namespace gtdynamics;

/** Ambient vector-valued scalar slots for y=2x. */
inline Values lineValues(Key x, Key y, double value) {
  Values result;
  result.insert(x, Vector(Vector1(value)));
  result.insert(y, Vector(Vector1(2 * value)));
  return result;
}

/** Exact linear equality and nonnegative-x inequality using constraint factors.
 */
inline std::pair<NonlinearEqualityConstraints, NonlinearInequalityConstraints>
lineConstraints(Key x, Key y) {
  NonlinearEqualityConstraints equalities;
  equalities.push_back(LinearConstraint::Equal(
                           JacobianFactor(x, Matrix1::Constant(-2), y,
                                          Matrix1::Identity(), Vector1::Zero()))
                           .createEqualityFactor());

  // Keep the inequality sigma explicit to exercise the usual row scaling.
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      LinearConstraint::GreaterEqual(
          JacobianFactor(x, Matrix1::Identity(), Vector1::Zero()), Vector1(2))
          .createInequalityFactor());
  return {equalities, inequalities};
}

/** A single type whose local slots 0/1 can be bound to any slice's x/y keys. */
inline IEConstraintManifold::shared_ptr lineType() {
  const auto [equalities, inequalities] = lineConstraints(0, 1);

  // Use x as the chart coordinate and y as its recovered dependent value.
  IEConstraintManifold::Params params;
  params.affineEqualities = true;
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{0}; });
  params.retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedALRetractor>(
          std::make_shared<IERetractorParams>()));
  return std::make_shared<IEConstraintManifold>(equalities, inequalities,
                                                lineValues(0, 1, 0), params);
}

}  // namespace manual_ieconstraint_manifold
