/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file AdmmSplitting.h
 * @brief Splitting strategies for the CMC tangent-QP ADMM backend.
 * @author GTDynamics contributors
 */
#pragma once

namespace gtdynamics {

/// Select the auxiliary variables and projection used by ADMM.
enum class AdmmSplitting {
  /// Duplicate reduced manifold coordinates and project onto local cones.
  ManifoldVariables,
  /// Duplicate inequality outputs and clip to the nonpositive orthant.
  ConstraintOutputs
};

}  // namespace gtdynamics
