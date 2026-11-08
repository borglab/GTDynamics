/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file FixedDynamicsBenchmark.h
 * @brief Read-only benchmark view of the production fixed-q/v model builders.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#pragma once

#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics::internal {
// Published read-only; legacy comparisons cannot mutate the conditioning.
struct FixedDynamicsBenchmark {
  fixed_dynamics::Diagnostics diagnostics;
  gtsam::NonlinearFactorGraph cost, rootBalance, effort, wrench, torqueSmooth,
      wrenchSmooth, acceleration;
  gtsam::NonlinearEqualityConstraints equalities;
  gtsam::NonlinearInequalityConstraints originalInequalities;
  gtsam::Values initial;
  gtsam::KeySet independentKeys;
};

std::shared_ptr<const FixedDynamicsBenchmark> prepareFixedDynamicsBenchmark(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& fixedValues, const gtsam::Values& accelerationPriors,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const fixed_dynamics::Options& options);
}  // namespace gtdynamics::internal
