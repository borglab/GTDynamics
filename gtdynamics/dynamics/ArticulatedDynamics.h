/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ArticulatedDynamics.h
 * @brief Shared articulated mechanics assembly for fixed and free kinematics.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
namespace gtdynamics {
namespace internal {
/** Internal shared assembly of Newton-Euler balance, acceleration kinematics,
 * reaction-wrench transmission, armature torque, sole transforms and cones.
 * q/v may be substituted for fixed dynamics or retained for coupled solves.
 * The independentKeys field selects acceleration/contact-wrench coordinates;
 * callers retaining q/v must add their configuration/velocity coordinates.
 */
struct ArticulatedDynamicsProblem {
  gtsam::Values zero;
  gtsam::NonlinearFactorGraph cost, effort, wrench, torqueSmooth, wrenchSmooth,
      balance, acceleration;
  gtsam::NonlinearFactorGraph rootBalance;
  gtsam::NonlinearEqualityConstraints equalities;
  gtsam::NonlinearInequalityConstraints inequalities;
  gtsam::KeySet independentKeys;
  fixed_dynamics::Result result;

  /** Build the same mechanics for both fixed and optimized q/v. */
  ArticulatedDynamicsProblem(
      const Robot& robot, const Trajectory& trajectory,
      const std::vector<Slice>& slices, const std::vector<double>& times,
      const gtsam::Values& fixedValues, const gtsam::Values& accelerationPriors,
      const std::vector<fixed_dynamics::Contact>& contacts,
      const gtsam::Vector& armature, const fixed_dynamics::Options& options,
      bool fixKinematics = true);
};
}  // namespace internal
}  // namespace gtdynamics
