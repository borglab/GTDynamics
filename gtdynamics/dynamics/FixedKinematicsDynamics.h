/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file FixedKinematicsDynamics.h
 * @brief Estimate accelerations and wrenches with fixed poses and velocities.
 * @author GTDynamics contributors
 */

#pragma once

#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PoseOnLink.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/nonlinear/Values.h>

namespace gtdynamics {
class Trajectory;

namespace fixed_dynamics {

/** One material sole frame and its conservative rectangular support patch.
 * Wrenches are [moment; force], in N m and N, expressed at the contact origin
 * in contact axes (+Z out of the sole). Lengths are metres. The rectangular
 * friction pyramid is inscribed in the circular Coulomb cone.
 */
struct Contact {
  PoseOnLink pose;
  double halfLength = 0.10;
  double halfWidth = 0.035;
  double frictionCoefficient = 0.6;
  /// Default-construct a contact for subsequent assignment of its sole frame.
  Contact() = default;
  /** Associate a material sole frame with its rectangular support patch.
   * Half extents are metres and frictionCoefficient is dimensionless. Patch
   * dimensions and friction must be positive and finite when solving dynamics.
   */
  Contact(const PoseOnLink& pose, double halfLength, double halfWidth,
          double frictionCoefficient)
      : pose(pose),
        halfLength(halfLength),
        halfWidth(halfWidth),
        frictionCoefficient(frictionCoefficient) {}
};

/** Parameters for dynamics with fixed configurations and velocities.
 * Link twist derivatives and joint accelerations remain optimization variables.
 * Gaussian priors align independent root/joint accelerations with derivatives
 * of the fitted velocity signal. Native acceleration kinematics determine all
 * other link accelerations exactly.
 * Temporal residuals are differences / sqrt(actual dt), i.e. discretizations
 * of an integral squared-rate penalty. No forces or torques are interpolated
 * in this optimization. Wrench moment scales use each patch's characteristic
 * length to convert moments into force-equivalent residuals.
 */
struct Options {
  /// Maximum outer CMC iterations.
  size_t maxIterations = 50;
  /// Gravity acceleration in world axes, m/s^2.
  gtsam::Vector3 gravity = gtsam::Vector3(0, 0, -9.81);
  /// Per-actuator torque prior standard deviation, N m.
  double effortSigma = 100.0;
  /// Force-equivalent wrench sigma, N; moments scale by patch length.
  double wrenchSigma = 500.0;
  /// Torque temporal-difference scale, N m / sqrt(s).
  double torqueSmoothSigma = 100.0;
  /// Force-equivalent wrench temporal-difference scale, N / sqrt(s).
  double wrenchSmoothSigma = 500.0;
  /// Root body-twist derivative prior scales: degrees/s^2 and m/s^2 per axis.
  double baseAngularAccelerationSigmaDegrees = 90.0;
  double baseLinearAccelerationSigma = 3.0;
  /// Revolute-joint acceleration prior standard deviation, degrees/s^2.
  double jointAccelerationSigmaDegrees = 100.0;
  /// Explicit fallback: only the floating-root force balance becomes soft.
  /// Configurations/velocities stay fixed. Acceleration kinematics, other link
  /// balances, and wrench transmissions remain exact. Residuals are reported.
  bool softDynamics = false;
  /// Per-axis soft root moment-balance standard deviation, N m.
  double balanceMomentSigma = 10.0;
  /// Per-axis soft root force-balance standard deviation, N.
  double balanceForceSigma = 100.0;
  /// Optional symmetric torque limits, indexed by joint id. Empty disables.
  gtsam::Vector torqueLimits;
};

/** Owning mechanics and solver diagnostics, without a state Values map.
 * Keys are slice-major: contactKeys follows contacts and torqueKeys follows
 * jointIds (ascending). baseResiduals contains one
 * unwhitened [moment; force] Newton-Euler residual per slice in root COM axes.
 * It is zero to numerical precision in strict mode.
 * Error fields are half the sum of squared whitened residuals. A linear seed
 * reports only acceleration-prior costs; effort and temporal costs are absent.
 * Timing fields are seconds; dimensions count scalar coordinates or rows.
 */
struct Diagnostics {
  std::vector<gtsam::Key> contactKeys;
  std::vector<gtsam::Key> torqueKeys;
  std::vector<size_t> jointIds;
  std::vector<gtsam::Vector6> baseResiduals;
  std::vector<double> objectiveHistory;
  double objectiveInitial = 0.0;
  double objectiveFinal = 0.0;
  double effortError = 0.0;
  double wrenchError = 0.0;
  double torqueSmoothError = 0.0;
  double wrenchSmoothError = 0.0;
  double balanceError = 0.0;
  double accelerationError = 0.0;
  double equalityL2 = 0.0;
  double inequalityL2 = 0.0;
  double maxForceResidual = 0.0;
  double maxMomentResidual = 0.0;
  /// Final negative objective gradient projected into the feasible tangent
  /// cone, measured in independent acceleration and sole-wrench coordinates.
  /// Infinity for a linear seed, whose stationarity is not evaluated.
  double projectedGradientNorm = 0.0;
  double assemblySeconds = 0.0;
  /// Complete equality-anchor construction, solve and value assembly.
  double initializationSeconds = 0.0;
  /// Assembly of the constrained Gaussian system, including linearization.
  double linearizationSeconds = 0.0;
  /// Gaussian QR optimization only, excluding system and Values assembly.
  double linearSolveSeconds = 0.0;
  /// Reconstruction of typed Values from the Gaussian solution.
  double valueAssemblySeconds = 0.0;
  /// Physical constraint checks, root residuals and applicable prior errors.
  double validationSeconds = 0.0;
  double manifoldSeconds = 0.0;
  double optimizationSeconds = 0.0;
  size_t iterations = 0;
  size_t variableDimension = 0;
  size_t equalityDimension = 0;
  size_t inequalityDimension = 0;
  size_t reducedDimension = 0;
  size_t numALSolves = 0;
  size_t numFeasibleTrials = 0;
  size_t numFeasibleProjections = 0;
  size_t wrenchSmoothFactors = 0;
  bool softDynamics = false;
  bool feasible = false;
  /// Feasible and projected-gradient norm <= 1e-5, independently of LM
  /// stopping.
  bool stationary = false;
};

/** Owning numerical result. values contains link twist derivatives, joint
 * accelerations, sole/reaction wrenches and torques. Input Values never change.
 * Separate diagnostics allow composed pipelines to keep one combined map.
 */
struct Result : Diagnostics {
  gtsam::Values values;
};
}  // namespace fixed_dynamics

/** Construct the affine equality-feasible dynamics seed used by the full
 * solver. Input frames, keys and units match solveFixedKinematicsDynamics.
 * Stance sole wrenches are fixed to equal positive normal shares of mass *
 * |gravity|; swing wrenches are zero. One constrained Gaussian QR solve
 * minimizes only acceleration-prior error subject to those loads and exact
 * mechanics. Effort/wrench/rate objectives are not constructed for the seed.
 * There is no manifold construction, cone projection or iterative dynamics
 * optimization. Throws for softDynamics or infeasible mechanics/wrench/optional
 * torque limits; it does not repair a seed that fails those checks. Results
 * have zero optimizer iterations/projections and stationary=false. Assembly,
 * Gaussian solve and residual checking are distinct from subsequent coupled
 * optimization.
 */
fixed_dynamics::Result fixedKinematicsDynamicsSeed(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& fixedValues, const gtsam::Values& accelerationPriors,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const fixed_dynamics::Options& options);

/** Infer accelerations, sole wrenches, and actuated joint torques over a
 * trajectory using IELMOptimizer, EliminationBasis, and ReducedALRetractor.
 * Fixed poses map COM axes into world; twists are body-fixed at each COM,
 * angular first. fixedValues supplies PoseKey/TwistKey for every link and
 * JointAngleKey/JointVelKey for every joint, without acceleration keys.
 * accelerationPriors supplies TwistAccelKey for the root and JointAccelKey for
 * each joint at every slice. These entries are soft prior means, preferably
 * obtained by locally differentiating a Gaussian-process velocity fit.
 * A twist derivative is d[omega_body; v_body]/dt: physical linear acceleration
 * in body axes is vdot_body + omega_body cross v_body. Joint means are rad/s^2.
 * times are strictly increasing seconds
 * and correspond exactly to slices. Contact activity comes from the original
 * trajectory's pose-contact phases; swing wrenches are exactly zero.
 * armature is empty (zero) or indexed by joint id, in kg m^2. Motor torque is
 * screw-axis projection of the child reaction wrench + armature * qddot.
 * Invalid inputs or infeasible exact dynamics throw; use softDynamics only
 * explicitly, and inspect the returned base residuals.
 */
fixed_dynamics::Result solveFixedKinematicsDynamics(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& fixedValues, const gtsam::Values& accelerationPriors,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const fixed_dynamics::Options& options);
}  // namespace gtdynamics
