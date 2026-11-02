/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
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
  Contact() = default;
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
  size_t maxIterations = 50;
  gtsam::Vector3 gravity = gtsam::Vector3(0, 0, -9.81);
  double effortSigma = 100.0;
  double wrenchSigma = 500.0;
  double torqueSmoothSigma = 100.0;
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
  double balanceMomentSigma = 10.0;
  double balanceForceSigma = 100.0;
  /// Optional symmetric torque limits, indexed by joint id. Empty disables.
  gtsam::Vector torqueLimits;
};

/** Native CMC-Opt result. Keys are slice-major: contactKeys follows contacts,
 * torqueKeys follows jointIds (ascending). values contains inferred link twist
 * derivatives, joint accelerations, wrenches, and torques. Input Values are
 * never altered. baseResiduals contains one
 * unwhitened [moment; force] Newton-Euler residual per slice in root COM axes.
 * It is zero to numerical precision in strict mode.
 */
struct Result {
  gtsam::Values values;
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
  double projectedGradientNorm = 0.0;
  double assemblySeconds = 0.0;
  double initializationSeconds = 0.0;
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
}  // namespace fixed_dynamics

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
