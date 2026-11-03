/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file Kinodynamics.h
 * @brief Coupled floating-base trajectory estimation with CMC-Opt.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtdynamics/dynamics/FixedKinematicsDynamics.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {
namespace kinodynamics {
/** Noise scales for trapezoidal collocation. Configuration and body twists,
 * their derivatives, reaction/contact wrenches and joint torques are free.
 * All rotation standard deviations are specified in degrees and converted at
 * construction. Collocation is a soft cost; articulated mechanics and wrench
 * cones are exact constraints. No reference trajectory observations are added.
 */
struct Options {
  size_t maxIterations = 50;
  size_t gaugeSlice =
      0;  ///< Fix the supplied root pose at this existing slice.
  size_t retractionThreads = 8;  ///< Independent per-slice projections; >=1.
  double poseRotationSigmaDegrees = 0.05;
  double poseTranslationSigma = 0.0005;   ///< metres per interval.
  double twistAngularSigmaDegrees = 2.0;  ///< degrees/s per interval.
  double twistLinearSigma = 0.02;         ///< m/s per interval.
  double jointPositionSigmaDegrees = 0.05;
  double jointVelocitySigmaDegrees = 2.0;
  double stanceVelocitySigma = 0.1;  ///< Sole-origin linear speed, m/s.
};
/** The raw initial values have zero v/a/wrench/torque, including world COM
 * velocities. Feasibility restoration is included in initializationSeconds;
 * it necessarily changes those zeros before constrained optimization begins.
 * Scalar histories are recorded without retaining full trajectory iterates.
 * errorHistory contains half squared whitened residuals in errorNames order.
 * Time samples are elapsed optimizer wall seconds, not simulated clip time.
 */
struct Result {
  gtsam::Values initialValues;
  gtsam::Values values;
  std::vector<gtsam::Key> contactKeys;
  std::vector<gtsam::Key> torqueKeys;
  std::vector<size_t> jointIds;
  std::vector<std::string> errorNames;
  std::vector<gtsam::Vector> errorHistory;
  std::vector<double> wallSeconds;
  double assemblySeconds = 0;
  double initializationSeconds = 0;
  double optimizationSeconds = 0;
  double initialEqualityL2 = 0;
  double equalityL2 = 0;
  double inequalityL2 = 0;
  double objectiveInitial = 0;
  double objectiveFinal = 0;
  size_t iterations = 0;
  size_t variableDimension = 0;
  size_t reducedDimension = 0;
  size_t equalityDimension = 0;
  size_t inequalityDimension = 0;
  bool feasible = false;
};
}  // namespace kinodynamics
/** Jointly estimate the full state at every supplied slice. InitialValues must
 * contain measured joint angles and contact-initialized COM poses, plus initial
 * values of any unconstrained measurement variables (landmarks/shared bias).
 * Other state entries are created at zero. PoseKey = world_T_COM;
 * TwistKey = [omega_body; v_body] at the COM, TwistAccelKey is its ordinary
 * body-coordinate derivative. LinearVelocityKey is the same COM speed in world
 * axes, tied exactly to the body twist for Gal3 IMU factors. The root
 * pose at options.gaugeSlice is fixed as gauge. Each measurement group is a
 * soft factor graph with a matching label; contacts/armature use
 * solveFixedKinematicsDynamics units. Uses native articulated constraints and
 * trapezoidal root/joint collocation. Requires one floating tree, strictly
 * increasing matching slices/times, and at least one supporting sole per slice.
 * Inputs remain unchanged.
 */
kinodynamics::Result solveKinodynamics(
    const Robot& robot, const Trajectory& trajectory,
    const std::vector<Slice>& slices, const std::vector<double>& times,
    const gtsam::Values& initialValues,
    const std::vector<gtsam::NonlinearFactorGraph>& measurementGroups,
    const std::vector<std::string>& measurementNames,
    const std::vector<fixed_dynamics::Contact>& contacts,
    const gtsam::Vector& armature, const kinodynamics::Options& options);
}  // namespace gtdynamics
