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
  /// Maximum accepted/current outer LM iterations; the example uses five.
  size_t maxIterations = 50;
  /// Existing Slice::k whose supplied world root pose fixes all six gauge axes.
  size_t gaugeSlice = 0;
  /// Maximum concurrent independent per-slice projections; at least one.
  size_t retractionThreads = 8;
  /// Per-axis root orientation collocation standard deviation, degrees.
  double poseRotationSigmaDegrees = 0.05;
  /// Per-axis root translation collocation standard deviation, metres.
  double poseTranslationSigma = 0.0005;
  /// Root angular-velocity collocation standard deviation, degrees/s per axis.
  double twistAngularSigmaDegrees = 2.0;
  /// Root linear-velocity collocation standard deviation, m/s per axis.
  double twistLinearSigma = 0.02;
  /// Revolute-joint position collocation standard deviation, degrees.
  double jointPositionSigmaDegrees = 0.05;
  /// Revolute-joint velocity collocation standard deviation, degrees/s.
  double jointVelocitySigmaDegrees = 2.0;
  /// Per-axis stance sole-origin linear velocity standard deviation, m/s.
  double stanceVelocitySigma = 0.1;
};
/** The raw initial values have zero v/a/wrench/torque, including world COM
 * velocities. Feasibility restoration is included in initializationSeconds;
 * it necessarily changes those zeros before constrained optimization begins.
 * Scalar histories are recorded without retaining full trajectory iterates.
 * errorHistory contains half squared whitened residuals in errorNames order.
 * Time samples are elapsed optimizer wall seconds, not simulated clip time.
 */
struct Result {
  /// Raw contact-initialized configurations with all dynamics entries zero.
  gtsam::Values initialValues;
  /// Final independent and dependent variables, with native DynamicsSymbol
  /// keys.
  gtsam::Values values;
  /// Slice-major sole wrench keys, following the input contacts order.
  std::vector<gtsam::Key> contactKeys;
  /// Slice-major motor torque keys, following ascending jointIds order.
  std::vector<gtsam::Key> torqueKeys;
  /// Ascending actuated joint ids; independent of Robot::joints traversal
  /// order.
  std::vector<size_t> jointIds;
  /// Measurement labels followed by the native regularization/collocation
  /// labels.
  std::vector<std::string> errorNames;
  /// Half-squared whitened errors per group, before and after outer iterations.
  std::vector<gtsam::Vector> errorHistory;
  /// Optimizer wall seconds at each errorHistory entry, including diagnostics.
  std::vector<double> wallSeconds;
  /// Native graph/variable assembly wall seconds, before feasibility
  /// restoration.
  double assemblySeconds = 0;
  /// Manifold identification and raw-zero feasibility restoration wall seconds.
  double initializationSeconds = 0;
  /// Coupled optimization wall seconds, including callbacks and error
  /// histories.
  double optimizationSeconds = 0;
  /// L2 norm of the raw initial equality residuals, before restoring
  /// feasibility.
  double initialEqualityL2 = 0;
  /// L2 norm of the final equality residuals in their original constraint
  /// units.
  double equalityL2 = 0;
  /// L2 norm of final positive inequality violations in original units.
  double inequalityL2 = 0;
  /// Half-squared whitened cost after feasibility restoration, before LM.
  double objectiveInitial = 0;
  /// Half-squared whitened cost of the final Values; excludes exact
  /// constraints.
  double objectiveFinal = 0;
  /// Number of completed outer LM iterations; reaching the cap is not
  /// convergence.
  size_t iterations = 0;
  /// Scalar ambient variable coordinates, including dependent link state.
  size_t variableDimension = 0;
  /// Free coordinates of constrained manifolds; excludes unconstrained
  /// landmarks.
  size_t reducedDimension = 0;
  /// Scalar equality residual rows before any rank reduction.
  size_t equalityDimension = 0;
  /// Scalar inequality residual rows, including all sole-wrench cone faces.
  size_t inequalityDimension = 0;
  /// Both final constraint violation norms are less than 1e-5.
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
 * Inputs remain unchanged. The returned result owns all numerical data and
 * remains valid after the inputs are destroyed. Costs use the per-interval
 * collocation scales in options; no automatic rescaling by time is applied.
 * Invalid dimensions, nonpositive/nonfinite scales, an absent gauge slice, or
 * unsupported topology throw std::invalid_argument. Missing input variables
 * and numerical projection/linear-solve failures propagate native exceptions.
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
