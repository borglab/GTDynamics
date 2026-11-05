/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file ZeroInitializer.h
 * @brief Immutable zero-value dynamics initialization.
 * @author Alejandro Escontrela, Yetong Zhang, GTDynamics contributors
 */
#pragma once
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtsam/nonlinear/Values.h>
namespace gtdynamics {
class ZeroSliceInitializer {
 public:
  /// Copy the Gaussian perturbation scale; zero gives a deterministic seed.
  explicit ZeroSliceInitializer(const Robot& robot, double gaussianNoise = 0.0);

  /// Destroy a polymorphic slice strategy.
  virtual ~ZeroSliceInitializer() = default;
  /**
   * @fn Return zero values for all variables for initial value of optimization.
   *
   * @param[in] t              Timestep to return zero initial values for.
   * @param[in] contactPoints Contact points for timestep t.
   */
  virtual gtsam::Values run(
      const int t, const std::optional<PointOnLinks>& contactPoints = {}) const;

 protected:
  const Robot robot_;
  const double gaussianNoise_;
};
class MultiPhaseZeroInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit MultiPhaseZeroInitializer(const Robot& robot,
                                     const double gaussianNoise = 1e-8);

  /**
   * @fn Initialize solution for multi-phase trajectory to nominal pose.
   *
   * @param[in] phaseSteps           Number of steps for each phase.
   * @param[in] transitionValues Initial values for transition graphs.
   * @param[in] dt                  Initial phase duration,
   * @param[in] phaseContactPoints  Contact points at each phase.
   * A supplied sliceInitializer controls its own perturbation scale; otherwise
   * the scale copied by this trajectory initializer is used.
   */
  gtsam::Values run(
      const std::vector<int>& phaseSteps,
      std::vector<gtsam::Values> transitionValues, double dt = 1. / 240,
      const std::optional<std::vector<PointOnLinks>>& phaseContactPoints = {},
      const ZeroSliceInitializer* sliceInitializer = nullptr) const;

 private:
  const Robot robot_;
  const double gaussianNoise_;
};
class ZeroTrajectoryInitializer {
 public:
  /** Bind the robot and copy tuning parameters. The model shares fixed
   * Link/Joint topology, which must remain unchanged. Trajectory data is
   * supplied separately to run(). */
  explicit ZeroTrajectoryInitializer(const Robot& robot,
                                     double gaussianNoise = 0.0);

  /**
   * @fn Return zero values of the trajectory for initial value of optimization.
   *
   * @param[in] numSteps      Total number of time steps.
   * @param[in] numPhases     Number of phases, -1 if not using.
   * @param[in] contactPoints Contact points along the trajectory.
   * @return Initial solution stored in a gtsam::Values object.
   */
  gtsam::Values run(
      const int numSteps, const int numPhases = -1,
      const std::optional<PointOnLinks>& contactPoints = {},
      const ZeroSliceInitializer* sliceInitializer = nullptr) const;

 private:
  const Robot robot_;
  const double gaussianNoise_;
};
}  // namespace gtdynamics
