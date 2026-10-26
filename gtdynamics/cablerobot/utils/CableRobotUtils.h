/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file CableRobotUtils.h
 * @brief Cable robot kinodynamics problem from PR #363.
 * @author Gerry Chen
 * @author Yetong Zhang
 */
#pragma once

#include <gtdynamics/cablerobot/factors/WinchFactor.h>
#include <gtdynamics/cmopt/TangentSpaceBasis.h>
#include <gtdynamics/universal_robot/Link.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <array>
#include <map>
#include <memory>
#include <string>

namespace gtdynamics {

/** Parameters of the original zero-gravity cable robot experiment.
 * Mount points and initPose are explicit values, initialized from the default
 * dimensions. Update them explicitly when changing the geometry.
 */
struct CableRobotParams {
  double dt = 0.1;
  double midTorque = 0.6;
  double collocationSigma = 1e-3;
  double Q = 1.0, R = 0.1;
  uint8_t eeId = 1;
  LinkSharedPtr ee =
      std::make_shared<Link>(eeId, "ee", 1, gtsam::Matrix3::Identity(),
                             gtsam::Pose3(), gtsam::Pose3());
  double cdprWidth = 3, cdprHeight = 2;
  std::array<gtsam::Point3, 4> cdprFrameMountPoints = {
      gtsam::Point3{cdprWidth, 0, 0}, gtsam::Point3{cdprWidth, cdprHeight, 0},
      gtsam::Point3{0, cdprHeight, 0}, gtsam::Point3{0, 0, 0}};
  double cdprEeWidth = 0.1, cdprEeHeight = 0.1;
  std::array<gtsam::Point3, 4> cdprEeMountPoints = {
      gtsam::Point3{cdprEeWidth / 2, -cdprEeHeight / 2, 0},
      gtsam::Point3{cdprEeWidth / 2, cdprEeHeight / 2, 0},
      gtsam::Point3{-cdprEeWidth / 2, cdprEeHeight / 2, 0},
      gtsam::Point3{-cdprEeWidth / 2, -cdprEeHeight / 2, 0}};
  WinchParams winchParams{0.0127, 0, 0, 0};
  gtsam::Pose3 initPose{gtsam::Rot3(),
                        gtsam::Point3(cdprWidth / 2, cdprHeight / 2, 0)};
};

/** Full Pose3/Vector6 cable robot model with four winches.
 * Initial pose/twist, cable kinematics, wrench balance and winch dynamics are
 * equalities. Trapezoidal collocation remains a cost. No tension or torque
 * inequalities, gravity, or additional planar constraints are imposed.
 */
class GTSAM_EXPORT CableRobot {
 public:
  /** Construct the problem builder from physical and cost parameters. */
  explicit CableRobot(const CableRobotParams &params) : params_(params) {}

  /** Build initial-state and per-step physical equality residuals. */
  gtsam::NonlinearFactorGraph constraintFactors(int numSteps) const;
  /** Build collocation, waypoint and torque-prior costs, grouped by name. */
  std::map<std::string, gtsam::NonlinearFactorGraph> costFactors(
      int numSteps, const std::map<int, gtsam::Pose3> &waypoints) const;
  /** Reproduce the original, generally infeasible constant initial guess. */
  gtsam::Values initialValues(int numSteps) const;
  /** Select pose/twist after time zero and all tensions for the optional
   * basis-key experiment. The benchmark uses the default dense basis instead.
   */
  BasisKeyFunction getBasisKeyFunction() const;
  /** Export lengths, velocities, accelerations, torques and end-effector poses.
   * @throws std::runtime_error when the output file cannot be opened.
   */
  void exportTrajectory(const gtsam::Values &results, size_t numSteps,
                        std::string filePath = "traj.csv") const;

 protected:
  /** Build pose and zero-twist priors at time zero. */
  gtsam::NonlinearFactorGraph initialFactor(const gtsam::Pose3 &init) const;
  /** Build length, velocity and acceleration cable equalities at one step. */
  gtsam::NonlinearFactorGraph kinematicsFactor(int k) const;
  /** Build zero-gravity wrench balance and winch equalities at one step. */
  gtsam::NonlinearFactorGraph dynamicsFactor(int k) const;
  /** Build fixed-time trapezoidal pose and twist costs on one interval. */
  gtsam::NonlinearFactorGraph collocationFactor(int k) const;
  /** Build collocation costs on all numSteps - 1 intervals. */
  gtsam::NonlinearFactorGraph collocationFactors(int numSteps) const;
  /** Build pose priors weighted by Q at the supplied sample indices. */
  gtsam::NonlinearFactorGraph waypointFactors(
      const std::map<int, gtsam::Pose3> &waypoints) const;
  /** Build four torque priors per step, weighted by R around midTorque. */
  gtsam::NonlinearFactorGraph minTorqueFactors(int numSteps) const;

  CableRobotParams params_;
};

}  // namespace gtdynamics
