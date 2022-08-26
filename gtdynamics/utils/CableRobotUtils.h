/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CableRobotUtils.h
 * @brief Utilities for cable robot trajectory optimization experiments.
 * @author: Gerry Chen
 * @author: Yetong Zhang
 */

#pragma once

#include "gtdynamics/universal_robot/Link.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/expressions.h>

#include <gtdynamics/manifold/ConnectedComponent.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtdynamics/optimizer/MutableLMOptimizer.h>
#include <gtdynamics/optimizer/PenaltyMethodOptimizer.h>
#include <gtdynamics/manifold/ManifoldOptimizer.h>
#include <gtdynamics/utils/DynamicsSymbol.h>
#include <gtdynamics/cablerobot/factors/WinchFactor.h>

namespace gtsam {

// TODO(gerry): rename these formerly const variables to not be const.
struct CableRobotParams {
  // Misc
  double dt = 1e-1;
  double midTorque = 0.6;
  double collocation_sigma = 1e-3;
  double Q = 1, R = 1e-1;

  // Link
  uint8_t eeId = 1;
  gtdynamics::LinkSharedPtr ee = make_shared<gtdynamics::Link>(
      eeId, "ee", 1, Matrix3::Identity(), Pose3(), Pose3());

  // Geometric parameters
  double cdprWidth = 3, cdprHeight = 2;
  std::array<Point3, 4> cdprFrameMountPoints = {
      Point3{cdprWidth, 0, 0},
      Point3{cdprWidth, cdprHeight, 0},
      Point3{0, cdprHeight, 0},
      Point3{0, 0, 0},
  };
  double cdprEeWidth = 0.1, cdprEeHeight = 0.1;
  std::array<Point3, 4> cdprEeMountPoints = {
      Point3{cdprEeWidth / 2, -cdprEeHeight / 2, 0},
      Point3{cdprEeWidth / 2, cdprEeHeight / 2, 0},
      Point3{-cdprEeWidth / 2, cdprEeHeight / 2, 0},
      Point3{-cdprEeWidth / 2, -cdprEeHeight / 2, 0},
  };
  gtdynamics::WinchParams winch_params{0.0127, 0, 0, 0};

  // Initialization
  Pose3 init_pose{Rot3(), Point3(cdprWidth / 2, cdprHeight / 2, 0)};
};

class CableRobot {
 public:
  CableRobot(const CableRobotParams &params) : params_(params) {}

 protected:
  NonlinearFactorGraph initialFactor(const Pose3 &init) const;
  NonlinearFactorGraph kinematicsFactor(int k) const;
  NonlinearFactorGraph dynamicsFactor(int k) const;
  NonlinearFactorGraph collocationFactor(int k) const;
  NonlinearFactorGraph collocationFactors(int num_steps) const;
  NonlinearFactorGraph waypointFactors(
      const std::map<int, Pose3> &waypoints) const;
  NonlinearFactorGraph minTorqueFactors(int k) const;

 public:
  NonlinearFactorGraph constraintFactors(int num_steps) const;
  std::map<std::string, NonlinearFactorGraph> costFactors(
      int num_steps, const std::map<int, Pose3> &waypoints) const;
  Values initialValues(int num_steps) const;

  /// Return function that select basis keys for constraint manifolds.
  BasisKeyFunc getBasisKeyFunc() const;

  /// Save trajectory to file.
  void exportTrajectory(const Values &results, const size_t num_steps,
                        std::string file_path = "traj.csv") const;

 protected:
  CableRobotParams params_;
};

}  // namespace gtsam
