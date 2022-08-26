/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.h
 * @brief Utilities for quadruped trajectory optimization experiments.
 * @author: Yetong Zhang
 */

#pragma once

#include "QuadrupedUtils.h"

#include <gtdynamics/cablerobot/factors/WinchFactor.h>

namespace gtsam {

// TODO(gerry): rename these formerly const variables to not be const.
struct CableRobotParams {
  // Misc
  double dt = 1e-2;
  double midTorque = 0.6;

  // Link
  uint8_t eeId = 1;
  gtdynamics::LinkSharedPtr ee = make_shared<gtdynamics::Link>(
      eeId, "ee", 1, Matrix3::Identity(), Pose3(), Pose3());

  // Geometric parameters
  double cdprWidth = 3, cdprHeight = 2;
  std::array<Point3, 4> cdprFrameMountPoints = {
      Point3{cdprWidth, 0, 0},  //
      Point3{cdprWidth, cdprHeight, 0}, Point3{0, cdprHeight, 0},
      Point3{0, 0, 0}};
  double cdprEeWidth = 0.1, cdprEeHeight = 0.1;
  std::array<Point3, 4> cdprEeMountPoints = {
      Point3{cdprEeWidth, 0, 0}, Point3{cdprEeWidth, cdprEeHeight, 0},
      Point3{0, cdprEeHeight, 0}, Point3{0, 0, 0}};
  gtdynamics::WinchParams winch_params{0.0127, 1e-7, 1e-3, 1e-3};

  // Initialization
  Pose3 init_pose{Rot3(), Point3(cdprEeWidth / 2, cdprHeight / 2, 0)};
};

class CableRobot {
 public:
  CableRobot(const CableRobotParams &params) : params_(params) {}

 protected:
  NonlinearFactorGraph initialFactor(const Pose3 &init) const;
  NonlinearFactorGraph kinematicsFactor(int k) const;
  NonlinearFactorGraph dynamicsFactor(int k) const;
  NonlinearFactorGraph collocationFactor(int k) const;

 public:
  NonlinearFactorGraph constraintFactors(int num_steps) const;
  NonlinearFactorGraph costFactors(int num_steps,
                                   const std::map<int, Pose3> &waypoints,
                                   double Q, double R) const;
  Values initialValues(int num_steps) const;

 protected:
  CableRobotParams params_;

  // /// Print joint angles.
  // void printJointAngles(const Values &values, int t = 0) const;

  // /// Return optimizer setting.
  // const gtdynamics::OptimizerSetting &opt() const {
  //   return graph_builder.opt();
  // }

  // /// Return function that select basis keys for constraint manifolds.
  // BasisKeyFunc getBasisKeyFunc() const {
  //   if (express_redundancy) {
  //     return &FindBasisKeysReduancy;
  //   } else {
  //     return &FindBasisKeys4C;
  //   }
  // }

  // /// Save trajectory to file.
  // void exportTrajectory(const Values &results, const size_t num_steps,
  //                       std::string file_path = "traj.csv");
};

}  // namespace gtsam
