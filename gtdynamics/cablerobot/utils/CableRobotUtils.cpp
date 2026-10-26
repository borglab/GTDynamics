/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  CableRobotUtils.cpp
 * @brief Utilities for cable robot trajectory optimization experiments.
 * @author: Gerry Chen
 * @author: Yetong Zhang
 */

#include "CableRobotUtils.h"

#include <gtdynamics/cablerobot/factors/CableAccelerationFactor.h>
#include <gtdynamics/cablerobot/factors/CableLengthFactor.h>
#include <gtdynamics/cablerobot/factors/CableTensionFactor.h>
#include <gtdynamics/cablerobot/factors/CableVelocityFactor.h>
#include <gtdynamics/cablerobot/factors/WinchFactor.h>
#include <gtdynamics/dynamics/WrenchFactor.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <fstream>
#include <iomanip>
#include <stdexcept>

using namespace gtsam;

namespace gtdynamics {

/// Shorthand for t_j_k, for j-th tension at time step k.
static DynamicsSymbol tensionKey(int cableIndex, int stepIndex = 0) {
  return DynamicsSymbol::JointSymbol("t", cableIndex, stepIndex);
}

NonlinearFactorGraph CableRobot::initialFactor(const Pose3 &initialPose) const {
  auto graph = NonlinearFactorGraph();
  graph.emplace_shared<PriorFactor<Pose3>>(PoseKey(params_.endEffectorId, 0),
                                           initialPose,
                                           noiseModel::Unit::Create(6));
  graph.emplace_shared<PriorFactor<Vector6>>(TwistKey(params_.endEffectorId, 0),
                                             Vector6::Zero(),
                                             noiseModel::Unit::Create(6));
  return graph;
}
NonlinearFactorGraph CableRobot::kinematicsFactor(int stepIndex) const {
  auto graph = NonlinearFactorGraph();
  for (int cableIndex = 0; cableIndex < 4; ++cableIndex) {
    graph.emplace_shared<CableLengthFactor>(
        JointAngleKey(cableIndex, stepIndex),       //
        PoseKey(params_.endEffectorId, stepIndex),  //
        noiseModel::Unit::Create(1), params_.frameMountPoints[cableIndex],
        params_.endEffectorMountPoints[cableIndex]);
    graph.emplace_shared<CableVelocityFactor>(
        JointVelKey(cableIndex, stepIndex),          //
        PoseKey(params_.endEffectorId, stepIndex),   //
        TwistKey(params_.endEffectorId, stepIndex),  //
        noiseModel::Unit::Create(1), params_.frameMountPoints[cableIndex],
        params_.endEffectorMountPoints[cableIndex]);
    graph.emplace_shared<CableAccelerationFactor>(
        JointAccelKey(cableIndex, stepIndex),             //
        PoseKey(params_.endEffectorId, stepIndex),        //
        TwistKey(params_.endEffectorId, stepIndex),       //
        TwistAccelKey(params_.endEffectorId, stepIndex),  //
        noiseModel::Unit::Create(1), params_.frameMountPoints[cableIndex],
        params_.endEffectorMountPoints[cableIndex]);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::dynamicsFactor(int stepIndex) const {
  auto graph = NonlinearFactorGraph();
  graph.push_back(WrenchFactor(noiseModel::Unit::Create(6), params_.endEffector,
                               {WrenchKey(params_.endEffectorId, 0, stepIndex),
                                WrenchKey(params_.endEffectorId, 1, stepIndex),
                                WrenchKey(params_.endEffectorId, 2, stepIndex),
                                WrenchKey(params_.endEffectorId, 3, stepIndex)},
                               stepIndex, Vector3::Zero()));
  for (int cableIndex = 0; cableIndex < 4; ++cableIndex) {
    graph.emplace_shared<CableTensionFactor>(
        tensionKey(cableIndex, stepIndex),          //
        PoseKey(params_.endEffectorId, stepIndex),  //
        WrenchKey(params_.endEffectorId, cableIndex, stepIndex),
        noiseModel::Unit::Create(6), params_.frameMountPoints[cableIndex],
        params_.endEffectorMountPoints[cableIndex]);
    graph.emplace_shared<WinchFactor>(
        TorqueKey(cableIndex, stepIndex),  //
        tensionKey(cableIndex, stepIndex), JointVelKey(cableIndex, stepIndex),
        JointAccelKey(cableIndex, stepIndex), noiseModel::Unit::Create(1),
        params_.winchParams);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::collocationFactor(int stepIndex) const {
  auto graph = NonlinearFactorGraph();
  graph.emplace_shared<FixTimeTrapezoidalPoseCollocationFactor>(
      PoseKey(params_.endEffectorId, stepIndex),
      PoseKey(params_.endEffectorId, stepIndex + 1),
      TwistKey(params_.endEffectorId, stepIndex),
      TwistKey(params_.endEffectorId, stepIndex + 1), params_.timeStep,
      noiseModel::Isotropic::Sigma(6, params_.collocationSigma));
  graph.emplace_shared<FixTimeTrapezoidalTwistCollocationFactor>(
      TwistKey(params_.endEffectorId, stepIndex),
      TwistKey(params_.endEffectorId, stepIndex + 1),
      TwistAccelKey(params_.endEffectorId, stepIndex),
      TwistAccelKey(params_.endEffectorId, stepIndex + 1), params_.timeStep,
      noiseModel::Isotropic::Sigma(6, params_.collocationSigma));
  return graph;
}

NonlinearFactorGraph CableRobot::collocationFactors(int numSteps) const {
  auto graph = NonlinearFactorGraph();
  for (int stepIndex = 0; stepIndex < numSteps - 1; ++stepIndex) {
    graph.push_back(collocationFactor(stepIndex));
  }
  return graph;
}
NonlinearFactorGraph CableRobot::waypointFactors(
    const std::map<int, Pose3> &waypoints) const {
  auto graph = NonlinearFactorGraph();
  auto waypointModel =
      noiseModel::Isotropic::Sigma(6, std::sqrt(1 / params_.waypointWeight));
  for (const auto &[stepIndex, waypointPose] : waypoints) {
    graph.emplace_shared<PriorFactor<Pose3>>(
        PoseKey(params_.endEffectorId, stepIndex), waypointPose, waypointModel);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::minTorqueFactors(int numSteps) const {
  auto graph = NonlinearFactorGraph();
  auto torqueModel =
      noiseModel::Isotropic::Sigma(1, std::sqrt(1 / params_.torqueWeight));
  for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex) {
      graph.emplace_shared<PriorFactor<double>>(
          TorqueKey(cableIndex, stepIndex), params_.torquePrior, torqueModel);
    }
  }
  return graph;
}

/******************************************************************************/
NonlinearFactorGraph CableRobot::constraintFactors(int numSteps) const {
  auto graph = NonlinearFactorGraph();
  graph.push_back(initialFactor(params_.initialPose));
  for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
    graph.push_back(kinematicsFactor(stepIndex));
    graph.push_back(dynamicsFactor(stepIndex));
  }
  return graph;
}
std::map<std::string, NonlinearFactorGraph> CableRobot::costFactors(
    int numSteps, const std::map<int, Pose3> &waypoints) const {
  return {{"Collocation", collocationFactors(numSteps)},
          {"waypoint", waypointFactors(waypoints)},
          {"minTorque", minTorqueFactors(numSteps)}};
}
Values CableRobot::initialValues(int numSteps) const {
  Values values;
  for (int stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex) {
      InsertJointAngle(&values, cableIndex, stepIndex, 1.66);
      InsertJointVel(&values, cableIndex, stepIndex, 0.0);
      InsertJointAccel(&values, cableIndex, stepIndex, 0.0);
      InsertWrench(&values, params_.endEffectorId, cableIndex, stepIndex,
                   Vector6::Zero());
      InsertTorque(&values, cableIndex, stepIndex, 0.6);
      values.insert(tensionKey(cableIndex, stepIndex), 47.);
    }
    InsertPose(&values, params_.endEffectorId, stepIndex, params_.initialPose);
    InsertTwist(&values, params_.endEffectorId, stepIndex, Vector6::Zero());
    InsertTwistAccel(&values, params_.endEffectorId, stepIndex,
                     Vector6::Zero());
  }
  return values;
}

BasisKeyFunction CableRobot::getBasisKeyFunction() const {
  return [](const KeyVector &keys) -> KeyVector {
    KeyVector basisKeys;
    for (const Key &key : keys) {
      auto symbol = gtdynamics::DynamicsSymbol(key);
      if (symbol.label() == "p" && symbol.time() > 0) {
        basisKeys.push_back(key);
      } else if (symbol.label() == "V" && symbol.time() > 0) {
        basisKeys.push_back(key);
      } else if (symbol.label() == "t") {
        // tension, instead of A, because robot is constrained to the plane
        basisKeys.push_back(key);
      }
    }
    return basisKeys;
  };
}

// mostly copy-pasted from QuadrupedUtils.cpp
void CableRobot::exportTrajectory(const Values &results, const size_t numSteps,
                                  std::string filePath) const {
  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::ofstream trajectoryFile(filePath);
  if (!trajectoryFile)
    throw std::runtime_error("Cannot open trajectory: " + filePath);
  trajectoryFile << std::setprecision(17);
  for (const auto *prefix : {"l", "ldot", "lddot", "tau"}) {
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex)
      trajectoryFile << prefix << cableIndex << ",";
  }
  trajectoryFile << "base_x,base_y,base_z,base_qx,base_qy,base_qz,base_qw\n";
  for (size_t stepIndex = 0; stepIndex < numSteps; stepIndex++) {
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex)
      trajectoryFile << JointAngle(results, cableIndex, stepIndex) << ",";
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex)
      trajectoryFile << JointVel(results, cableIndex, stepIndex) << ",";
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex)
      trajectoryFile << JointAccel(results, cableIndex, stepIndex) << ",";
    for (int cableIndex = 0; cableIndex < 4; ++cableIndex)
      trajectoryFile << Torque(results, cableIndex, stepIndex) << ",";
    const Pose3 pose = Pose(results, params_.endEffectorId, stepIndex);
    const auto rotation = pose.rotation().toQuaternion();
    trajectoryFile << pose.x() << "," << pose.y() << "," << pose.z() << ","
                   << rotation.x() << "," << rotation.y() << "," << rotation.z()
                   << "," << rotation.w() << "\n";
  }
  trajectoryFile.close();
}

}  // namespace gtdynamics
