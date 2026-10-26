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
static DynamicsSymbol tensionKey(int j, int k = 0) {
  return DynamicsSymbol::JointSymbol("t", j, k);
}

NonlinearFactorGraph CableRobot::initialFactor(const Pose3 &init) const {
  auto graph = NonlinearFactorGraph();
  graph.emplace_shared<PriorFactor<Pose3>>(PoseKey(params_.eeId, 0), init,
                                           noiseModel::Unit::Create(6));
  graph.emplace_shared<PriorFactor<Vector6>>(
      TwistKey(params_.eeId, 0), Vector6::Zero(), noiseModel::Unit::Create(6));
  return graph;
}
NonlinearFactorGraph CableRobot::kinematicsFactor(int k) const {
  auto graph = NonlinearFactorGraph();
  for (int ji = 0; ji < 4; ++ji) {
    graph.emplace_shared<CableLengthFactor>(JointAngleKey(ji, k),      //
                                            PoseKey(params_.eeId, k),  //
                                            noiseModel::Unit::Create(1),
                                            params_.cdprFrameMountPoints[ji],
                                            params_.cdprEeMountPoints[ji]);
    graph.emplace_shared<CableVelocityFactor>(JointVelKey(ji, k),         //
                                              PoseKey(params_.eeId, k),   //
                                              TwistKey(params_.eeId, k),  //
                                              noiseModel::Unit::Create(1),
                                              params_.cdprFrameMountPoints[ji],
                                              params_.cdprEeMountPoints[ji]);
    graph.emplace_shared<CableAccelerationFactor>(
        JointAccelKey(ji, k),            //
        PoseKey(params_.eeId, k),        //
        TwistKey(params_.eeId, k),       //
        TwistAccelKey(params_.eeId, k),  //
        noiseModel::Unit::Create(1), params_.cdprFrameMountPoints[ji],
        params_.cdprEeMountPoints[ji]);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::dynamicsFactor(int k) const {
  auto graph = NonlinearFactorGraph();
  graph.push_back(WrenchFactor(
      noiseModel::Unit::Create(6), params_.ee,
      {WrenchKey(params_.eeId, 0, k), WrenchKey(params_.eeId, 1, k),
       WrenchKey(params_.eeId, 2, k), WrenchKey(params_.eeId, 3, k)},
      k, Vector3::Zero()));
  for (int ji = 0; ji < 4; ++ji) {
    graph.emplace_shared<CableTensionFactor>(
        tensionKey(ji, k),         //
        PoseKey(params_.eeId, k),  //
        WrenchKey(params_.eeId, ji, k), noiseModel::Unit::Create(6),
        params_.cdprFrameMountPoints[ji], params_.cdprEeMountPoints[ji]);
    graph.emplace_shared<WinchFactor>(
        TorqueKey(ji, k),  //
        tensionKey(ji, k), JointVelKey(ji, k), JointAccelKey(ji, k),
        noiseModel::Unit::Create(1), params_.winchParams);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::collocationFactor(int k) const {
  auto graph = NonlinearFactorGraph();
  graph.emplace_shared<FixTimeTrapezoidalPoseCollocationFactor>(
      PoseKey(params_.eeId, k), PoseKey(params_.eeId, k + 1),
      TwistKey(params_.eeId, k), TwistKey(params_.eeId, k + 1), params_.dt,
      noiseModel::Isotropic::Sigma(6, params_.collocationSigma));
  graph.emplace_shared<FixTimeTrapezoidalTwistCollocationFactor>(
      TwistKey(params_.eeId, k), TwistKey(params_.eeId, k + 1),
      TwistAccelKey(params_.eeId, k), TwistAccelKey(params_.eeId, k + 1),
      params_.dt, noiseModel::Isotropic::Sigma(6, params_.collocationSigma));
  return graph;
}

NonlinearFactorGraph CableRobot::collocationFactors(int numSteps) const {
  auto graph = NonlinearFactorGraph();
  for (int k = 0; k < numSteps - 1; ++k) {
    graph.push_back(collocationFactor(k));
  }
  return graph;
}
NonlinearFactorGraph CableRobot::waypointFactors(
    const std::map<int, Pose3> &waypoints) const {
  auto graph = NonlinearFactorGraph();
  auto waypointModel =
      noiseModel::Isotropic::Sigma(6, std::sqrt(1 / params_.Q));
  for (const auto &[k, v] : waypoints) {
    graph.emplace_shared<PriorFactor<Pose3>>(PoseKey(params_.eeId, k), v,
                                             waypointModel);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::minTorqueFactors(int numSteps) const {
  auto graph = NonlinearFactorGraph();
  auto torqueModel = noiseModel::Isotropic::Sigma(1, std::sqrt(1 / params_.R));
  for (int k = 0; k < numSteps; ++k) {
    for (int ji = 0; ji < 4; ++ji) {
      graph.emplace_shared<PriorFactor<double>>(TorqueKey(ji, k),
                                                params_.midTorque, torqueModel);
    }
  }
  return graph;
}

/******************************************************************************/
NonlinearFactorGraph CableRobot::constraintFactors(int numSteps) const {
  auto graph = NonlinearFactorGraph();
  graph.push_back(initialFactor(params_.initPose));
  for (int k = 0; k < numSteps; ++k) {
    graph.push_back(kinematicsFactor(k));
    graph.push_back(dynamicsFactor(k));
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
  for (int k = 0; k < numSteps; ++k) {
    for (int ji = 0; ji < 4; ++ji) {
      InsertJointAngle(&values, ji, k, 1.66);
      InsertJointVel(&values, ji, k, 0.0);
      InsertJointAccel(&values, ji, k, 0.0);
      InsertWrench(&values, params_.eeId, ji, k, Vector6::Zero());
      InsertTorque(&values, ji, k, 0.6);
      values.insert(tensionKey(ji, k), 47.);
    }
    InsertPose(&values, params_.eeId, k, params_.initPose);
    InsertTwist(&values, params_.eeId, k, Vector6::Zero());
    InsertTwistAccel(&values, params_.eeId, k, Vector6::Zero());
  }
  return values;
}

BasisKeyFunction CableRobot::getBasisKeyFunction() const {
  return [](const KeyVector &keys) -> KeyVector {
    KeyVector basisKeys;
    for (const Key &key : keys) {
      auto symb = gtdynamics::DynamicsSymbol(key);
      if (symb.label() == "p" && symb.time() > 0) {
        basisKeys.push_back(key);
      } else if (symb.label() == "V" && symb.time() > 0) {
        basisKeys.push_back(key);
      } else if (symb.label() == "t") {
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
    for (int ji = 0; ji < 4; ++ji) trajectoryFile << prefix << ji << ",";
  }
  trajectoryFile << "base_x,base_y,base_z,base_qx,base_qy,base_qz,base_qw\n";
  for (size_t t = 0; t < numSteps; t++) {
    for (int ji = 0; ji < 4; ++ji)
      trajectoryFile << JointAngle(results, ji, t) << ",";
    for (int ji = 0; ji < 4; ++ji)
      trajectoryFile << JointVel(results, ji, t) << ",";
    for (int ji = 0; ji < 4; ++ji)
      trajectoryFile << JointAccel(results, ji, t) << ",";
    for (int ji = 0; ji < 4; ++ji)
      trajectoryFile << Torque(results, ji, t) << ",";
    const Pose3 pose = Pose(results, params_.eeId, t);
    const auto rotation = pose.rotation().toQuaternion();
    trajectoryFile << pose.x() << "," << pose.y() << "," << pose.z() << ","
                   << rotation.x() << "," << rotation.y() << "," << rotation.z()
                   << "," << rotation.w() << "\n";
  }
  trajectoryFile.close();
}

}  // namespace gtdynamics
