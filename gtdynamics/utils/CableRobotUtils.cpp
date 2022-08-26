/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QuadrupedUtils.cpp
 * @brief Quadruped utilities implementations.
 * @author: Yetong Zhang
 */

#include "CableRobotUtils.h"

#include <gtdynamics/cablerobot/factors/CableLengthFactor.h>
#include <gtdynamics/cablerobot/factors/CableVelocityFactor.h>
#include <gtdynamics/cablerobot/factors/CableAccelerationFactor.h>
#include <gtdynamics/cablerobot/factors/CableTensionFactor.h>
#include <gtdynamics/cablerobot/factors/WinchFactor.h>
#include <gtdynamics/factors/WrenchFactor.h>
#include <gtdynamics/factors/CollocationFactors.h>

using namespace gtdynamics;

/// Shorthand for t_j_k, for j-th tension at time step k.
inline DynamicsSymbol TensionKey(int j, int k = 0) {
  return DynamicsSymbol::JointSymbol("t", j, k);
}

namespace gtsam {

NonlinearFactorGraph CableRobot::initialFactor(const Pose3& init) const {
  auto graph = NonlinearFactorGraph();
  graph.emplace_shared<PriorFactor<Pose3>>(params_.eeId, init,
                                           noiseModel::Unit::Create(6));
  graph.emplace_shared<PriorFactor<Vector6>>(params_.eeId, Vector6::Zero(),
                                             noiseModel::Unit::Create(6));
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
      k, Vector3(0, -9.81, 0)));
  for (int ji = 0; ji < 4; ++ji) {
    graph.emplace_shared<CableTensionFactor>(
        TensionKey(ji, k),         //
        PoseKey(params_.eeId, k),  //
        WrenchKey(params_.eeId, ji, k), noiseModel::Unit::Create(1),
        params_.cdprFrameMountPoints[ji], params_.cdprEeMountPoints[ji]);
    graph.emplace_shared<WinchFactor>(
        TorqueKey(ji, k),  //
        TensionKey(ji, k), JointVelKey(ji, k), JointAccelKey(ji, k),
        noiseModel::Unit::Create(1), params_.winch_params);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::collocationFactor(int k) const {
  auto graph = NonlinearFactorGraph();
  graph.emplace_shared<FixTimeTrapezoidalPoseCollocationFactor>(
      PoseKey(params_.eeId, k), PoseKey(params_.eeId, k + 1),
      TwistKey(params_.eeId, k), TwistKey(params_.eeId, k + 1), params_.dt,
      noiseModel::Unit::Create(6));
  graph.emplace_shared<FixTimeTrapezoidalTwistCollocationFactor>(
      TwistKey(params_.eeId, k), TwistKey(params_.eeId, k + 1),
      TwistAccelKey(params_.eeId, k), TwistAccelKey(params_.eeId, k + 1),
      params_.dt, noiseModel::Unit::Create(6));
  return graph;
}

/******************************************************************************/
NonlinearFactorGraph CableRobot::constraintFactors(int num_steps) const {
  auto graph = NonlinearFactorGraph();
  graph.push_back(initialFactor(params_.init_pose));
  for (int k = 0; k < num_steps; ++k) {
    graph.push_back(kinematicsFactor(k));
    graph.push_back(dynamicsFactor(k));
    if (k < num_steps - 1) graph.push_back(collocationFactor(k));
  }
  return graph;
}
NonlinearFactorGraph CableRobot::costFactors(
    int num_steps, const std::map<int, Pose3>& waypoints, double Q,
    double R) const {
  auto graph = NonlinearFactorGraph();
  noiseModel::Base::shared_ptr Q_model = noiseModel::Isotropic::Sigma(6, Q);
  noiseModel::Base::shared_ptr R_model = noiseModel::Isotropic::Sigma(4, R);
  for (const auto& [k, v] : waypoints)
    graph.emplace_shared<PriorFactor<Pose3>>(params_.eeId, v, Q_model);
  for (int k = 0; k < num_steps; ++k) {
    for (int ji = 0; ji < 4; ++ji) {
      graph.emplace_shared<PriorFactor<double>>(
          JointAngleKey(ji, k), params_.midTorque, noiseModel::Unit::Create(1));
    }
  }
  return graph;
}
Values CableRobot::initialValues(int num_steps) const {
  Values values;
  for (int k = 0; k < num_steps; ++k) {
    for (int ji = 0; ji < 4; ++ji) {
      InsertJointAngle(&values, ji, k, 1.5);
      InsertJointVel(&values, ji, k, 0.0);
      InsertJointAccel(&values, ji, k, 0.0);
      InsertWrench(&values, params_.eeId, ji, k, Vector6::Zero());
      InsertTorque(&values, ji, k, 0.0);
      values.insert(TensionKey(ji, k), 0.0);
    }
    InsertPose(&values, params_.eeId, k, params_.init_pose);
    InsertTwist(&values, params_.eeId, k, Vector6::Zero());
    InsertTwistAccel(&values, params_.eeId, k, Vector6::Zero());
  }
  return values;
}

}  // namespace gtsam
