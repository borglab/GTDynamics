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

#include <boost/algorithm/string/join.hpp>

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
      k, Vector3(0, -9.81 * 0, 0)));
  for (int ji = 0; ji < 4; ++ji) {
    graph.emplace_shared<CableTensionFactor>(
        TensionKey(ji, k),         //
        PoseKey(params_.eeId, k),  //
        WrenchKey(params_.eeId, ji, k), noiseModel::Unit::Create(6),
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
      noiseModel::Isotropic::Sigma(6, params_.collocation_sigma));
  graph.emplace_shared<FixTimeTrapezoidalTwistCollocationFactor>(
      TwistKey(params_.eeId, k), TwistKey(params_.eeId, k + 1),
      TwistAccelKey(params_.eeId, k), TwistAccelKey(params_.eeId, k + 1),
      params_.dt, noiseModel::Isotropic::Sigma(6, params_.collocation_sigma));
  return graph;
}

NonlinearFactorGraph CableRobot::collocationFactors(int num_steps) const {
  auto graph = NonlinearFactorGraph();
  for (int k = 0; k < num_steps - 1; ++k) {
    graph.push_back(collocationFactor(k));
  }
  return graph;
}
NonlinearFactorGraph CableRobot::waypointFactors(
    const std::map<int, Pose3> &waypoints) const {
  auto graph = NonlinearFactorGraph();
  auto Q_model = noiseModel::Isotropic::Sigma(6, std::sqrt(1 / params_.Q));
  for (const auto &[k, v] : waypoints) {
    graph.emplace_shared<PriorFactor<Pose3>>(PoseKey(params_.eeId, k), v,
                                             Q_model);
  }
  return graph;
}
NonlinearFactorGraph CableRobot::minTorqueFactors(int num_steps) const {
  auto graph = NonlinearFactorGraph();
  auto R_model = noiseModel::Isotropic::Sigma(1, std::sqrt(1 / params_.R));
  for (int k = 0; k < num_steps; ++k) {
    for (int ji = 0; ji < 4; ++ji) {
      graph.emplace_shared<PriorFactor<double>>(TorqueKey(ji, k),
                                                params_.midTorque, R_model);
    }
  }
  return graph;
}

/******************************************************************************/
NonlinearFactorGraph CableRobot::constraintFactors(int num_steps) const {
  auto graph = NonlinearFactorGraph();
  graph.push_back(initialFactor(params_.init_pose));
  for (int k = 0; k < num_steps; ++k) {
    graph.push_back(kinematicsFactor(k));
    graph.push_back(dynamicsFactor(k));
    // if (k < num_steps - 1) graph.push_back(collocationFactor(k));
  }
  return graph;
}
std::map<std::string, NonlinearFactorGraph> CableRobot::costFactors(
    int num_steps, const std::map<int, Pose3> &waypoints) const {
  return {{"Collocation", collocationFactors(num_steps)},
          {"waypoint", waypointFactors(waypoints)},
          {"minTorque", minTorqueFactors(num_steps)}};
}
Values CableRobot::initialValues(int num_steps) const {
  Values values;
  for (int k = 0; k < num_steps; ++k) {
    for (int ji = 0; ji < 4; ++ji) {
      InsertJointAngle(&values, ji, k, 1.66);
      InsertJointVel(&values, ji, k, 0.0);
      InsertJointAccel(&values, ji, k, 0.0);
      InsertWrench(&values, params_.eeId, ji, k, Vector6::Zero());
      InsertTorque(&values, ji, k, 0.6);
      values.insert(TensionKey(ji, k), 47.);
    }
    InsertPose(&values, params_.eeId, k, params_.init_pose);
    InsertTwist(&values, params_.eeId, k, Vector6::Zero());
    InsertTwistAccel(&values, params_.eeId, k, Vector6::Zero());
  }
  return values;
}

BasisKeyFunc CableRobot::getBasisKeyFunc() const {
  return [](const ConnectedComponent::shared_ptr &cc) -> KeyVector {
    KeyVector basis_keys;
    for (const Key &key : cc->keys_) {
      auto symb = gtdynamics::DynamicsSymbol(key);
      symb.print();
      if (symb.label() == "p" && symb.time() > 0) {
        basis_keys.push_back(key);
      } else if (symb.label() == "V" && symb.time() > 0) {
        basis_keys.push_back(key);
      } else if (symb.label() == "t") {
        // tension, instead of A, because robot is constrained to the plane
        basis_keys.push_back(key);
      }
    }
    std::cout << "basis keys:" << std::endl;
    for (auto key : basis_keys) gtdynamics::DynamicsSymbol(key).print();
    std::cout << std::endl;
    return basis_keys;
  };
}

// mostly copy-pasted from QuadrupedUtils.cpp
void CableRobot::exportTrajectory(const Values &results, const size_t num_steps,
                                  std::string file_path) const {
  // Log the joint angles, velocities, accels, torques, and current goal pose.
  std::vector<std::string> jnames = {"0", "1", "2", "3"};
  std::string jnames_str = boost::algorithm::join(jnames, ",");
  std::ofstream traj_file;
  traj_file.open(file_path);
  // angles, vels, accels, torques.
  traj_file << "l" << jnames_str << ","
            << "ldot" << jnames_str << ","
            << "lddot" << jnames_str << ","
            << "tau" << jnames_str  //
            << ",base_x"
            << ",base_y"
            << ",base_z"
            << ",base_qx"
            << ",base_qy"
            << ",base_qz"
            << ",base_qw"
            << "\n";
  for (int t = 0; t < num_steps; t++) {
    std::vector<std::string> vals;
    for (int ji = 0; ji < 4; ++ji)
      vals.push_back(std::to_string(JointAngle(results, ji, t)));
    for (int ji = 0; ji < 4; ++ji)
      vals.push_back(std::to_string(JointVel(results, ji, t)));
    for (int ji = 0; ji < 4; ++ji)
      vals.push_back(std::to_string(JointAccel(results, ji, t)));
    for (int ji = 0; ji < 4; ++ji)
      vals.push_back(std::to_string(Torque(results, ji, t)));

    Pose3 bp = Pose(results, params_.eeId, t);
    vals.push_back(std::to_string(bp.x()));
    vals.push_back(std::to_string(bp.y()));
    vals.push_back(std::to_string(bp.z()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().x()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().y()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().z()));
    vals.push_back(std::to_string(bp.rotation().toQuaternion().w()));

    std::string vals_str = boost::algorithm::join(vals, ",");
    traj_file << vals_str << "\n";
  }
  traj_file.close();
}

}  // namespace gtsam
