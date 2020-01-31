/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsGraphBuilder.h
 * @brief Builds a dynamics graph from a UniversalRobot object.
 * @author Yetong Zhang, Alejandro Escontrela
 */

#include "dynamics/DynamicsGraph.h"

#include <PoseFactor.h>
#include <TorqueFactor.h>
#include <TwistAccelFactor.h>
#include <TwistFactor.h>
#include <WrenchEquivalenceFactor.h>
#include <WrenchFactors.h>
#include <WrenchPlanarFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>

#include <utils.h>

#include <vector>
#include <iostream>

namespace robot {

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::dynamicsFactorGraph(
    const UniversalRobot &robot, const int t,
    const boost::optional<gtsam::Vector3> &gravity,
    const boost::optional<gtsam::Vector3> &plannar_axis) const {
  gtsam::NonlinearFactorGraph graph;

  // add factors corresponding to links
  for (int idx = 0; idx < robot.numLinks(); idx++) {
    auto link = robot.links()[idx];

    int i = link->getID();
    if (link->isFixed()) {
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          PoseKey(i, t), link->getFixedPose(),
          gtsam::noiseModel::Constrained::All(6)));
      graph.add(gtsam::PriorFactor<gtsam::Vector6>(
          TwistKey(i, t), gtsam::Vector6::Zero(),
          gtsam::noiseModel::Constrained::All(6)));
      graph.add(gtsam::PriorFactor<gtsam::Vector6>(
          TwistAccelKey(i, t), gtsam::Vector6::Zero(),
          gtsam::noiseModel::Constrained::All(6)));
    } else {
      // Get all wrench keys associated with this link.
      const auto &connected_joints = link->getJoints();
      std::vector<gtsam::LabeledSymbol> wrenches;
      for (auto &&joint : connected_joints)
        wrenches.push_back(WrenchKey(i, joint->getID(), t));

      // Create wrench factor for this link.
      if (wrenches.size() == 0)
        graph.add(WrenchFactor0(TwistKey(i, t), TwistAccelKey(i, t),
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      else if (wrenches.size() == 1)
        graph.add(WrenchFactor1(TwistKey(i, t), TwistAccelKey(i, t),
                                wrenches[0], PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      else if (wrenches.size() == 2)
        graph.add(WrenchFactor2(
            TwistKey(i, t), TwistAccelKey(i, t), wrenches[0], wrenches[1],
            PoseKey(i, t), opt_.f_cost_model, link->inertiaMatrix(), gravity));
      else if (wrenches.size() == 3)
        graph.add(WrenchFactor3(TwistKey(i, t), TwistAccelKey(i, t),
                                wrenches[0], wrenches[1], wrenches[2],
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      else if (wrenches.size() == 4)
        graph.add(WrenchFactor4(TwistKey(i, t), TwistAccelKey(i, t),
                                wrenches[0], wrenches[1], wrenches[2],
                                wrenches[3], PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      else
        throw std::runtime_error("Wrench factor not defined");
    }
  }

  // add factors corresponding to joints
  for (auto &&joint : robot.joints()) {
    const auto &link_1 = joint->parentLink();
    const auto &link_2 = joint->childLink().lock();
    int i1 = link_1->getID();
    int i2 = link_2->getID();  // cannot use methods for a weak ptr?
    int j = joint->getID();
    // add pose factor
    graph.add(manipulator::PoseFactor(PoseKey(i1, t), PoseKey(i2, t),
                                      JointAngleKey(j, t), opt_.p_cost_model,
                                      joint->McpCom(), joint->screwAxis()));

    // add twist factor
    graph.add(manipulator::TwistFactor(TwistKey(i1, t), TwistKey(i2, t),
                                       JointAngleKey(j, t), JointVelKey(j, t),
                                       opt_.v_cost_model, joint->McpCom(),
                                       joint->screwAxis()));

    // add twist acceleration factor
    graph.add(manipulator::TwistAccelFactor(
        TwistKey(i2, t), TwistAccelKey(i1, t), TwistAccelKey(i2, t),
        JointAngleKey(j, t), JointVelKey(j, t), JointAccelKey(j, t),
        opt_.a_cost_model, joint->McpCom(), joint->screwAxis()));

    // add wrench equivalence factor
    graph.add(WrenchEquivalenceFactor(WrenchKey(i1, j, t), WrenchKey(i2, j, t),
                                      JointAngleKey(j, t), opt_.f_cost_model,
                                      joint->McpCom(), joint->screwAxis()));

    // add torque factor
    graph.add(manipulator::TorqueFactor(WrenchKey(i2, j, t), TorqueKey(j, t),
                                        opt_.t_cost_model, joint->screwAxis()));

    // add planar wrench factor
    if (plannar_axis) {
      graph.add(WrenchPlanarFactor(WrenchKey(i2, j, t),
                                   gtsam::noiseModel::Constrained::All(3),
                                   *plannar_axis));
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph DynamicsGraphBuilder::forwardDynamicsPriors(
    const UniversalRobot &robot, const int t, const gtsam::Vector &joint_angles,
    const gtsam::Vector &joint_vels, const gtsam::Vector &torques) const {
  gtsam::NonlinearFactorGraph graph;
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->getID();
    graph.add(
        gtsam::PriorFactor<double>(JointAngleKey(j, t), joint_angles[idx],
                                   gtsam::noiseModel::Constrained::All(1)));
    graph.add(
        gtsam::PriorFactor<double>(JointVelKey(j, t), joint_vels[idx],
                                   gtsam::noiseModel::Constrained::All(1)));
    graph.add(gtsam::PriorFactor<double>(
        TorqueKey(j, t), torques[idx], gtsam::noiseModel::Constrained::All(1)));
  }
  return graph;
}

gtsam::Vector DynamicsGraphBuilder::jointAccels(const UniversalRobot &robot,
                                                const gtsam::Values &result,
                                                const int t) {
  gtsam::Vector joint_accels = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->getID();
    joint_accels[idx] = result.atDouble(JointAccelKey(j, t));
  }
  return joint_accels;
}

gtsam::Vector DynamicsGraphBuilder::jointVels(const UniversalRobot &robot,
                                              const gtsam::Values &result,
                                              const int t) {
  gtsam::Vector joint_vels = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->getID();
    joint_vels[idx] = result.atDouble(JointVelKey(j, t));
  }
  return joint_vels;
}

gtsam::Vector DynamicsGraphBuilder::jointAngles(const UniversalRobot &robot,
                                                const gtsam::Values &result,
                                                const int t) {
  gtsam::Vector joint_angles = gtsam::Vector::Zero(robot.numJoints());
  auto joints = robot.joints();
  for (int idx = 0; idx < robot.numJoints(); idx++) {
    auto joint = joints[idx];
    int j = joint->getID();
    joint_angles[idx] = result.atDouble(JointAngleKey(j, t));
  }
  return joint_angles;
}

gtsam::Values DynamicsGraphBuilder::zeroValues(const UniversalRobot &robot,
                                               const int t) {
  gtsam::Vector zero_twists = gtsam::Vector6::Zero(),
                zero_accels = gtsam::Vector6::Zero(),
                zero_wrenches = gtsam::Vector6::Zero(),
                zero_torque = gtsam::Vector1::Zero(),
                zero_q = gtsam::Vector1::Zero(),
                zero_v = gtsam::Vector1::Zero(),
                zero_a = gtsam::Vector1::Zero();
  gtsam::Values zero_values;
  for (auto &link : robot.links()) {
    int i = link->getID();
    zero_values.insert(PoseKey(i, t), link->Twcom());
    zero_values.insert(TwistKey(i, t), zero_twists);
    zero_values.insert(TwistAccelKey(i, t), zero_accels);
  }
  for (auto &joint : robot.joints()) {
    int j = joint->getID();
    auto parent_link = joint->parentLink();
    auto child_link = joint->childLink().lock();
    zero_values.insert(WrenchKey(parent_link->getID(), j, t), zero_wrenches);
    zero_values.insert(WrenchKey(child_link->getID(), j, t), zero_wrenches);
    zero_values.insert(TorqueKey(j, t), zero_torque[0]);
    zero_values.insert(JointAngleKey(j, t), zero_q[0]);
    zero_values.insert(JointVelKey(j, t), zero_v[0]);
    zero_values.insert(JointAccelKey(j, t), zero_a[0]);
  }
  return zero_values;
}

gtsam::Values DynamicsGraphBuilder::optimize(
    const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &init_values,
    OptimizerType optim_type) {
  if (optim_type == OptimizerType::GaussNewton) {
    gtsam::GaussNewtonOptimizer optimizer(graph, init_values);
    optimizer.optimize();
    return optimizer.values();
  } else if (optim_type == OptimizerType::LM) {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values);
    optimizer.optimize();
    return optimizer.values();
  } else if (optim_type == OptimizerType::PDL) {
    gtsam::DoglegOptimizer optimizer(graph, init_values);
    optimizer.optimize();
    return optimizer.values();
  } else {
    throw std::runtime_error("optimizer not implemented yet");
  }
}

void print_key(const gtsam::Key &key) {
  auto symb = gtsam::LabeledSymbol(key);
  char ch = symb.chr();
  int index = symb.label();
  int t = symb.index();
  if (ch == 'F') {
    std::cout << ch << int(index / 16) << index % 16 << "_" << t;
  } else if (ch == 't') {
    if (index == 0) {  // phase key
      std::cout << "dt" << t;
    } else if (index == 1) {  // time key
      std::cout << "t" << t;
    } else {  // time to open valve
      std::cout << "ti" << t;
    }
  } else {
    std::cout << ch << index << "_" << t;
  }
  std::cout << "\t";
}

// print the factors of the factor graph
void DynamicsGraphBuilder::print_values(const gtsam::Values &values) {
  for (auto &key : values.keys()) {
    print_key(key);
    std::cout << "\n";
    values.at(key).print();
    std::cout << "\n";
  }
}

// print the factors of the factor graph
void DynamicsGraphBuilder::print_graph(
    const gtsam::NonlinearFactorGraph &graph) {
  for (auto &factor : graph) {
    for (auto &key : factor->keys()) {
      print_key(key);
    }
    std::cout << "\n";
  }
}

}  // namespace robot
