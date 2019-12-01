/**
 * @file  DynamicsGraphBuilder.h
 * @brief robot arm motion planner using nonlinear factor graph
 * @Author:
 */
#pragma once

// #include <GaussianProcessPriorFactor.h>
// #include <GaussianProcessPriorPose3Factor.h>
// #include <JointLimitFactor.h>
#include <OptimizerSetting.h>
#include <PoseFactor.h>
#include <ToolPoseFactor.h>
// #include <ToolWrenchFactor.h>
#include <TorqueFactor.h>
#include <TwistAccelFactor.h>
#include <TwistFactor.h>
#include <WrenchEquivalenceFactor.h>
#include <WrenchFactors.h>
#include <iostream>
#include <utils.h>

#include <RobotJoint.h>
#include <RobotLink.h>
#include <UniversalRobot.h>

#include <boost/optional.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

namespace robot {

/* Shorthand for F_i_j_t, for wrenches at j-th joint on the i-th link at time t. */
gtsam::LabeledSymbol WrenchKey(int i, int j, int t) {
  return gtsam::LabeledSymbol('F', i*10+j, t); // a hack here for a key with 3 numbers
}

/* Shorthand for T_i_t, for torque on the i-th link at time t. */
gtsam::LabeledSymbol TorqueKey(int i, int t) {
  return gtsam::LabeledSymbol('T', i, t);
}

/* Shorthand for p_j_i, for COM pose on the j-th link at time i. */
gtsam::LabeledSymbol PoseKey(int j, int i) {
  return gtsam::LabeledSymbol('p', j, i);
}

/* Shorthand for V_j_i, for 6D link twist vector on the j-th link. */
gtsam::LabeledSymbol TwistKey(int j, int i) {
  return gtsam::LabeledSymbol('V', j, i);
}

/* Shorthand for A_j_i, for twist accelerations on the j-th link at time i. */
gtsam::LabeledSymbol TwistAccelKey(int j, int i) {
  return gtsam::LabeledSymbol('A', j, i);
}

/* Shorthand for q_j_i, for j-th joint angle at time i. */
gtsam::LabeledSymbol JointAngleKey(int j, int i) {
  return gtsam::LabeledSymbol('q', j, i);
}

/* Shorthand for v_j_i, for j-th joint velocity at time i. */
gtsam::LabeledSymbol JointVelKey(int j, int i) {
  return gtsam::LabeledSymbol('v', j, i);
}

/* Shorthand for a_j_i, for j-th joint acceleration at time i. */
gtsam::LabeledSymbol JointAccelKey(int j, int i) {
  return gtsam::LabeledSymbol('a', j, i);
}

/**
 * DynamicsGraphBuilder is a class which builds a factor graph to do kinodynamic
 * motion planning
 */
class DynamicsGraphBuilder {
private:
  manipulator::OptimizerSetting opt_;

public:
  using SphereCenters = std::vector<std::vector<gtsam::Point3>>;

  /**
   * Constructor
   * Keyword arguments:
   *  opt  -- optimizer setting
   */
  explicit DynamicsGraphBuilder() {
    opt_ = manipulator::OptimizerSetting();
    // set all dynamics related factors to be constrained
    opt_.bp_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.bv_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.ba_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.p_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.v_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.a_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.f_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.t_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.tf_cost_model = gtsam::noiseModel::Constrained::All(6);
    opt_.q_cost_model = gtsam::noiseModel::Constrained::All(1);
    opt_.qv_cost_model = gtsam::noiseModel::Constrained::All(1);

    opt_.setJointLimitCostModel(1e-3);

    opt_.setLM();
  }
  ~DynamicsGraphBuilder() {}

  /** return nonlinear factor graph of all factors
   */

  gtsam::NonlinearFactorGraph dynamics_factor_graph(
      UniversalRobot &robot, // add const here, add to links() function
      const boost::optional<gtsam::Vector3> &gravity = boost::none) const {
    using namespace gtsam;
    NonlinearFactorGraph graph;

    int t = 0;

    // Add joint factors to limit angle, velocity, acceleration, and torque.
    // graph.push_back(robot.jointLimitFactors(opt_.jl_cost_model, t));
  
    // add factors corresponding to links
    for (const auto& link : robot.links()) {
      const auto &connected_joints = link->getJoints();
      int i = link->getID();
      if (connected_joints.size() == 0) {
        graph.add(WrenchFactor0(TwistKey(i, t), TwistAccelKey(i, t),
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      } else if (connected_joints.size() == 1) {
        graph.add(WrenchFactor1(TwistKey(i, t), TwistAccelKey(i, t),
                                WrenchKey(i, connected_joints[0]->getID(), t),
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      } else if (connected_joints.size() == 2) {
        graph.add(WrenchFactor2(TwistKey(i, t), TwistAccelKey(i, t),
                                WrenchKey(i, connected_joints[0]->getID(), t),
                                WrenchKey(i, connected_joints[1]->getID(), t),
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      } else if (connected_joints.size() == 3) {
        graph.add(WrenchFactor3(TwistKey(i, t), TwistAccelKey(i, t),
                                WrenchKey(i, connected_joints[0]->getID(), t),
                                WrenchKey(i, connected_joints[1]->getID(), t),
                                WrenchKey(i, connected_joints[2]->getID(), t),
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      } else if (connected_joints.size() == 4) {
        graph.add(WrenchFactor4(TwistKey(i, t), TwistAccelKey(i, t),
                                WrenchKey(i, connected_joints[0]->getID(), t),
                                WrenchKey(i, connected_joints[1]->getID(), t),
                                WrenchKey(i, connected_joints[2]->getID(), t),
                                WrenchKey(i, connected_joints[3]->getID(), t),
                                PoseKey(i, t), opt_.f_cost_model,
                                link->inertiaMatrix(), gravity));
      } else {
        // std::cout<<"Wrench factor not defined\n";
        throw std::runtime_error("Wrench factor not defined");
      }
    }

    // add factors corresponding to joints
    for (const auto &joint : robot.joints()) {
      const auto &link_1 = joint->parentLink();
      const auto &link_2 = joint->childLink();
      int i1 = link_1->getID();
      int i2 = link_2.lock()->getID(); // cannot use methods for a weak ptr?
      int j = joint->getID();
      // add pose factor
      graph.add(manipulator::PoseFactor(PoseKey(i1, t), PoseKey(i2, t),
                                        JointAngleKey(j, t), opt_.p_cost_model,
                                        joint->pMc(), joint->screwAxis()));

      // add twist factor
      graph.add(manipulator::TwistFactor(TwistKey(i1, t), TwistKey(i2, t),
                                         JointAngleKey(j, t), JointVelKey(j, t),
                                         opt_.v_cost_model, joint->pMc(),
                                         joint->screwAxis()));

      // add twist acceleration factor
      graph.add(manipulator::TwistAccelFactor(
          TwistKey(i2, t), TwistAccelKey(i1, t), TwistAccelKey(i2, t),
          JointAngleKey(j, t), JointVelKey(j, t), JointAccelKey(j, t),
          opt_.a_cost_model, joint->pMc(), joint->screwAxis()));

      // add wrench equivalence factor
      graph.add(WrenchEquivalenceFactor(
          WrenchKey(i1, j, t), WrenchKey(i2, j, t), JointAngleKey(j, t),
          opt_.f_cost_model, joint->pMc(), joint->screwAxis()));

      // add torque factor
      graph.add(manipulator::TorqueFactor(WrenchKey(i2, j, t), TorqueKey(j, t),
                                          opt_.t_cost_model,
                                          joint->screwAxis()));
    }
    return graph;
  }
};

} // namespace robot
