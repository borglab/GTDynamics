/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file DynamicsGraph.h
 * @brief Builds a lean dynamics graph from a Robot object.
 * @author Dan Barladeanu
 */

#include "gtdynamics/dynamics/ChainDynamicsGraph.h"

namespace gtdynamics {

std::vector<std::vector<JointSharedPtr>> ChainDynamicsGraph::getChainJoints(const Robot& robot) {
  std::vector<JointSharedPtr> FR(3), FL(3), RR(3), RL(3);

  int loc = 0;
  for (auto&& joint : robot.joints()) {
    if (joint->name().find("lower") != std::string::npos) {
      loc = 2;
    }
    if (joint->name().find("upper") != std::string::npos) {
      loc = 1;
    }
    if (joint->name().find("hip") != std::string::npos) {
      loc = 0;
    }
    if (joint->name().find("FR") != std::string::npos) {
      FR[loc] = joint;
    }
    if (joint->name().find("FL") != std::string::npos) {
      FL[loc] = joint;
    }
    if (joint->name().find("RR") != std::string::npos) {
      RR[loc] = joint;
    }
    if (joint->name().find("RL") != std::string::npos) {
      RL[loc] = joint;
    }
  }

  std::vector<std::vector<JointSharedPtr>> chain_joints{FL, FR, RL, RR};

  return chain_joints;
}

Chain BuildChain(std::vector<JointSharedPtr>& joints) {
  auto j0 = joints[0];
  auto j1 = joints[1];
  auto j2 = joints[2];

  // Calculate all relevant relative poses.
  Pose3 M_T_H = j0->pMc();
  Pose3 M_H_T = M_T_H.inverse();
  Pose3 M_H_U = j1->pMc();
  Pose3 M_U_H = M_H_U.inverse();
  Pose3 M_U_L = j2->pMc();
  Pose3 M_L_U = M_U_L.inverse();
  Pose3 M_T_L = M_T_H * M_H_U * M_U_L;
  Pose3 M_L_T = M_T_L.inverse();

  // Create chains
  Chain chain1(M_T_H, j0->cScrewAxis());
  Chain chain2(M_H_U, j1->cScrewAxis());
  Chain chain3(M_U_L, j2->cScrewAxis());

  std::vector<Chain> chains{chain1, chain2, chain3};

  Chain composed = Chain::compose(chains);

  return composed;
}

std::vector<Chain> ChainDynamicsGraph::getComposedChains(
    std::vector<std::vector<JointSharedPtr>>& chain_joints) {
  Chain composed_fr, composed_fl, composed_rr, composed_rl;

  composed_fl = BuildChain(chain_joints[0]);
  composed_fr = BuildChain(chain_joints[1]);
  composed_rl = BuildChain(chain_joints[2]);
  composed_rr = BuildChain(chain_joints[3]);

  std::vector<Chain> composed_chains{composed_fl, composed_fr, composed_rl,
                                     composed_rr};

  return composed_chains;
}

NonlinearFactorGraph ChainDynamicsGraph::dynamicsFactors(
    const Robot &robot,const int t,
    const boost::optional<PointOnLinks> &contact_points,
    const boost::optional<double> &mu) const {

    // Initialize graph
    NonlinearFactorGraph graph;

    // Set Gravity Wrench
    Vector6 gravityMass;
    auto graph_gravity = gravity();
    if (graph_gravity) 
      gravityMass << 0.0, 0.0, 0.0, *graph_gravity * trunk_mass_;
    else
      gravityMass << 0.0, 0.0, 0.0, 0.0, 0.0, (-9.8) * trunk_mass_;
    Vector6_ gravity_wrench(gravityMass);

    // Create expression for wrench constraint on trunk
    Vector6_ trunk_wrench_constraint = gravity_wrench;

    // Set tolerances
    Vector3 contact_tolerance = Vector3::Ones() * dynamics_tolerance_;
    Vector3 chain_tolerance = Vector3::Ones() * dynamics_tolerance_;
    Vector6 wrench_tolerance = Vector6::Ones() * dynamics_tolerance_;

    for (int i = 0 ; i < 4 ; ++i) {
    // Get key for wrench at hip joint with id 0
    const Key wrench_key_3i_T = WrenchKey(0, 3*i, t);

    // create expression for the wrench
    Vector6_ wrench_3i_T(wrench_key_3i_T);

    // add wrench to trunk constraint
    trunk_wrench_constraint += wrench_3i_T;

    // Get expression for chain on the leg
    Vector3_ expression_chain =
        composed_chains_[i].ChainConstraint3(chain_joints_[i], wrench_key_3i_T, t);

    // Add constraint for chain
    auto chain_constraint = VectorExpressionEquality<3>(expression_chain,
                                                            chain_tolerance);

    graph.add(chain_constraint.createFactor(1));

    // constraint on zero angles
    for (int j = 0; j < 3; ++j) {
      const int joint_id = chain_joints_[i][j]->id();
      Double_ angle(JointAngleKey(joint_id, t));
      auto angle_constraint = DoubleExpressionEquality(angle,
                                                          angle_tolerance_);
      graph.add(angle_constraint.createFactor(1));
      if (j<2) {
        Double_ tor(TorqueKey(joint_id, t));
        auto torque_constraint = DoubleExpressionEquality(tor,
                                                            torque_tolerance_);
        graph.add(torque_constraint.createFactor(1));
      }
    }

    // For contact constraint, calculate the expression for the wrench of joint 2
    // on the lower link
    Vector6_ wrench_0_H =
        (-1) * chain_joints_[i][0]->childAdjointWrench(wrench_3i_T, t);
    Vector6_ wrench_1_U =
        (-1) * chain_joints_[i][1]->childAdjointWrench(wrench_0_H, t);
    Vector6_ wrench_2_L =
        (-1) * chain_joints_[i][2]->childAdjointWrench(wrench_1_U, t);

    // Create contact constraint
    Point3 contact_in_com(0.0, 0.0, -0.07);
    Vector3_ contact_constraint = ContactDynamicsMomentConstraint(
        wrench_2_L, gtsam::Pose3(gtsam::Rot3(), (-1) * contact_in_com));
    auto contact_expression = VectorExpressionEquality<3>(contact_constraint,
                                                           contact_tolerance);
    graph.add(contact_expression.createFactor(1));
  }
  // Add trunk wrench constraint to constraints
  auto trunk_constraint = VectorExpressionEquality<6>(
      trunk_wrench_constraint, wrench_tolerance);

  graph.add(trunk_constraint.createFactor(1));

  return graph;
}

} // namesapce gtdynamics