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

using gtsam::Key;
using gtsam::Point3;
using gtsam::Vector3_;
using gtsam::Vector6;
using gtsam::Vector6_;

std::vector<std::vector<JointSharedPtr>> ChainDynamicsGraph::getChainJoints(
    const Robot &robot) {
  std::vector<JointSharedPtr> FR(3), FL(3), RR(3), RL(3);

  int loc = 0;
  // TODO(Varun): This seems very specific to the A1, which is sad.
  for (auto &&joint : robot.joints()) {
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

Chain BuildChain(std::vector<JointSharedPtr> &joints) {
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
    std::vector<std::vector<JointSharedPtr>> &chain_joints) {
  Chain composed_fr, composed_fl, composed_rr, composed_rl;

  composed_fl = BuildChain(chain_joints[0]);
  composed_fr = BuildChain(chain_joints[1]);
  composed_rl = BuildChain(chain_joints[2]);
  composed_rr = BuildChain(chain_joints[3]);

  // TODO(Varun): make the code agnostic to quadrupeds, bipeds, hexapods, etc
  std::vector<Chain> composed_chains{composed_fl, composed_fr, composed_rl,
                                     composed_rr};

  return composed_chains;
}

NonlinearFactorGraph ChainDynamicsGraph::dynamicsFactors(
    const Robot &robot, const int t,
    const std::optional<PointOnLinks> &contact_points,
    const std::optional<double> &mu) const {
  // Initialize graph
  NonlinearFactorGraph graph;

  // Set Gravity Wrench
  Vector6 gravityMass;
  auto graph_gravity = gravity();
  gravityMass << 0.0, 0.0, 0.0, gravity() * trunk_mass_;
  Vector6_ gravity_wrench(gravityMass);

  // Create expression for wrench constraint on trunk
  Vector6_ trunk_wrench_constraint = gravity_wrench;

  // Set tolerances

  // Get tolerance
  Vector6 wrench_tolerance = opt().f_cost_model->sigmas();
  Vector3 contact_tolerance = opt().cm_cost_model->sigmas();

  std::vector<Key> wrench_keys;

  for (int i = 0; i < 4; ++i) {
    bool foot_in_contact = false;
    // Get key for wrench at hip joint with id 0
    const Key wrench_key_3i_T = WrenchKey(0, 3 * i, t);

    // create expression for the wrench and initialize to zero
    Vector6_ wrench_3i_T(Vector6::Zero());

    // Set wrench expression on trunk by leg, according to contact
    for (auto &&cp : *contact_points) {
      if (cp.link->id() == 3 * (i + 1)) {
        wrench_3i_T = Vector6_(wrench_key_3i_T);
        foot_in_contact = true;
      }
    }

    // add wrench to trunk constraint
    wrench_keys.push_back(wrench_key_3i_T);

    // Get expression for end effector wrench using chain
    Vector6_ wrench_end_effector = composed_chains_[i].AdjointWrenchConstraint3(
        chain_joints_[i], wrench_key_3i_T, t);

    // Create contact constraint
    Point3 contact_in_com(0.0, 0.0, -0.07);
    Vector3_ contact_constraint = ContactDynamicsMomentConstraint(
        wrench_end_effector,
        gtsam::Pose3(gtsam::Rot3(), (-1) * contact_in_com));
    auto contact_expression =
        VectorExpressionEquality<3>(contact_constraint, contact_tolerance);
    if (foot_in_contact)
      graph.add(contact_expression.createFactor(1));
    else {
      Vector6 wrench_zero = gtsam::Z_6x1;
      graph.addPrior(wrench_key_3i_T, wrench_zero, opt().f_cost_model);
    }
  }

  // Add wrench factor for trunk link
  graph.add(WrenchFactor(opt().f_cost_model, robot.link("trunk"), wrench_keys,
                         t, gravity()));

  return graph;
}

gtsam::NonlinearFactorGraph ChainDynamicsGraph::qFactors(
    const Robot &robot, const int t,
    const std::optional<PointOnLinks> &contact_points) const {
  NonlinearFactorGraph graph;

  // Get Pose key for base link
  const Key base_key = PoseKey(0, t);

  // Get tolerance
  Vector6 tolerance = opt().p_cost_model->sigmas();

  for (int i = 0; i < 4; ++i) {
    // Get end effector key
    const Key end_effector_key = PoseKey(3 * (i + 1), t);

    // Get expression for end effector pose
    gtsam::Vector6_ chain_expression = composed_chains_[i].Poe3Factor(
        chain_joints_[i], base_key, end_effector_key, t);

    auto chain_constraint =
        VectorExpressionEquality<6>(chain_expression, tolerance);

    graph.add(chain_constraint.createFactor(1.0));
  }

  // Add contact factors.
  if (contact_points) {
    for (auto &&cp : *contact_points) {
      ContactHeightFactor contact_pose_factor(
          PoseKey(cp.link->id(), t), opt().cp_cost_model, cp.point, gravity());
      graph.add(contact_pose_factor);
    }
  }

  return graph;
}

gtsam::NonlinearFactorGraph ChainDynamicsGraph::dynamicsFactorGraph(
    const Robot &robot, const int t,
    const std::optional<PointOnLinks> &contact_points,
    const std::optional<double> &mu) const {
  NonlinearFactorGraph graph;
  graph.add(qFactors(robot, t, contact_points));
  // TODO(Varun): Why are these commented out?
  // graph.add(vFactors(robot, t, contact_points));
  // graph.add(aFactors(robot, t, contact_points));
  graph.add(dynamicsFactors(robot, t, contact_points, mu));
  return graph;
}

}  // namespace gtdynamics
