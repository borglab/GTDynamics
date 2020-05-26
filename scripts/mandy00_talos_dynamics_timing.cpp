/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  mandy00_talos_dynamics_timing.cpp
 * @brief check the timing of different elimination orders for inverse dynamics
 * of talos robot
 * @Author: Mandy Xie
 */
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>

#include <iostream>

using namespace gtdynamics;
using namespace gtsam;

int main(int argc, char** argv) {
  auto talos = Robot("../../urdfs/talos.urdf");
  // Build a factor graph with all the kinodynamics constraints.
  auto graph_builder = DynamicsGraph();
  Vector3 gravity = (Vector(3) << 0, 0, -9.8).finished();
  int t = 0;
  Robot::JointValues joint_angles, joint_vels, joint_accels;
  // joint_angles["torso_1_joint"] = 0;
  // joint_vels["torso_1_joint"] = 0;
  // joint_accels["torso_1_joint"] = 0;
  // joint_angles["torso_2_joint"] = 0;
  // joint_vels["torso_2_joint"] = 0;
  // joint_accels["torso_2_joint"] = 0;
  // joint_angles["head_1_joint"] = 0;
  // joint_vels["head_1_joint"] = 0;
  // joint_accels["head_1_joint"] = 0;
  // joint_angles["head_2_joint"] = 0;
  // joint_vels["head_2_joint"] = 0;
  // joint_accels["head_2_joint"] = 0;
  // joint_angles["arm_left_1_joint"] = 0;
  // joint_vels["arm_left_1_joint"] = 0;
  // joint_accels["arm_left_1_joint"] = 0;
  // joint_angles["arm_left_2_joint"] = 0;
  // joint_vels["arm_left_2_joint"] = 0;
  // joint_accels["arm_left_2_joint"] = 0;
  // joint_angles["arm_left_3_joint"] = 0;
  // joint_vels["arm_left_3_joint"] = 0;
  // joint_accels["arm_left_3_joint"] = 0;
  // joint_angles["arm_left_4_joint"] = 0;
  // joint_vels["arm_left_4_joint"] = 0;
  // joint_accels["arm_left_4_joint"] = 0;
  // joint_angles["arm_left_5_joint"] = 0;
  // joint_vels["arm_left_5_joint"] = 0;
  // joint_accels["arm_left_5_joint"] = 0;
  // joint_angles["arm_left_6_joint"] = 0;
  // joint_vels["arm_left_6_joint"] = 0;
  // joint_accels["arm_left_6_joint"] = 0;
  // joint_angles["arm_left_7_joint"] = 0;
  // joint_vels["arm_left_7_joint"] = 0;
  // joint_accels["arm_left_7_joint"] = 0;
  // joint_angles["arm_right_1_joint"] = 0;
  // joint_vels["arm_right_1_joint"] = 0;
  // joint_accels["arm_right_1_joint"] = 0;
  // joint_angles["arm_right_2_joint"] = 0;
  // joint_vels["arm_right_2_joint"] = 0;
  // joint_accels["arm_right_2_joint"] = 0;
  // joint_angles["arm_right_3_joint"] = 0;
  // joint_vels["arm_right_3_joint"] = 0;
  // joint_accels["arm_right_3_joint"] = 0;
  // joint_angles["arm_right_4_joint"] = 0;
  // joint_vels["arm_right_4_joint"] = 0;
  // joint_accels["arm_right_4_joint"] = 0;
  // joint_angles["arm_right_5_joint"] = 0;
  // joint_vels["arm_right_5_joint"] = 0;
  // joint_accels["arm_right_5_joint"] = 0;
  // joint_angles["arm_right_6_joint"] = 0;
  // joint_vels["arm_right_6_joint"] = 0;
  // joint_accels["arm_right_6_joint"] = 0;
  // joint_angles["arm_right_7_joint"] = 0;
  // joint_vels["arm_right_7_joint"] = 0;
  // joint_accels["arm_right_7_joint"] = 0;
  // joint_angles["gripper_left_joint"] = 0;
  // joint_vels["gripper_left_joint"] = 0;
  // joint_accels["gripper_left_joint"] = 0;
  // joint_angles["gripper_right_joint"] = 0;
  // joint_vels["gripper_right_joint"] = 0;
  // joint_accels["gripper_right_joint"] = 0;
  // joint_angles["leg_left_1_joint"] = 0;
  // joint_vels["leg_left_1_joint"] = 0;
  // joint_accels["leg_left_1_joint"] = 0;
  // joint_angles["leg_left_2_joint"] = 0;
  // joint_vels["leg_left_2_joint"] = 0;
  // joint_accels["leg_left_2_joint"] = 0;
  // joint_angles["leg_left_3_joint"] = 0;
  // joint_vels["leg_left_3_joint"] = 0;
  // joint_accels["leg_left_3_joint"] = 0;
  // joint_angles["leg_left_4_joint"] = 0;
  // joint_vels["leg_left_4_joint"] = 0;
  // joint_accels["leg_left_4_joint"] = 0;
  // joint_angles["leg_left_5_joint"] = 0;
  // joint_vels["leg_left_5_joint"] = 0;
  // joint_accels["leg_left_5_joint"] = 0;
  // joint_angles["leg_left_6_joint"] = 0;
  // joint_vels["leg_left_6_joint"] = 0;
  // joint_accels["leg_left_6_joint"] = 0;
  // joint_angles["leg_right_1_joint"] = 0;
  // joint_vels["leg_right_1_joint"] = 0;
  // joint_accels["leg_right_1_joint"] = 0;
  // joint_angles["leg_right_2_joint"] = 0;
  // joint_vels["leg_right_2_joint"] = 0;
  // joint_accels["leg_right_2_joint"] = 0;
  // joint_angles["leg_right_3_joint"] = 0;
  // joint_vels["leg_right_3_joint"] = 0;
  // joint_accels["leg_right_3_joint"] = 0;
  // joint_angles["leg_right_4_joint"] = 0;
  // joint_vels["leg_right_4_joint"] = 0;
  // joint_accels["leg_right_4_joint"] = 0;
  // joint_angles["leg_right_5_joint"] = 0;
  // joint_vels["leg_right_5_joint"] = 0;
  // joint_accels["leg_right_5_joint"] = 0;
  // joint_angles["leg_right_6_joint"] = 0;
  // joint_vels["leg_right_6_joint"] = 0;
  // joint_accels["leg_right_6_joint"] = 0;

  std::string prior_link_name = "base_link";
  auto l1 = talos.getLinkByName(prior_link_name);
  Vector6 V_l1 = Vector6::Zero();
  auto fk_results = talos.forwardKinematics(joint_angles, joint_vels,
                                            prior_link_name, l1->wTcom(), V_l1);

  // test inverse dynamics
  Values result = graph_builder.linearSolveID(
      talos, t, joint_angles, joint_vels, joint_accels, fk_results, gravity);

  return 0;
}
