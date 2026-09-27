"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_forward_inv_kinematics.py
 * @brief Forward kinematics factor and pose-goal inverse kinematics tests.
 * @author Varun Agrawal, Karthik Shaji
"""

import unittest
# pylint: disable=no-name-in-module, import-error, no-member
from pathlib import Path

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd


class TestRobot(GtsamTestCase):
    """Tests related to specific robot config."""
    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        model_file = Path(gtd.URDF_PATH) / "a1" / "a1.urdf"
        self.robot = gtd.CreateRobotFromFile(str(model_file))
        self.base_name = "trunk"

        self.joint_angles = gtsam.Values()
        angles = {
            0: 0.000174304,
            1: 0.923033,
            2: -1.83381,
            3: 0.000172539,
            4: 0.924125,
            5: -1.83302,
            6: 0.000137167,
            7: 0.878277,
            8: -1.85284,
            9: 0.000140037,
            10: 0.877832,
            11: -1.852,
        }
        for i, angle in angles.items():
            gtd.InsertJointAngle(self.joint_angles, i, 0, angle)

    def test_forward_kinematics_factor(self):
        """Test if forward kinematics are correct."""
        # self.joint_angles.print("", gtd.GTDKeyFormatter)
        t = 0
        end_link_name = "FR_lower"
        end_link = self.robot.link(end_link_name)
        factor = gtd.ForwardKinematicsFactor(
            X(t),
            gtd.PoseKey(end_link.id(), t),  # wTleg
            self.robot,
            self.base_name,
            end_link_name,
            self.joint_angles,
            gtsam.noiseModel.Isotropic.Sigma(6, 0.01),
            t)

        values = gtsam.Values()
        values.insert(X(t), gtsam.Pose3())

        fk = self.robot.forwardKinematics(self.joint_angles, t, self.base_name)
        end_link_pose = gtd.Pose(fk, end_link.id(), t)

        values.insert(gtd.PoseKey(end_link.id(), t), end_link_pose)

        graph = gtsam.NonlinearFactorGraph()
        graph.push_back(factor)
        np.testing.assert_almost_equal(graph.error(values), 0)

class TestManipulator(GtsamTestCase):
    """Forward/inverse kinematics round-trip on the bar_lab arm.

    Naming follows the aTb convention: w_T_com is the link CoM pose in the
    world frame, com_T_goal is the goal frame expressed in the CoM frame.
    """

    def setUp(self):
        # load example robot; fix the arm base so the gantry can't drift to
        # satisfy the pose goal (reduces the problem to the 6-DOF robot1 arm).
        model_file = Path(gtd.URDF_PATH) / "bar_lab.urdf"
        self.robot = gtd.CreateRobotFromFile(str(model_file)).fixLink("robot1_base")
        self.k = 0
        self.angle_rad = {
            "robot1_joint_1": 0.1,
            "robot1_joint_2": 0.2,
            "robot1_joint_3": 0.3,
            "robot1_joint_4": 0.4,
            "robot1_joint_5": 0.5,
            "robot1_joint_6": 0.6,
        }
        self.joint_angles = gtsam.Values()
        for j in self.robot.joints():
            gtd.InsertJointAngle(self.joint_angles, j.id(), self.k,
                                 self.angle_rad.get(j.name(), 0.0))

    def test_forward_inv_kinematics(self):
        """FK gives a reachable pose; IK with that PoseGoal recovers it."""
        k = self.k
        link6 = self.robot.link("robot1_link_6")

        # --- Forward kinematics: end-effector CoM pose in world (w_T_com). ---
        fk = self.robot.forwardKinematics(self.joint_angles, k, "robot1_base")
        w_T_com_goal = gtd.Pose(fk, link6.id(), k)

        # --- Build a full SE(3) pose goal on link_6 (identity TCP offset). ---
        com_T_goal = gtsam.Pose3()       # identity: constrain the link frame itself
        w_T_goal = w_T_com_goal          # desired world pose == the FK pose
        pose_goal = gtd.PoseGoal(link6, com_T_goal, w_T_goal)
        pose_goals = {k: pose_goal}  # inverse() takes {timestep: PoseGoal}

        # The FK solution must satisfy its own goal.
        self.assertTrue(pose_goal.satisfied(fk, k, 1e-5))

        # Square-penalty joint priors bias the solution toward the target config.
        joint_priors = gtsam.Values()
        for j in self.robot.joints():
            gtd.InsertJointAngle(joint_priors, j.id(), k,
                                 self.angle_rad.get(j.name(), 0.0))

        # --- Inverse kinematics with the SE(3) pose goal. ---
        # Note: the wrapper does not expose the optimization Method, so this runs
        # with the default SOFT_CONSTRAINTS (penalty) solve. The target config is
        # a zero-cost optimum, so the pose is still recovered closely.
        slice0 = gtd.Slice(k)
        kinematics = gtd.Kinematics()
        result = kinematics.inverse(slice0, self.robot, pose_goals, joint_priors,
                                    True)

        # The recovered end-effector pose matches the goal.
        w_T_com_achieved = gtd.Pose(result, link6.id(), k)
        self.gtsamAssertEquals(w_T_com_achieved, w_T_goal, 1e-3)
        self.assertTrue(pose_goal.satisfied(result, k, 1e-3))

        # The recovered arm joint angles match the target configuration.
        for j in self.robot.joints():
            if j.name() in self.angle_rad:
                got = gtd.JointAngle(result, j.id(), k)
                self.assertAlmostEqual(got, self.angle_rad[j.name()], delta=1e-2)

    def test_gantry_prior_sigma(self):
        """Looser gantry prior sigmas let the gantry absorb more of the motion."""
        k = self.k
        # Fix the gantry base so the gantry joints lie in the chain to the EE.
        robot = gtd.CreateRobotFromFile(
            str(Path(gtd.URDF_PATH) / "bar_lab.urdf")).fixLink("columns")
        link6 = robot.link("robot1_link_6")
        gantry_joints = ["bridge1_joint_EA_X", "robot1_joint_EA_Y",
                         "robot1_joint_EA_Z"]

        # Reachable goal from a config that uses the gantry (so it is redundant).
        config = {
            "bridge1_joint_EA_X": 2.0, "robot1_joint_EA_Y": 1.5,
            "robot1_joint_EA_Z": 0.5, "robot1_joint_1": 0.3,
            "robot1_joint_2": 0.4, "robot1_joint_3": -0.5,
            "robot1_joint_4": 0.2, "robot1_joint_5": 0.6, "robot1_joint_6": -0.3,
        }
        q = gtsam.Values()
        for j in robot.joints():
            gtd.InsertJointAngle(q, j.id(), k, config.get(j.name(), 0.0))
        fk = robot.forwardKinematics(q, k, "columns")
        pose_goals = {k: gtd.PoseGoal(link6, gtsam.Pose3(),
                                      gtd.Pose(fk, link6.id(), k))}
        slice0 = gtd.Slice(k)

        # Total gantry travel from the zero prior mean for a given gantry sigma.
        def gantry_travel(gantry_sigma):
            params = gtd.KinematicsParameters()
            for name in gantry_joints:
                params.setJointPriorSigma(robot.joint(name).key(), gantry_sigma)
            result = gtd.Kinematics(params).inverse(
                slice0, robot, pose_goals, gtsam.Values(), True)
            return sum(abs(gtd.JointAngle(result, robot.joint(n).id(), k))
                       for n in gantry_joints)

        # Looser gantry priors -> the gantry moves more to reach the goal.
        self.assertLess(gantry_travel(0.1), gantry_travel(10.0))






if __name__ == "__main__":
    unittest.main()
