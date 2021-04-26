"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_robot.py
 * @brief Test Robot class.
 * @author Varun Agrawal
"""

# pylint: disable=no-name-in-module, import-error, no-member
import os.path as osp
import unittest

import kinpy as kp
import numpy as np
from gtsam import Point3, Pose3, Rot3
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd


class TestRobot(GtsamTestCase):
    """Tests for the Robot class."""
    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        URDF_PATH = osp.join(osp.dirname(osp.realpath(__file__)), "..", "..",
                             "urdfs")
        self.ROBOT_MODEL = osp.join(URDF_PATH, "a1.urdf")

    def test_forward_kinematics(self):
        """Test if forward kinematics are correct via comparison to a 3rd party library."""
        def transform_to_pose(transform):
            return Pose3(Rot3.Quaternion(*transform.rot), transform.pos)

        def pose_to_transform(pose: Pose3):
            return kp.Transform(pose.rotation().quaternion(),
                                pose.translation())

        robot = gtd.CreateRobotFromFile(self.ROBOT_MODEL)
        robot_kp = kp.build_chain_from_urdf(open(self.ROBOT_MODEL).read())

        wTb = Pose3()

        th = {}
        joints = [
            "FR_hip_joint", "FR_upper_joint", "FR_lower_joint", "FL_hip_joint",
            "FL_upper_joint", "FL_lower_joint", "RR_hip_joint",
            "RR_upper_joint", "RR_lower_joint", "RL_hip_joint",
            "RL_upper_joint", "RL_lower_joint"
        ]

        # joint_angles = np.zeros(12)
        joint_angles = np.array([
            0., 0.924, -1.833, 0., 0.923, -1.834, 0., 0.878, -1.852, 0., 0.878,
            -1.853
        ])

        joint_angles_values = gtd.Values()

        for idx, joint in enumerate(joints):
            th[joint] = joint_angles[idx]
            gtd.InsertJointAngleDouble(joint_angles_values,
                                       robot.joint(joint).id(),
                                       joint_angles[idx])

        # print(joint_angles_values)
        fk = robot.forwardKinematics(joint_angles_values, 0, "trunk")

        ret = robot_kp.forward_kinematics(th, world=pose_to_transform(wTb))

        for link in robot.links():
            self.gtsamAssertEquals(transform_to_pose(ret[link.name()]),
                                   gtd.Pose(fk, link.id(), 0))


if __name__ == "__main__":
    unittest.main()
