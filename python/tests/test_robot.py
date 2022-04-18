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

import gtdynamics as gtd
import numpy as np
from gtsam import Point3, Pose3, Rot3
from gtsam.utils.test_case import GtsamTestCase


class TestRobot(GtsamTestCase):
    """Tests for the Robot class."""
    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        self.ROBOT_MODEL = osp.join(gtd.URDF_PATH, "a1", "a1.urdf")

    def test_fixed_joint(self):
        """Test if the fixed joint is parsed correctly."""
        robot = gtd.CreateRobotFromFile(self.ROBOT_MODEL)
        # Try to get the fixed links
        self.assertIsNotNone(robot.link("FR_toe"))
        self.assertIsNotNone(robot.link("FL_toe"))
        self.assertIsNotNone(robot.link("RR_toe"))
        self.assertIsNotNone(robot.link("RL_toe"))

    def test_forward_kinematics(self):
        """Test if FK is correct via comparison to a 3rd party library."""
        robot = gtd.CreateRobotFromFile(self.ROBOT_MODEL)

        th = {}
        joints = [
            "FR_hip_joint", "FR_upper_joint", "FR_lower_joint", "FL_hip_joint",
            "FL_upper_joint", "FL_lower_joint", "RR_hip_joint",
            "RR_upper_joint", "RR_lower_joint", "RL_hip_joint",
            "RL_upper_joint", "RL_lower_joint"
        ]

        joint_angles = np.array(
            [0., 0.1, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

        joint_angles_values = gtd.Values()

        for idx, joint in enumerate(joints):
            th[joint] = joint_angles[idx]
            gtd.InsertJointAngle(joint_angles_values,
                                 robot.joint(joint).id(), joint_angles[idx])

        # Forward kinematics via GTDynamics.
        fk = robot.forwardKinematics(joint_angles_values, 0, "trunk")

        # Transform from lower link CoM to CoM of combined lower+toe link.
        # Lower + toe links are combined as they are connected by fixed joint.
        lowerTcom = Pose3(Rot3(), Point3(-0.00170571, 4.63725e-07, -0.0245871))

        # Dict of link-transforms where the transform is
        # from the link frame to the link CoM frame.
        lTcom_adjustments = {
            "FL_hip":
            Pose3(Rot3(), Point3(-0.003311, 0.000635, 3.1e-05)),
            "FL_lower":
            Pose3(Rot3(), Point3(0.006435, 0.0, -0.107388)).compose(lowerTcom),
            "FL_upper":
            Pose3(Rot3(), Point3(-0.003237, -0.022327, -0.027326)),
            "FR_hip":
            Pose3(Rot3(), Point3(-0.003311, -0.000635, 3.1e-05)),
            "FR_lower":
            Pose3(Rot3(), Point3(0.006435, 0.0, -0.107388)).compose(lowerTcom),
            "FR_upper":
            Pose3(Rot3(), Point3(-0.003237, 0.022327, -0.027326)),
            "RL_hip":
            Pose3(Rot3(), Point3(0.003311, 0.000635, 3.1e-05)),
            "RL_lower":
            Pose3(Rot3(), Point3(0.006435, 0.0, -0.107388)).compose(lowerTcom),
            "RL_upper":
            Pose3(Rot3(), Point3(-0.003237, -0.022327, -0.027326)),
            "RR_hip":
            Pose3(Rot3(), Point3(0.003311, -0.000635, 3.1e-05)),
            "RR_lower":
            Pose3(Rot3(), Point3(0.006435, 0.0, -0.107388)).compose(lowerTcom),
            "RR_upper":
            Pose3(Rot3(), Point3(-0.003237, 0.022327, -0.027326)),
            "trunk":
            Pose3(Rot3(), Point3(0.012731, 0.002186, 0.000515)),
        }

        # True FK poses computed using kinpy
        bTl_poses = {
            "FL_hip":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (0.170269, 0.044814, -0.000515)),
            "FL_lower":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (0.170269, 0.129864, -0.200515)),
            "FL_upper":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (0.170269, 0.129864, -0.000515)),
            "FR_hip":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (0.170269, -0.049186, -0.000515)),
            "FR_lower":
            Pose3(Rot3.Quaternion(0.99875026, 0.0, 0.04997917, 0.),
                  (0.15030232, -0.134236, -0.19951583)),
            "FR_upper":
            Pose3(Rot3.Quaternion(0.99875026, 0.0, 0.04997917, 0.),
                  (0.170269, -0.134236, -0.000515)),
            "RL_hip":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.195731, 0.044814, -0.000515)),
            "RL_lower":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.195731, 0.129864, -0.200515)),
            "RL_upper":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.195731, 0.129864, -0.000515)),
            "RR_hip":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.195731, -0.049186, -0.000515)),
            "RR_lower":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.195731, -0.134236, -0.200515)),
            "RR_upper":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.195731, -0.134236, -0.000515)),
            "trunk":
            Pose3(Rot3.Quaternion(1., 0., 0., 0.),
                  (-0.012731, -0.002186, -0.000515)),
        }
        # Test all the links
        for link in robot.links():
            lTcom = lTcom_adjustments[link.name()]
            bTl = bTl_poses[link.name()]
            expected_bTcom = bTl.compose(lTcom)

            actual_bTcom = gtd.Pose(fk, link.id(), 0)
            self.gtsamAssertEquals(actual_bTcom, expected_bTcom, tol=1e-3)


if __name__ == "__main__":
    unittest.main()
