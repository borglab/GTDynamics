"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_panda.py
 * @brief Test Franka Panda robot can be read and makes sense.
 * @author Frank Dellaert
"""

# pylint: disable=no-name-in-module, import-error, no-member

import unittest
from pathlib import Path

from gtsam import Point3, Pose3, Rot3, Values
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd


class TestSerial(GtsamTestCase):
    """Test Franka Panda robot can be read and makes sense."""

    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        model_path = Path(gtd.URDF_PATH) / "panda" / "panda.urdf"
        self.robot = gtd.CreateRobotFromFile(str(model_path))
        self.base_name = "link0"

    def test_forward_kinematics(self):
        """Test forward kinematics."""
        # Check number of links and joints
        self.assertEqual(len(self.robot.links()), 8)
        self.assertEqual(len(self.robot.joints()), 7)

        # Check FK at rest
        joint_angles = Values()
        for i in range(7):
            gtd.InsertJointAngle(joint_angles, i, 0.0)
        fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)
        # Use this to print: fk.print("fk", gtd.GTDKeyFormatter)
        sR7 = Rot3([
            [1, 0, 0],  #
            [0, -1, 0],
            [0, 0, -1]
        ])
        # regression
        expected_sT7 = Pose3(sR7, Point3(0.129315, 0.00227401, 0.88387))
        actual_sT7 = gtd.Pose(fk, 7)
        self.gtsamAssertEquals(actual_sT7, expected_sT7, tol=1e-3)


if __name__ == "__main__":
    unittest.main()
