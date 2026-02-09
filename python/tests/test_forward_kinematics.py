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


if __name__ == "__main__":
    unittest.main()
