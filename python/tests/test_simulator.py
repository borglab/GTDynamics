"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_simulator.py
 * @brief Test Simulator class.
 * @author Frank Dellaert, Yetong Zhang, and Varun Agrawal
"""

# pylint: disable=no-name-in-module, import-error, no-member

import os.path as osp
import unittest

import gtdynamics as gtd
import numpy as np
from gtsam import Values
from gtsam.utils.test_case import GtsamTestCase


class TestLink(GtsamTestCase):
    """Test Simulator Class"""

    URDF_PATH = osp.join(osp.dirname(osp.realpath(__file__)), "..", "..",
                         "models", "urdfs")

    def test_simulator(self):
        """Test simulating two steps on a simple one-link robot."""
        # load example robot
        robot = gtd.CreateRobotFromFile(
            osp.join(self.URDF_PATH, "test", "simple_urdf.urdf"), "")
        robot = robot.fixLink("l1")
        gravity = np.zeros(3)
        planar_axis = np.asarray([1, 0, 0])

        initial_values = Values()

        torques = Values()
        gtd.InsertTorque(torques, 0, 1.0)

        simulator = gtd.Simulator(robot, initial_values, gravity, planar_axis)

        num_steps = 1 + 1
        dt = 1
        torques_seq = [torques for _ in range(num_steps)]
        results = simulator.simulate(torques_seq, dt)

        acceleration = 0.0625
        expected_qAccel = acceleration
        expected_qVel = acceleration * dt
        expected_qAngle = acceleration * 0.5 * dt * dt
        self.assertEqual(expected_qAngle, gtd.JointAngle(results, 0, 0))
        self.assertEqual(expected_qVel, gtd.JointVel(results, 0, 0))
        self.assertEqual(expected_qAccel, gtd.JointAccel(results, 0, 0))


if __name__ == "__main__":
    unittest.main()
