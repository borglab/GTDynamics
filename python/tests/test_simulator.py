"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_simulator.py
 * @brief test Simulator class.
 * @author Frank Dellaert and Yetong Zhang
"""

# pylint: disable=no-name-in-module, import-error, no-member
import unittest

import numpy as np
from gtsam import Point3, Pose3, Rot3, Values
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd


class TestLink(GtsamTestCase):
    """Test Simulator Class"""

    def test_simulator(self):
        """Test simulating two steps on a simple one-link robot."""
    # load example robot
    SDF_PATH = "../../sdfs/"
    robot = gtd.CreateRobotFromFile(SDF_PATH + "/test/simple.sdf",
                                    "simple")

    print(robot)
    initial_values = Values()
    # InsertJointAngle(initial_values, 0, 0.0)
    # InsertJointVel(initial_values, 0, 0.0)
    # InsertTorque(torques, 0, 1.0)

    # Simulator simulator(robot, initial_values, gravity, planar_axis)

    # num_steps = 1 + 1
    # dt = 1
    # vector < gtsam: : Values > torques_seq(num_steps, torques)
    # auto results = simulator.simulate(torques_seq, dt)

    # print(results)

    # acceleration = 0.0625
    # expected_qAccel = acceleration
    # expected_qVel = acceleration * dt
    # expected_qAngle = acceleration * 0.5 * dt * dt
    # self.gtsamAssertEquals(expected_qAngle, JointAngle(results, 0))
    # self.gtsamAssertEquals(expected_qVel, JointVel(results, 0))
    # self.gtsamAssertEquals(expected_qAccel, JointAccel(results, 0))


if __name__ == "__main__":
    unittest.main()
