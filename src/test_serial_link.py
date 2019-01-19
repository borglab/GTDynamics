#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np
from dh_parameters import PUMA_calibration, RR_calibration
from gtsam import Point3, Pose3, Rot3
from serial_link import SerialLink
from utils import GtsamTestCase, vector


class TestSerialLink(GtsamTestCase):
    """Unit tests for different manipulators class."""

    def test_fkine(self):
        """Try forward kinematics."""
        qz = vector(0, 0, 0, 0, 0, 0)
        robot = SerialLink(PUMA_calibration)
        T = robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3.Rx(math.radians(90)), Point3(0, 0, 0)))

    # def test_RR_forward_dynamics(self):
    #     """Try a simple RR robot."""
    #     expected_joint_accels = vector(0, 0)  # frome MATLAB
    #     # Call a function with appropriate arguments to co compute them
    #     joint_angles = [0, 0]
    #     joint_velocities = [1, 1]
    #     joint_torques = [0, 0]
    #     manipulator = SerialLink(RR_calibration)
    #     factor_graph = manipulator.forward_factor_graph(
    #         joint_angles, joint_velocities, joint_torques)
    #     actual_joint_accels = manipulator.factor_graph_optimization(
    #         factor_graph)
    #     np.testing.assert_array_almost_equal(
    #         actual_joint_accels, expected_joint_accels)

    # def test_PUMA_forward_dynamics(self):
    #     """Try a PUMA robot."""
    #     expected_joint_accels = vector(
    #         0.174533, 0.349066, 0.523599, 0.698132, 0.872665, 1.047198)  # from MATLAB
    #     # Call a function with appropriate arguments to co compute them
    #     joint_angles = [0, 0, 0, 0, 0, 0]
    #     joint_velocities = [-5, -10, -15, -20, -25, -30]
    #     joint_torques = [0.626950752326773, -34.8262338725151, 1.02920598714973,
    #                      -0.0122426673731905, 0.166693973271978, 7.20736555357164e-05]
    #     manipulator = SerialLink(PUMA_calibration)
    #     factor_graph = manipulator.forward_factor_graph(
    #         joint_angles, joint_velocities, joint_torques)
    #     actual_joint_accels = manipulator.factor_graph_optimization(
    #         factor_graph)
    #     np.testing.assert_array_almost_equal(
    #         actual_joint_accels, expected_joint_accels)


if __name__ == "__main__":
    unittest.main()
