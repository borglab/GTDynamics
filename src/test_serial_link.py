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
from dh_parameters import RR_calibration, PUMA_calibration
from gtsam import Point3, Pose3, Rot3
from serial_link import SerialLink
from utils import GtsamTestCase, vector


class TestRR(GtsamTestCase):
    """Unit tests for DH RR."""

    def setUp(self):
        """Create RR robot."""
        self.robot = SerialLink(
            RR_calibration,
            tool=Pose3(Rot3.Ry(math.radians(90)), Point3(0, 0, 0))
        )

    def test_link_frames(self):
        """Try link_frames."""
        configuration = self.robot.link_frames()
        self.assertIsInstance(configuration, list)
        self.assertEquals(len(configuration), 2)
        self.gtsamAssertEquals(
            configuration[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(
            configuration[1], Pose3(Rot3(), Point3(4, 0, 0)))

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        expected_joint_accels = vector(0, 0)  # frome MATLAB
        # Call a function with appropriate arguments to co compute them
        joint_angles = [0, 0]
        joint_velocities = [1, 1]
        joint_torques = [0, 0]
        factor_graph = self.robot.forward_factor_graph(
            joint_angles, joint_velocities, joint_torques)
        actual_joint_accels = self.robot.factor_graph_optimization(
            factor_graph)
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)


class TestPuma(GtsamTestCase):
    """Unit tests for DH Puma."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(PUMA_calibration)

    def test_fkine(self):
        """Try forward kinematics, example from Corke 2017 page 203."""
        qz = vector(0, 0, 0, 0, 0, 0)
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3(), Point3(0.4521, -0.15, 0.4318)), tol=1e-4)

    def test_link_frames(self):
        """Try link_frames."""
        configuration = self.robot.link_frames()
        self.assertIsInstance(configuration, list)
        self.assertEquals(len(configuration), 6)
        self.gtsamAssertEquals(
            configuration[0], Pose3(Rot3.Rx(math.radians(90)), Point3(0, 0, 0)))

    # def test_PUMA_forward_dynamics(self):
    #     """Try a PUMA robot."""
    #     expected_joint_accels = vector(
    #         0.174533, 0.349066, 0.523599, 0.698132, 0.872665, 1.047198)  # from MATLAB
    #     # Call a function with appropriate arguments to co compute them
    #     joint_angles = [0, 0, 0, 0, 0, 0]
    #     joint_velocities = [-5, -10, -15, -20, -25, -30]
    #     joint_torques = [0.626950752326773, -34.8262338725151, 1.02920598714973,
    #                      -0.0122426673731905, 0.166693973271978, 7.20736555357164e-05]
    #     factor_graph = self.robot.forward_factor_graph(
    #         joint_angles, joint_velocities, joint_torques)
    #     actual_joint_accels = self.robot.factor_graph_optimization(
    #         factor_graph)
    #     np.testing.assert_array_almost_equal(
    #         actual_joint_accels, expected_joint_accels)


class TestPumaPlus(GtsamTestCase):
    """Unit tests for Puma, with base and tool transforms."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(
            PUMA_calibration,
            base=Pose3(Rot3.Rx(math.pi), Point3(0, 0, 3)),
            tool=Pose3(Rot3(), Point3(0, 0, 0.2)))

    def test_fkine(self):
        """Try forward kinematics, second example from Corke 2017 page 204."""
        qz = vector(0, 0, 0, 0, 0, 0)
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3.Rx(math.pi), Point3(0.4521, 0.15, 2.3682)), tol=1e-4)


if __name__ == "__main__":
    unittest.main()
