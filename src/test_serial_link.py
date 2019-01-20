#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest
import utils

import numpy as np
from dh_parameters import RR_calibration, PUMA_calibration
from gtsam import Point3, Pose3, Rot3
from serial_link import SerialLink

HALF_PI = math.pi/2
R90 = Rot3.Rz(HALF_PI)
R180 = Rot3.Rz(math.pi)
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


class TestRR(utils.GtsamTestCase):
    """Unit tests for DH RR."""

    QZ = utils.vector(0.00, 0.00)  # at rest
    Q1 = utils.vector(HALF_PI, 0)  # vertical
    Q2 = utils.vector(0, math.pi)  # doubled back

    # The joint screw axis, in the COM frame, is the same for both joints
    AXIS = utils.unit_twist([0, 0, 1], [-1, 0, 0])

    def setUp(self):
        """Create RR robot."""
        self.robot = SerialLink(
            RR_calibration,
            tool=Pose3(Rot3.Ry(HALF_PI), Point3(0, 0, 0))
        )

    @staticmethod
    def transform(theta):
        """Expected link transform depending on angle."""
        return Pose3(Rot3.Rz(theta), Point3(0, 0, 0)).compose(Pose3(Rot3(), Point3(2, 0, 0)))

    def test_link_transforms(self):
        """Test link_transforms."""
        # Check zero joint angles
        frames = self.robot.link_transforms()
        self.assertIsInstance(frames, list)
        self.assertEquals(len(frames), 2)
        self.gtsamAssertEquals(frames[0], self.transform(0))
        self.gtsamAssertEquals(frames[1], self.transform(0))

        # Check vertical configuration
        frames = self.robot.link_transforms(self.Q1)
        self.gtsamAssertEquals(frames[0], self.transform(HALF_PI))
        self.gtsamAssertEquals(frames[1], self.transform(0))

        # Check doubled back configuration
        frames = self.robot.link_transforms(self.Q2)
        self.gtsamAssertEquals(frames[0], self.transform(0))
        self.gtsamAssertEquals(frames[1], self.transform(math.pi))

    def test_link_frames(self):
        """Test link_frames."""
        # Check zero joint angles
        frames = self.robot.link_frames()
        self.assertIsInstance(frames, list)
        self.assertEquals(len(frames), 2)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(Rot3(), Point3(4, 0, 0)))

        # Check vertical configuration
        frames = self.robot.link_frames(self.Q1)
        self.gtsamAssertEquals(frames[0], Pose3(R90, Point3(0, 2, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(R90, Point3(0, 4, 0)))

        # Check doubled back configuration
        frames = self.robot.link_frames(self.Q2)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(R180, Point3(0, 0, 0)))

    def test_com_frames(self):
        """Test com_frames."""
        # Check zero joint angles
        Ts = self.robot.com_frames()
        self.assertIsInstance(Ts, list)
        self.assertEquals(len(Ts), 2)
        self.gtsamAssertEquals(Ts[0], Pose3(Rot3(), Point3(1, 0, 0)))
        self.gtsamAssertEquals(Ts[1], Pose3(Rot3(), Point3(3, 0, 0)))

        # Check vertical configuration
        Ts = self.robot.com_frames(self.Q1)
        self.gtsamAssertEquals(Ts[0], Pose3(R90, Point3(0, 1, 0)))
        self.gtsamAssertEquals(Ts[1], Pose3(R90, Point3(0, 3, 0)))

        # Check doubled back configuration
        Ts = self.robot.com_frames(self.Q2)
        self.gtsamAssertEquals(Ts[0], Pose3(Rot3(), Point3(1, 0, 0)))
        self.gtsamAssertEquals(Ts[1], Pose3(R180, Point3(1, 0, 0)))

    def test_screw_axes(self):
        """Test screw_axes."""
        screw_axes = self.robot.screw_axes()
        self.assertIsInstance(screw_axes, list)
        self.assertEquals(len(screw_axes), 2)
        np.testing.assert_array_almost_equal(screw_axes[0], self.AXIS)
        np.testing.assert_array_almost_equal(screw_axes[1], self.AXIS)

    def test_jTi_list(self):
        """Test jTi_list."""
        # Check zero joint angles
        jTi_list = self.robot.jTi_list(self.QZ)
        self.assertIsInstance(jTi_list, list)
        self.assertEquals(len(jTi_list), 1)
        self.gtsamAssertEquals(jTi_list[0], Pose3(Rot3(), Point3(-2, 0, 0)))

        # Check vertical configuration
        frames = self.robot.com_frames(self.Q1)
        expected = frames[1].between(frames[0])
        jTi_list = self.robot.jTi_list(self.Q1)
        self.gtsamAssertEquals(jTi_list[0], expected)

        # Check doubled back configuration
        frames = self.robot.com_frames(self.Q2)
        expected = frames[1].between(frames[0])
        jTi_list = self.robot.jTi_list(self.Q2)
        self.gtsamAssertEquals(jTi_list[0], expected)
        
    def test_twists(self):
        """Test twists."""
        # Check zero joint angles
        Ts = self.robot.com_frames()

        # Check zero joint velocities
        twists = self.robot.twists(Ts, utils.vector(0, 0))
        self.assertIsInstance(twists, list)
        self.assertEquals(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], ZERO6)
        np.testing.assert_array_almost_equal(twists[1], ZERO6)

        # Check rotating joint 1
        twists = self.robot.twists(Ts, utils.vector(3, 0))
        np.testing.assert_array_almost_equal(twists[0], 3 * self.AXIS)
        # second joint is also rotating around point (0,0,0), which is (-3,0,0) in COM frame 2
        expected = utils.unit_twist([0, 0, 3], [-3, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check rotating joint 2
        twists = self.robot.twists(Ts, utils.vector(0, 2))
        np.testing.assert_array_almost_equal(twists[0], ZERO6)
        # second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame 2
        expected = utils.unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check both rotating, should be linear combination
        twists = self.robot.twists(Ts, utils.vector(3, 2))
        np.testing.assert_array_almost_equal(twists[0], 3 * self.AXIS)
        expected = utils.unit_twist(
            [0, 0, 3], [-3, 0, 0]) + utils.unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check doubled back configuration
        Ts = self.robot.com_frames(self.Q2)

        # Check zero joint velocities
        twists = self.robot.twists(Ts, utils.vector(3, 2))
        self.assertIsInstance(twists, list)
        self.assertEquals(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], 3*self.AXIS)
        expected = utils.unit_twist(
            [0, 0, 3], [1, 0, 0]) + utils.unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

    # def test_RR_forward_dynamics(self):
    #     """Test a simple RR robot."""
    #     expected_joint_accels = utils.vector(0, 0)  # from MATLAB
    #     # Call a function with appropriate arguments to co compute them
    #     joint_angles = [0, 0]
    #     joint_velocities = [1, 1]
    #     joint_torques = [0, 0]
    #     factor_graph = self.robot.forward_factor_graph(
    #         joint_angles, joint_velocities, joint_torques)
    #     actual_joint_accels = self.robot.factor_graph_optimization(
    #         factor_graph)
    #     np.testing.assert_array_almost_equal(
    #         actual_joint_accels, expected_joint_accels)


class TestPuma(utils.GtsamTestCase):
    """Unit tests for DH Puma."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(PUMA_calibration)

    def test_fkine(self):
        """Test forward kinematics, example from Corke 2017 page 203."""
        qz = ZERO6
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3(), Point3(0.4521, -0.15, 0.4318)), tol=1e-4)

    def test_link_frames(self):
        """Test link_frames."""
        frames = self.robot.link_frames()
        self.assertIsInstance(frames, list)
        self.assertEquals(len(frames), 6)
        self.gtsamAssertEquals(
            frames[0], Pose3(Rot3.Rx(HALF_PI), Point3(0, 0, 0)))

    # def test_PUMA_forward_dynamics(self):
    #     """Test a PUMA robot."""
    #     expected_joint_accels = utils.vector(
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


class TestPumaPlus(utils.GtsamTestCase):
    """Unit tests for Puma, with base and tool transforms."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(
            PUMA_calibration,
            base=Pose3(Rot3.Rx(math.pi), Point3(0, 0, 3)),
            tool=Pose3(Rot3(), Point3(0, 0, 0.2)))

    def test_fkine(self):
        """Test forward kinematics, second example from Corke 2017 page 204."""
        qz = ZERO6
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3.Rx(math.pi), Point3(0.4521, 0.15, 2.3682)), tol=1e-4)


if __name__ == "__main__":
    unittest.main()
