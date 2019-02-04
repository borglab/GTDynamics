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
from link_parameters import PUMA_calibration_dh, RR_calibration_dh, RR_calibration_urdf
from gtsam import Point3, Pose3, Rot3
from serial_link import SerialLink
from utils import GtsamTestCase, unit_twist, vector

HALF_PI = math.pi/2
R90 = Rot3.Rz(HALF_PI)
R180 = Rot3.Rz(math.pi)
ZERO6 = vector(0, 0, 0, 0, 0, 0)


class BaseTestCase(GtsamTestCase):
    """Unit tests for single link, use same link properties as RR."""

    def check_forward_dynamics(self, joint_angles=None, joint_velocities=None,
                               joint_torques=None, expected_joint_accels=None,
                               base_twist_accel=ZERO6, external_wrench=ZERO6, debug=False):
        """Test forward dynamics."""
        N = self.robot.num_links
        zeros = np.zeros((N,), np.float)

        if joint_angles is None:
            joint_angles = zeros
        if joint_velocities is None:
            joint_velocities = zeros
        if joint_torques is None:
            joint_torques = zeros

        factor_graph = self.robot.forward_factor_graph(
            joint_angles, joint_velocities, joint_torques,
            base_twist_accel=base_twist_accel, external_wrench=external_wrench)
        self.assertEqual(factor_graph.size(), 1 + N*3 + 1)

        result = self.robot.factor_graph_optimization(factor_graph)
        if debug:
            print(result)

        if expected_joint_accels is None:
            expected_joint_accels = zeros
        np.testing.assert_array_almost_equal(
            self.robot.extract_joint_accelerations(result), expected_joint_accels)

class TestURDF_RR(BaseTestCase):
    """Unit tests for DH RR."""

    QZ = vector(0.00, 0.00)  # at rest
    Q1 = vector(HALF_PI, 0)  # vertical
    Q2 = vector(0, math.pi)  # doubled back

    # The joint screw axis, in the COM frame, is the same for both joints
    AXIS = unit_twist([0, 0, 1], [-1, 0, 0])

    def setUp(self):
        """Create RR robot."""
        self.robot = SerialLink(
            RR_calibration_urdf,
            tool=Pose3(Rot3(), Point3(2, 0, 0))
        )
    
    def test_link_transforms(self):
        """Test link_transforms."""
        # Check zero joint angles
        frames = self.robot.link_transforms()
        self.assertIsInstance(frames, list)
        self.assertEqual(len(frames), 2)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(Rot3(), Point3(2, 0, 0)))

        # Check vertical configuration
        frames = self.robot.link_transforms(self.Q1)
        self.gtsamAssertEquals(frames[0], Pose3(R90, Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(Rot3(), Point3(2, 0, 0)))

        # Check doubled back configuration
        frames = self.robot.link_transforms(self.Q2)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(R180, Point3(2, 0, 0)))

    def test_link_frames(self):
        """Test link_frames."""
        # Check zero joint angles
        frames = self.robot.link_frames()
        self.assertIsInstance(frames, list)
        self.assertEqual(len(frames), 2)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(Rot3(), Point3(4, 0, 0)))

        # Check vertical configuration
        frames = self.robot.link_frames(self.Q1)
        self.gtsamAssertEquals(frames[0], Pose3(R90, Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(R90, Point3(2, 2, 0)))

        # Check doubled back configuration
        frames = self.robot.link_frames(self.Q2)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(frames[1], Pose3(R180, Point3(4, 0, 0)))

    def test_com_frames(self):
        """Test com_frames."""
        # Check zero joint angles
        Ts = self.robot.com_frames()
        self.assertIsInstance(Ts, list)
        self.assertEqual(len(Ts), 2)
        self.gtsamAssertEquals(Ts[0], Pose3(Rot3(), Point3(3, 0, 0)))
        self.gtsamAssertEquals(Ts[1], Pose3(Rot3(), Point3(5, 0, 0)))

        # Check vertical configuration
        Ts = self.robot.com_frames(self.Q1)
        self.gtsamAssertEquals(Ts[0], Pose3(R90, Point3(2, 1, 0)))
        self.gtsamAssertEquals(Ts[1], Pose3(R90, Point3(2, 3, 0)))

        # Check doubled back configuration
        Ts = self.robot.com_frames(self.Q2)
        self.gtsamAssertEquals(Ts[0], Pose3(Rot3(), Point3(3, 0, 0)))
        self.gtsamAssertEquals(Ts[1], Pose3(R180, Point3(3, 0, 0)))

    def test_screw_axes(self):
        """Test screw_axes."""
        screw_axes = self.robot.screw_axes()
        self.assertIsInstance(screw_axes, list)
        self.assertEqual(len(screw_axes), 2)
        np.testing.assert_array_almost_equal(screw_axes[0], self.AXIS)
        np.testing.assert_array_almost_equal(screw_axes[1], self.AXIS)

    def test_jTi_list(self):
        """Test jTi_list."""
        # Check zero joint angles
        jTi_list = self.robot.jTi_list(self.QZ)
        self.assertIsInstance(jTi_list, list)
        self.assertEqual(len(jTi_list), 3)
        self.gtsamAssertEquals(jTi_list[0], Pose3(Rot3(), Point3(-3, 0, 0)))
        self.gtsamAssertEquals(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0)))
        self.gtsamAssertEquals(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0)))

        # Check vertical configuration
        jTi_list = self.robot.jTi_list(self.Q1)
        self.gtsamAssertEquals(jTi_list[0], Pose3(
            R90.inverse(), Point3(-1, 2, 0)))
        self.gtsamAssertEquals(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0)))
        self.gtsamAssertEquals(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0)))

        # Check doubled back configuration
        jTi_list = self.robot.jTi_list(self.Q2)
        self.gtsamAssertEquals(jTi_list[0], Pose3(Rot3(), Point3(-3, 0, 0)))
        self.gtsamAssertEquals(jTi_list[1], Pose3(R180, Point3(0, 0, 0)))
        self.gtsamAssertEquals(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0)))

    def test_twists(self):
        """Test twists."""
        # Check zero joint angles
        Ts = self.robot.com_frames()

        # Check zero joint velocities
        twists = self.robot.twists(Ts, vector(0, 0))
        self.assertIsInstance(twists, list)
        self.assertEqual(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], ZERO6)
        np.testing.assert_array_almost_equal(twists[1], ZERO6)

        # Check rotating joint 1
        twists = self.robot.twists(Ts, vector(3, 0))
        np.testing.assert_array_almost_equal(twists[0], 3 * self.AXIS)
        # second joint is also rotating around point (0,0,0), which is (-3,0,0) in COM frame 2
        expected = unit_twist([0, 0, 3], [-3, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check rotating joint 2
        twists = self.robot.twists(Ts, vector(0, 2))
        np.testing.assert_array_almost_equal(twists[0], ZERO6)
        # second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame 2
        expected = unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check both rotating, should be linear combination
        twists = self.robot.twists(Ts, vector(3, 2))
        np.testing.assert_array_almost_equal(twists[0], 3 * self.AXIS)
        expected = unit_twist(
            [0, 0, 3], [-3, 0, 0]) + unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check doubled back configuration
        Ts = self.robot.com_frames(self.Q2)

        # Check zero joint velocities
        twists = self.robot.twists(Ts, vector(3, 2))
        self.assertIsInstance(twists, list)
        self.assertEqual(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], 3*self.AXIS)
        expected = unit_twist(
            [0, 0, 3], [1, 0, 0]) + unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

    def test_forward_dynamics_stationary(self):
        """Test stationary case."""
        self.check_forward_dynamics()

    def test_forward_external_wrench(self):
        """Test case when an external wrench is applied."""
        self.check_forward_dynamics(
            external_wrench=vector(0, 0, 0, 0, -2.5, 0),
            expected_joint_accels=vector(5, -20)
        )

    def test_forward_dynamics_gravity(self):
        """Test gravity compensation case: assume Y-axis is up."""
        self.check_forward_dynamics(
            base_twist_accel=vector(0, 0, 0, 0, 9.8, 0),
            expected_joint_accels=vector(-9.8, 19.6)
        )


class TestDH_R(BaseTestCase):
    """Unit tests for single link, use same link properties as RR."""

    def setUp(self):
        """Create simple single-link robot."""
        self.robot = SerialLink(RR_calibration_dh[:1])

    def test_forward_dynamics_stationary(self):
        """Test stationary case."""
        self.check_forward_dynamics()

    def test_forward_external_wrench(self):
        """Test case when an external downward (-Y) force is applied."""
        # zero acceleration expected as torque cancels the external wrench
        self.check_forward_dynamics(joint_torques=vector(5),
                                    external_wrench=vector(
                                        0, 0, 0, 0, -2.5, 0),
                                    expected_joint_accels=vector(0)
                                    )

    def test_forward_dynamics_gravity(self):
        """Test gravity compensation case: assume Y-axis is up."""
        # gravity = -9.8, we force based to have negative gravity acceleration
        self.check_forward_dynamics(
            base_twist_accel=vector(0, 0, 0, 0, 9.8, 0),
            expected_joint_accels=vector(-9.8))


class TestDH_RR(BaseTestCase):
    """Unit tests for DH RR."""

    QZ = vector(0.00, 0.00)  # at rest
    Q1 = vector(HALF_PI, 0)  # vertical
    Q2 = vector(0, math.pi)  # doubled back

    # The joint screw axis, in the COM frame, is the same for both joints
    AXIS = unit_twist([0, 0, 1], [-1, 0, 0])

    def setUp(self):
        """Create RR robot."""
        self.robot = SerialLink(
            RR_calibration_dh,
            tool=Pose3()
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
        self.assertEqual(len(frames), 2)
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
        self.assertEqual(len(frames), 2)
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
        self.assertEqual(len(Ts), 2)
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
        self.assertEqual(len(screw_axes), 2)
        np.testing.assert_array_almost_equal(screw_axes[0], self.AXIS)
        np.testing.assert_array_almost_equal(screw_axes[1], self.AXIS)

    def test_jTi_list(self):
        """Test jTi_list."""
        # Check zero joint angles
        jTi_list = self.robot.jTi_list(self.QZ)
        self.assertIsInstance(jTi_list, list)
        self.assertEqual(len(jTi_list), 3)
        self.gtsamAssertEquals(jTi_list[0], Pose3(Rot3(), Point3(-1, 0, 0)))
        self.gtsamAssertEquals(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0)))
        self.gtsamAssertEquals(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0)))

        # Check vertical configuration
        jTi_list = self.robot.jTi_list(self.Q1)
        self.gtsamAssertEquals(jTi_list[0], Pose3(
            R90.inverse(), Point3(-1, 0, 0)))
        self.gtsamAssertEquals(jTi_list[1], Pose3(Rot3(), Point3(-2, 0, 0)))
        self.gtsamAssertEquals(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0)))

        # Check doubled back configuration
        jTi_list = self.robot.jTi_list(self.Q2)
        self.gtsamAssertEquals(jTi_list[0], Pose3(Rot3(), Point3(-1, 0, 0)))
        self.gtsamAssertEquals(jTi_list[1], Pose3(R180, Point3(0, 0, 0)))
        self.gtsamAssertEquals(jTi_list[2], Pose3(Rot3(), Point3(-1, 0, 0)))

    def test_twists(self):
        """Test twists."""
        # Check zero joint angles
        Ts = self.robot.com_frames()

        # Check zero joint velocities
        twists = self.robot.twists(Ts, vector(0, 0))
        self.assertIsInstance(twists, list)
        self.assertEqual(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], ZERO6)
        np.testing.assert_array_almost_equal(twists[1], ZERO6)

        # Check rotating joint 1
        twists = self.robot.twists(Ts, vector(3, 0))
        np.testing.assert_array_almost_equal(twists[0], 3 * self.AXIS)
        # second joint is also rotating around point (0,0,0), which is (-3,0,0) in COM frame 2
        expected = unit_twist([0, 0, 3], [-3, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check rotating joint 2
        twists = self.robot.twists(Ts, vector(0, 2))
        np.testing.assert_array_almost_equal(twists[0], ZERO6)
        # second joint rotating around point (2,0,0), which is (-2,0,0) in COM frame 2
        expected = unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check both rotating, should be linear combination
        twists = self.robot.twists(Ts, vector(3, 2))
        np.testing.assert_array_almost_equal(twists[0], 3 * self.AXIS)
        expected = unit_twist(
            [0, 0, 3], [-3, 0, 0]) + unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check doubled back configuration
        Ts = self.robot.com_frames(self.Q2)

        # Check zero joint velocities
        twists = self.robot.twists(Ts, vector(3, 2))
        self.assertIsInstance(twists, list)
        self.assertEqual(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], 3*self.AXIS)
        expected = unit_twist(
            [0, 0, 3], [1, 0, 0]) + unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

    def test_forward_dynamics_stationary(self):
        """Test stationary case."""
        self.check_forward_dynamics()

    def test_forward_external_wrench(self):
        """Test case when an external wrench is applied."""
        self.check_forward_dynamics(
            external_wrench=vector(0, 0, 0, 0, -2.5, 0),
            expected_joint_accels=vector(5, -20)
        )

    def test_forward_dynamics_gravity(self):
        """Test gravity compensation case: assume Y-axis is up."""
        self.check_forward_dynamics(
            base_twist_accel=vector(0, 0, 0, 0, 9.8, 0),
            expected_joint_accels=vector(-9.8, 19.6)
        )


class TestDH_Puma(BaseTestCase):
    """Unit tests for DH Puma."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(PUMA_calibration_dh,
            base=Pose3(Rot3(), Point3()),
            tool=Pose3(Rot3(), Point3()))

    def test_fkine(self):
        """Test forward kinematics, example from Corke 2017 page 203."""
        qz = ZERO6
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3(), Point3(0.58185, -0.4521, 2.76831e-17)), tol=1e-4)

    def test_link_frames(self):
        """Test link_frames."""
        frames = self.robot.link_frames()
        self.assertIsInstance(frames, list)
        self.assertEqual(len(frames), 6)
        self.gtsamAssertEquals(
            frames[0], Pose3(Rot3.Rx(HALF_PI), Point3(0, 0, 0)))

    # @unittest.skip("Wrong result after change to classic DH")
    def test_PUMA_forward_dynamics_stationary(self):
        """Test forward dynamics, stationary case."""
        self.check_forward_dynamics()

    # @unittest.skip("MATLAB example needs to be re-done with regular DH.")
    def test_PUMA_forward_dynamics_matlab(self):
        """Test forward dynamics, Mandy's MATLAB example."""
        self.check_forward_dynamics(
            joint_velocities=np.radians(vector(-5, -10, -15, -20, -25, -30)),
            joint_torques=vector(2.14242560e+00, -4.72874900e+01, 1.37677604e+01, 2.15162000e-01, 1.45261716e-03, 7.67944871e-05),
            expected_joint_accels=vector(
                0.174533, 0.349066, 0.523599, 0.698132, 0.872665, 1.047198),  # from MATLAB
            base_twist_accel=vector(0, 0, 0, 0, 0, 9.8)
        )


class TestDH_PumaPlus(BaseTestCase):
    """Unit tests for Puma, with base and tool transforms."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(
            PUMA_calibration_dh,
            base=Pose3(Rot3.Rx(math.pi), Point3(0, 0, 3)),
            tool=Pose3(Rot3(), Point3(0, 0, 0.2)))

    def test_fkine(self):
        """Test forward kinematics, second example from Corke 2017 page 204."""
        qz = ZERO6
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3.Rx(math.pi), Point3(0.58185, 0.4521, 2.8)), tol=1e-4)


if __name__ == "__main__":
    unittest.main()
