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
from utils import GtsamTestCase, unit_twist, vector

HALF_PI = math.pi/2
R90 = Rot3.Rz(HALF_PI)
R180 = Rot3.Rz(math.pi)
ZERO6 = vector(0, 0, 0, 0, 0, 0)


class BaseTestCase(GtsamTestCase):
    """Unit tests for single link, use same link properties as RR."""

    def check_forward_dynamics(self, joint_angles=None, joint_velocities=None,
                               torques=None, accelerations=None,
                               gravity=None, external_wrench=ZERO6, debug=False):
        """Test forward dynamics."""
        N = self.robot.num_links
        zeros = np.zeros((N,), np.float)

        if joint_angles is None:
            joint_angles = zeros
        if joint_velocities is None:
            joint_velocities = zeros
        if torques is None:
            torques = zeros

        factor_graph = self.robot.forward_factor_graph(
            joint_angles, joint_velocities, torques,
            gravity=gravity, external_wrench=external_wrench)
        if debug:
            print(factor_graph)
        self.assertEqual(factor_graph.size(), N*3 + 2)

        result = self.robot.factor_graph_optimization(factor_graph)
        if debug:
            print(result)

        if accelerations is None:
            accelerations = zeros
        np.testing.assert_array_almost_equal(
            self.robot.extract_joint_accelerations(result), accelerations)

    def check_inverse_dynamics(self, joint_angles=None, joint_velocities=None,
                               accelerations=None, torques=None,
                               gravity=None, external_wrench=ZERO6, debug=False):
        """Test inverse dynamics."""
        N = self.robot.num_links
        zeros = np.zeros((N,), np.float)

        if joint_angles is None:
            joint_angles = zeros
        if joint_velocities is None:
            joint_velocities = zeros
        if accelerations is None:
            accelerations = zeros

        factor_graph = self.robot.inverse_factor_graph(
            joint_angles, joint_velocities, accelerations,
            gravity=gravity, external_wrench=external_wrench)
        if debug:
            print(factor_graph)
        self.assertEqual(factor_graph.size(), N*3 + 2)

        result = self.robot.factor_graph_optimization(factor_graph)
        if debug:
            print(result)

        if torques is None:
            torques = zeros
        np.testing.assert_array_almost_equal(
            self.robot.extract_torques(result), torques)


class TestR(BaseTestCase):
    """Unit tests for single link, use same link properties as RR."""

    def setUp(self):
        """Create simple single-link robot."""
        self.robot = SerialLink(RR_calibration[:1])

    def test_link_frames(self):
        """Test link_frames."""
        # Check zero joint angles
        frames = self.robot.link_frames()
        self.assertIsInstance(frames, list)
        self.assertEqual(len(frames), 1)
        self.gtsamAssertEquals(frames[0], Pose3(Rot3(), Point3(2, 0, 0)))

        # Check vertical configuration
        frames = self.robot.link_frames(vector(math.radians(90)))
        self.gtsamAssertEquals(frames[0], Pose3(R90, Point3(0, 2, 0)))

    def test_stationary(self):
        """Test stationary case."""
        self.check_forward_dynamics()
        self.check_inverse_dynamics()

    def test_external_wrench(self):
        """Test case when an external downward (-Y) force is applied."""
        # zero acceleration expected as torque cancels the external wrench
        scenario = {"torques": vector(5),
                    "accelerations": vector(0),
                    "external_wrench": vector(0, 0, 0, 0, -2.5, 0)}
        self.check_forward_dynamics(**scenario)
        self.check_inverse_dynamics(**scenario)

    def test_gravity_compensation(self):
        """Test gravity compensation for rest: assume Y-axis is up."""
        # Acceleration due to gravity = -9.8, in negative Y direction
        scenario = {"torques": vector(0),
                    "accelerations": vector(-9.8),
                    "gravity": vector(0, -9.8, 0)}
        self.check_forward_dynamics(**scenario)
        self.check_inverse_dynamics(**scenario)

    def test_gravity_compensation_vertical(self):
        """Test gravity compensation for vertical case: assume Y-axis is up."""
        # Acceleration due to gravity = -9.8, in negative Y direction
        scenario = {"joint_angles": vector(math.radians(90)),
                    "torques": vector(0),
                    "accelerations": vector(0),
                    "gravity": vector(0, -9.8, 0)}
        self.check_forward_dynamics(**scenario)
        self.check_inverse_dynamics(**scenario)


class TestRR(BaseTestCase):
    """Unit tests for DH RR."""

    QZ = vector(0.00, 0.00)  # at rest
    Q1 = vector(HALF_PI, 0)  # vertical
    Q2 = vector(0, math.pi)  # doubled back

    # The joint screw axis, in the COM frame, is the same for both joints
    AXIS = unit_twist([0, 0, 1], [-1, 0, 0])

    def setUp(self):
        """Create RR robot."""
        self.robot = SerialLink(
            RR_calibration,
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

        # Check both rotating, should be linear combination
        twists = self.robot.twists(Ts, vector(3, 2))
        self.assertIsInstance(twists, list)
        self.assertEqual(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], 3*self.AXIS)
        expected = unit_twist(
            [0, 0, 3], [1, 0, 0]) + unit_twist([0, 0, 2], [-1, 0, 0])
        np.testing.assert_array_almost_equal(twists[1], expected)

        # Check that we get exactly the same answer when using factor graph
        twists = self.robot.twists_gtsam(self.Q2, vector(3, 2))
        self.assertIsInstance(twists, list)
        self.assertEqual(len(twists), 2)
        np.testing.assert_array_almost_equal(twists[0], 3*self.AXIS)
        np.testing.assert_array_almost_equal(twists[1], expected)

    def test_stationary(self):
        """Test stationary case."""
        self.check_forward_dynamics()
        self.check_inverse_dynamics()

    def test_external_wrench(self):
        """Test case when an external downward (-Y) force is applied."""
        # zero acceleration expected as torque cancels the external wrench
        scenario = {"torques": vector(0, 0),
                    "accelerations": vector(5, -20),
                    "external_wrench": vector(0, 0, 0, 0, -2.5, 0)}
        self.check_forward_dynamics(**scenario)
        self.check_inverse_dynamics(**scenario)

    def test_forward_dynamics_gravity(self):
        """Test gravity compensation case: assume Y-axis is up."""
        # Acceleration due to gravity = -9.8, in negative Y direction
        scenario = {"torques": vector(0, 0),
                    "accelerations": vector(-9.8, 19.6),
                    "gravity": vector(0, -9.8, 0)}
        self.check_forward_dynamics(**scenario)
        self.check_inverse_dynamics(**scenario)


class TestPuma(BaseTestCase):
    """Unit tests for DH Puma."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(PUMA_calibration)

    @unittest.skip("Wrong result after change to classic DH")
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
        self.assertEqual(len(frames), 6)
        self.gtsamAssertEquals(
            frames[0], Pose3(Rot3.Rx(HALF_PI), Point3(0, 0, 0)))

    @unittest.skip("Wrong result after change to classic DH")
    def test_stationary(self):
        """Test stationary case."""
        self.check_forward_dynamics(debug=True)
        self.check_inverse_dynamics()

    @unittest.skip("MATLAB example needs to be re-done with regular DH.")
    def test_PUMA_forward_dynamics_matlab(self):
        """Test forward dynamics, Mandy's MATLAB example."""
        self.check_forward_dynamics(
            joint_velocities=np.radians(vector(-5, -10, -15, -20, -25, -30)),
            torques=vector(2.34853527092267, -47.4289308101242, 3.89597516275938, -
                           0.582995766943385, -0.0205988067713663, 7.68216420389942e-05,),
            accelerations=vector(
                0.174533, 0.349066, 0.523599, 0.698132, 0.872665, 1.047198)  # from MATLAB
        )


class TestPumaPlus(BaseTestCase):
    """Unit tests for Puma, with base and tool transforms."""

    def setUp(self):
        """Create Puma robot."""
        self.robot = SerialLink(
            PUMA_calibration,
            base=Pose3(Rot3.Rx(math.pi), Point3(0, 0, 3)),
            tool=Pose3(Rot3(), Point3(0, 0, 0.2)))

    @unittest.skip("Wrong result after change to classic DH")
    def test_fkine(self):
        """Test forward kinematics, second example from Corke 2017 page 204."""
        qz = ZERO6
        T = self.robot.fkine(qz)
        self.assertIsInstance(T, Pose3)
        self.gtsamAssertEquals(
            T, Pose3(Rot3.Rx(math.pi), Point3(0.4521, 0.15, 2.3682)), tol=1e-4)


if __name__ == "__main__":
    unittest.main()
