"""
Explore closed-form 6DOF elimination IK idea.
Requires sympy to be installed.
"""


import math
import os
import time
import unittest

import gtsam
import numpy as np
import sympy

from faster_ik import FasterIK, FasterIK_R6, FasterIK_RRR
from link_parameters import R6_calibration_dh, RRR_calibration_dh
from serial_link import SerialLink
from utils import GtsamTestCase, point3_of_vector, vector

MY_PATH = os.path.dirname(os.path.realpath(__file__))
URDFS_PATH = os.path.join(MY_PATH, '../urdfs')
sympy.init_printing()


O = np.array([[0], [0], [0]])
X = np.array([[1], [0], [0]])
Y = np.array([[0], [1], [0]])
Z = np.array([[0], [0], [1]])

I3 = np.column_stack([X, Y, Z])


def pose(R, t):
    """Create a Pose3 out of 2 numpy arrays."""
    return gtsam.Pose3(gtsam.Rot3(R), point3_of_vector(t.flatten()))


class TestFasterIKRRR(GtsamTestCase):
    """Unit tests for FasterIK class, using RRR elbow arm."""

    def setUp(self):
        """Read URDF from file and create class instance."""
        self.elbow = SerialLink(RRR_calibration_dh)
        self.ik = FasterIK_RRR(self.elbow)

    def test_rotation(self):
        """Check expression for rotation matrix at end of three links."""
        R12 = np.column_stack([X, Z, -Y])
        R23 = np.column_stack([X, Y, Z])
        R34 = np.column_stack([X, -Z, Y])
        R3 = [R12, R23, R34]
        Re, Zs, constraints, syms = FasterIK.rotation(*R3)
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 3)
        self.assertEqual(len(syms), 6)

    def test_pose(self):
        """Check expression for pose at end of three links."""
        Re, te, _, constraints, syms = self.ik.pose()
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 3)
        self.assertEqual(len(syms), 6)

    def test_fk(self):
        """Check forward kinematics for two configurations of RRR."""
        R, t = self.ik.fk(vector(0, 0, 0))  # rest
        np.testing.assert_array_almost_equal(t, vector(2, 0, 1))
        np.testing.assert_array_almost_equal(R, np.column_stack([X, Y, Z]))

        R, t = self.ik.fk(vector(0, math.pi/2, 0))  # up
        np.testing.assert_array_almost_equal(t, vector(0, 0, 3))
        np.testing.assert_array_almost_equal(R, np.column_stack([Z, Y, -X]))

    # @unittest.skip("too slow")
    def test_solve(self):
        solutions, syms = self.ik.solve(lock_theta1=True)
        self.assertEqual(len(solutions), 2)
        self.assertEqual(len(syms), 6)

        x, z = syms[-2:]
        numerical = FasterIK.solutions_for(solutions, (x, 1.5), (z, 2))

        np.testing.assert_array_almost_equal(
            numerical[0], [0.51, 0.86, 0.625, -0.781], decimal=2)
        np.testing.assert_array_almost_equal(
            numerical[1], [0.99, 0.14, 0.625, 0.781], decimal=2)


class TestFasterIKR6(GtsamTestCase):
    """Unit tests for FasterIK class, using 6DOF manipulator."""

    def setUp(self):
        """Read URDF from file and create class instance."""
        self.r6 = SerialLink(R6_calibration_dh)
        self.ik = FasterIK_R6(self.r6)

    def test_pose(self):
        """Check expression for pose at end of 6DOF manipulator."""
        Re, te, Zs, constraints, syms = self.ik.pose()
        self.assertIsInstance(Re, sympy.Matrix)
        self.assertIsInstance(constraints, list)
        self.assertEqual(len(constraints), 6)
        self.assertEqual(len(syms), 12)

    def test_fk(self):
        """Check forward kinematics for two configurations of 6DOF manipulator."""
        R, t = self.ik.fk(vector(0, 0, 0, 0, 0, 0))  # rest
        np.testing.assert_array_almost_equal(t, vector(2, 0, 2))
        np.testing.assert_array_almost_equal(R, np.column_stack([X, Y, Z]))

        R, t = self.ik.fk(vector(0, math.pi/2, 0, 0, 0, 0))  # up
        np.testing.assert_array_almost_equal(t, vector(-1, 0, 3))
        np.testing.assert_array_almost_equal(R, np.column_stack([Z, Y, -X]))

    @unittest.skip("too slow")
    def test_solve(self):
        solutions, syms = self.ik.solve(True)
        self.assertEqual(len(solutions), 4)
        self.assertEqual(len(syms), 12)

        x, z = syms[-2:]
        numerical = FasterIK.solutions_for(solutions, (x, 1.5), (z, 2))

        np.testing.assert_array_almost_equal(
            numerical[0], [0.75, 0.661, 0.125, -0.992, -1.0, 0, 0.75, -0.661, -1.0, 0], decimal=2)
        np.testing.assert_array_almost_equal(
            numerical[1], [0.75, 0.661, 0.125, -0.992, 1.0, 0, 0.75, 0.661, 1.0, 0], decimal=2)
        np.testing.assert_array_almost_equal(
            numerical[2], [0.75, -0.661, 0.125, 0.992, -1.0, 0, 0.75, 0.661, -1.0, 0], decimal=2)
        np.testing.assert_array_almost_equal(
            numerical[3], [0.75, -0.661, 0.125, 0.992, 1.0, 0, 0.75, -0.661, 1.0, 0], decimal=2)


class TestFasterIKFanuc(GtsamTestCase):
    """Unit tests for FasterIK class, using RRR elbow arm."""

    def setUp(self):
        """Read URDF from file and create class instance."""
        urdf_file = os.path.join(URDFS_PATH, "fanuc_lrmate200id.urdf")
        self.ik = self.ik.from_urdf(urdf_file, "Part6")


if __name__ == "__main__":
    unittest.main()
