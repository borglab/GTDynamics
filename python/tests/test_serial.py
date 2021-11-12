"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_serial.py
 * @brief Test Serial class with Panda robot.
 * @author Frank Dellaert
"""

import unittest
from pathlib import Path
from typing import Optional, Tuple

import gtdynamics as gtd
import numpy as np
from gtsam import Point3, Pose3, Rot3, Values
from gtsam.utils.test_case import GtsamTestCase
from prototype.serial import Serial


def axis(*A):
    """Make n*1 axis from list"""
    return np.array([list(A)], dtype=float).transpose()


class TestSerial(GtsamTestCase):
    """Test Serial FK in GTD context."""

    def test_one_link(self):
        """Test creating just one link."""
        sMb, Jb = Pose3(Rot3(), Point3(5, 0, 0)), axis(0, 0, 1, 0, 5, 0)
        self.assertEqual(Jb.shape, (6, 1))
        joint1 = Serial(sMb, Jb)
        self.assertIsInstance(joint1, Serial)

        # FK at rest
        self.gtsamAssertEquals(joint1.poe([0]), sMb)
        sTb = Pose3(Rot3.Rz(np.pi/2), Point3(0, 5, 0))

        # FK not at rest
        q = [np.pi/2]
        self.gtsamAssertEquals(joint1.poe(q), sTb)
        J = np.zeros((6, 1))
        self.gtsamAssertEquals(joint1.poe(q, J=J), sTb)
        np.testing.assert_allclose(J, Jb)

    def test_three_link(self):
        """Test creating a two-link arm in SE(2)."""
        sMb, Jb = Pose3(Rot3(), Point3(5, 0, 0)), axis(0, 0, 1, 0, 5, 0)
        joint1 = Serial(sMb, Jb)
        joint2 = Serial(sMb, Jb)
        joint3 = Serial(sMb, Jb)
        three_links = Serial.compose(joint1, joint2, joint3)
        self.assertIsInstance(three_links, Serial)
        sM3, J3 = three_links.spec()
        expected = Pose3(Rot3(), Point3(15, 0, 0))
        self.gtsamAssertEquals(sM3, expected)
        expected_J3 = np.array(
            [[0, 0, 1, 0, 15, 0],
             [0, 0, 1, 0, 10,  0],
             [0, 0, 1, 0, 5,  0]]).transpose()
        np.testing.assert_allclose(J3, expected_J3)

        # FK at rest
        self.assertIsInstance(three_links.spec(), tuple)
        self.gtsamAssertEquals(three_links.poe([0, 0, 0]), expected)

        # FK not at rest
        q = np.array([0, 0, np.pi/2])
        sT3 = Pose3(Rot3.Rz(np.pi/2), Point3(10, 5, 0))
        self.gtsamAssertEquals(three_links.poe(q), sT3)
        expected_J1 = sT3.inverse().Adjoint(axis(0, 0, 1, 0, 0, 0))
        np.testing.assert_allclose(expected_J1, [0, 0, 1, 10, 5, 0])
        expected_J = np.array(
            [expected_J1,
             [0, 0, 1, 5, 5,  0],
             [0, 0, 1, 0, 5,  0]]).transpose()
        J = np.zeros((6, 3))
        self.gtsamAssertEquals(three_links.poe(q, J=J), sT3)
        np.testing.assert_allclose(J, expected_J)


# load example robot
MODEL_FILE = Path(gtd.URDF_PATH) / "panda" / "panda.urdf"
BASE_NAME = "link0"
# Crucial to fix base link or FK gives wrong result
ROBOT = gtd.CreateRobotFromFile(str(MODEL_FILE)).fixLink(BASE_NAME)


class TestPanda(GtsamTestCase):
    """Test Serial FK applied to Panda."""

    def setUp(self):
        """Set up the fixtures."""

        # Create serial sub-system
        self.serial = Serial.from_robot(ROBOT, BASE_NAME)

    @ staticmethod
    def JointAngles(q: list):
        """Create Values with joint angles."""
        joint_angles = Values()
        for j, q_j in enumerate(q):
            gtd.InsertJointAngle(joint_angles, j, q_j)
        return joint_angles

    def test_panda_FK(self):
        """Test GTSAM forward kinematics at rest."""

        # First check link 0 is fixed:
        self.assertTrue(ROBOT.link("link0").isFixed())

        # Check FK at rest, Conventional FK with GTSAM.
        joint_angles = self.JointAngles(np.zeros((7,)))
        fk = ROBOT.forwardKinematics(joint_angles, 0, BASE_NAME)
        # Use this to print: fk.print("fk", gtd.GTDKeyFormatter)
        sR7 = Rot3([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        expected_sM7 = Pose3(sR7, Point3(0.0882972, 0.00213401, 0.933844))
        actual_sM7 = gtd.Pose(fk, 7)
        self.gtsamAssertEquals(actual_sM7, expected_sM7, tol=1e-3)

        # Check that Serial can be constructed from robot
        sM7, J7 = self.serial.spec()
        self.gtsamAssertEquals(sM7, expected_sM7, tol=1e-3)

        # Check Panda twsist in end-effector frame
        self.assertEqual(J7.shape, (6, 7))
        expected_J7 = np.array([
            [0,   0,   -1,    -0.002, -0.088,  0],
            [0,   -1,     0,    0.601,  0,    0.088],
            [0,   0,   -1,    -0.002, -0.088,  0],
            [0,    1,    0,   -0.285, 0,   -0.006],
            [0,   0,   -1,    -0.002, -0.088,  0],
            [0,    1,    0,    0.099, 0,   -0.088],
            [0,    0,    1,     0.002,  0,    0]]).transpose()
        np.testing.assert_allclose(J7, expected_J7, atol=0.001)

    def check_poe(self, q_list):
        """Test FK with POE"""
        joint_angles = self.JointAngles(q_list)
        q = np.array(q_list)

        # Conventional FK with GTSAM.
        fk = ROBOT.forwardKinematics(joint_angles, 0, BASE_NAME)
        sT7 = gtd.Pose(fk, 7)

        # FK with POE.
        poe_sT7 = self.serial.poe(q)
        self.gtsamAssertEquals(poe_sT7, sT7, tol=1e-7)

        # FK with POE and Jacobians
        poe_J = np.zeros((6, 7))
        poe_sT7 = self.serial.poe(q, J=poe_J)
        self.gtsamAssertEquals(poe_sT7, sT7, tol=1e-7)

        # Check that last column in Jacobian is unchanged, because that twist
        # affects the end-effector one on one.
        np.testing.assert_allclose(poe_J[:, 6], self.serial.axes[:, 6])

        # Check derivatives
        q_list[6] += 0.00001
        q = np.array(q_list)
        sT7_plus = self.serial.poe(q)
        xi = sT7.logmap(sT7_plus)/0.00001
        np.testing.assert_allclose(poe_J[:, 6], xi, atol=0.01)

    def test_forward_kinematics_at_rest(self):
        """Test forward kinematics at rest."""
        self.check_poe([0, 0, 0, 0, 0, 0, 0])

    def test_forward_kinematics_joint0(self):
        """Test forward kinematics with non-zero joint0 angle."""
        self.check_poe([np.pi/2, 0, 0, 0, 0, 0, 0])

    def test_forward_kinematics_middle(self):
        """Test forward kinematics with middle joint rotated."""
        self.check_poe([0, 0, 0, 0, np.pi/2, 0, 0])

    def test_forward_kinematics_random(self):
        """Test forward kinematics with random configuration."""
        self.check_poe([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])

    def test_panda_decomposition(self):
        """Test composition of Panda as shoulder and arm"""
        # Construct Panda arm with compose
        shoulder = Serial.from_robot(ROBOT, BASE_NAME, (0, 3))
        arm = Serial.from_robot(ROBOT, joint_range=(3, 6))
        wrist = Serial.from_robot(ROBOT, joint_range=(6, 7))
        panda = Serial.compose(shoulder, arm, wrist)
        self.assertEqual(panda.axes.shape, (6, 7))

        # Conventional FK with GTSAM.
        q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        joint_angles = self.JointAngles(q)
        fk = ROBOT.forwardKinematics(joint_angles, 0, BASE_NAME)
        sTb = gtd.Pose(fk, 7)

        # FK with POE.
        poe_sTb = panda.poe(q)
        self.gtsamAssertEquals(poe_sTb, sTb, tol=1e-7)


if __name__ == "__main__":
    unittest.main()
