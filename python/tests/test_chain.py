"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_chain.py
 * @brief Test Chain class with Panda robot.
 * @author Frank Dellaert
"""

import unittest
from pathlib import Path

import gtdynamics as gtd
import numpy as np
from gtsam import Point3, Pose3, Rot3, Values
from gtsam.utils.test_case import GtsamTestCase

from prototype.chain import Chain


def axis(*A):
    """Make n*1 axis from list"""
    return np.array([list(A)], dtype=float).transpose()


class TestChain(GtsamTestCase):
    """Test Chain FK in GTD context."""

    def test_one_link(self):
        """Test creating just one link."""
        sMb, Jb = Pose3(Rot3(), Point3(5, 0, 0)), axis(0, 0, 1, 0, 5, 0)
        self.assertEqual(Jb.shape, (6, 1))
        joint1 = Chain(sMb, Jb)
        self.assertIsInstance(joint1, Chain)

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
        joint1 = Chain(sMb, Jb)
        joint2 = Chain(sMb, Jb)
        joint3 = Chain(sMb, Jb)
        three_links = Chain.compose(joint1, joint2, joint3)
        self.assertIsInstance(three_links, Chain)
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
    """Test Chain FK applied to Panda."""

    def setUp(self):
        """Set up the fixtures."""

        # Create chain sub-system
        self.chain = Chain.from_robot(ROBOT, BASE_NAME)

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
        #regression
        expected_sM7 = Pose3(sR7, Point3(0.098517, 0.004252, 0.971403))
        actual_sM7 = gtd.Pose(fk, 7)
        self.gtsamAssertEquals(actual_sM7, expected_sM7, tol=1e-3)

        # Check that Chain can be constructed from robot
        sM7, J7 = self.chain.spec()
        self.gtsamAssertEquals(sM7, expected_sM7, tol=1e-3)

        # Check Panda twist in end-effector frame
        self.assertEqual(J7.shape, (6, 7))
        expected_J7 = np.array([[0, 0, -1, -0.004252, -0.0985, 0],
                                [0, -1, 0, 0.6384, 0, 0.0985],
                                [0, 0, -1, -0.00425, -0.0985, 0],
                                [0, 1, 0, -0.322, 0, -0.016],
                                [0, 0, -1, -0.00425, -0.0985, 0],
                                [0, 1, 0, 0.06159, 0, -0.0985],
                                [0, 0, 1, 0.00425, 0.0105, 0]]).transpose()
        #regression
        np.testing.assert_allclose(J7, expected_J7, atol=0.001)

    def check_poe(self, q_list, fTe=None):
        """Test FK with POE"""
        joint_angles = self.JointAngles(q_list)
        q = np.array(q_list)

        # Conventional FK with GTSAM.
        fk = ROBOT.forwardKinematics(joint_angles, 0, BASE_NAME)
        sT7 = gtd.Pose(fk, 7)
        if fTe is not None:
            sT7 = sT7.compose(fTe)

        # FK with POE.
        poe_sT7 = self.chain.poe(q, fTe=fTe)
        self.gtsamAssertEquals(poe_sT7, sT7, tol=1e-7)

        # FK with POE and Jacobians
        poe_J = np.zeros((6, 7))
        poe_sT7 = self.chain.poe(q, fTe=fTe, J=poe_J)
        self.gtsamAssertEquals(poe_sT7, sT7, tol=1e-7)

        # Unless we apply an extra transform at the end, check that last column
        # in the Jacobian is unchanged from the axes, because that twist
        # affects the end-effector one on one.
        if fTe is None:
            np.testing.assert_allclose(poe_J[:, 6], self.chain.axes[:, 6])

        # Check derivatives
        q_list[6] += 0.00001
        q = np.array(q_list)
        sT7_plus = self.chain.poe(q, fTe=fTe)
        xi = sT7.logmap(sT7_plus)/0.00001
        np.testing.assert_allclose(poe_J[:, 6], xi, atol=0.01)

    def test_forward_kinematics_at_rest(self):
        """Test forward kinematics at rest."""
        self.check_poe([0, 0, 0, 0, 0, 0, 0])

    def test_forward_kinematics_at_rest_with_offset(self):
        """FK at rest, with offset."""
        fTe = Pose3(Rot3.Ry(0.1), Point3(1, 2, 3))
        self.check_poe([0, 0, 0, 0, 0, 0, 0], fTe)

    def test_forward_kinematics_joint0(self):
        """Test forward kinematics with non-zero joint0 angle."""
        self.check_poe([np.pi/2, 0, 0, 0, 0, 0, 0])

    def test_forward_kinematics_joint0_with_offset(self):
        """FK with non-zero joint0, with offset."""
        fTe = Pose3(Rot3.Ry(0.1), Point3(1, 2, 3))
        self.check_poe([np.pi/2, 0, 0, 0, 0, 0, 0], fTe)

    def test_forward_kinematics_middle(self):
        """Test forward kinematics with middle joint rotated."""
        self.check_poe([0, 0, 0, 0, np.pi/2, 0, 0])

    def test_forward_kinematics_random(self):
        """Test forward kinematics with random configuration."""
        self.check_poe([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])

    def test_forward_kinematics_random_offset(self):
        """FK with random configuration, plus offset."""
        fTe = Pose3(Rot3.Ry(0.1), Point3(1, 2, 3))
        self.check_poe([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7], fTe)

    def test_panda_decomposition(self):
        """Test composition of Panda as shoulder and arm"""
        # Construct Panda arm with compose
        shoulder = Chain.from_robot(ROBOT, BASE_NAME, (0, 3))
        arm = Chain.from_robot(ROBOT, joint_range=(3, 6))
        wrist = Chain.from_robot(ROBOT, joint_range=(6, 7))
        panda = Chain.compose(shoulder, arm, wrist)
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
