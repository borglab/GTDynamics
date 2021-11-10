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


def compose(A: Tuple[Pose3, np.ndarray], B: Tuple[Pose3, np.ndarray]):
    """Monoid operation for pose,Jacobian pairs."""
    aTb, Jb = A
    bTc, Jc = B
    assert Jb.shape[0] == 6 and Jc.shape[0] == 6

    # Compose poses
    aTc = aTb.compose(bTc)

    # Check if one of specs is an offset:
    if Jb.shape[1] == 0:
        return aTc, Jc

    c_Ad_b = bTc.inverse().AdjointMap()
    if Jc.shape[1] == 0:
        return aTc, c_Ad_b @ Jb

    # if not, do normal case:
    return aTc, np.hstack((c_Ad_b @ Jb, Jc))


class Serial():
    """Three-link arm class."""

    def __init__(self, sMb, axes: np.ndarray):
        """Create from end-effector at rest and Jacobian.

        Arguments:
            sMb: rest pose of "body" with respect to "spatial" frame
            axes: screw axes of all joints expressed in body frame
        """
        assert isinstance(sMb, Pose3)
        assert isinstance(axes, np.ndarray)
        self.sMb = sMb
        self.axes = np.expand_dims(axes, 1) if len(axes.shape) == 1 else axes

    @classmethod
    def compose(cls, *components):
        """Create from a variable number of other Serial instances."""
        spec = components[0].spec()
        for component in components[1:]:
            spec = compose(spec, component.spec())
        return cls(*spec)

    def spec(self):
        """Return end-effector at rest and Jacobian."""
        return self.sMb, self.axes

    def __repr__(self):
        return f"Serial\n: {self.sMb}\n{np.round(self.axes,3)}\n"

    @classmethod
    def from_robot(cls, robot: gtd.Robot, base_name: str):
        """Initialize from a robot with given base link"""
        # Create offset to first link parent
        base_link = robot.link(base_name)
        sM0 = base_link.bMcom()
        offset = Serial(sM0, np.zeros((6, 0)))

        # Convert all joints into Serial instances
        joints = [offset]+[cls(joint.pMc(),
                               joint.cScrewAxis()) for joint in robot.joints()]

        # Now, let compose do the work!
        return cls.compose(*joints)

    def poe(self, q: np.ndarray,
            fTe: Optional[Pose3] = None,
            J: Optional[np.ndarray] = None):
        """ Perform forward kinematics given q, return Pose of end-effector.
            When q is smaller than #joints, actuates last joints in chain.

        Arguments:
            q (np.ndarray): joint angles for all joints.
            fTe: optionally, the end-effector pose with respect to final link.
            J: optionally, the manipulator Jacobian.
        Returns:
            jTe (Pose3)
        """
        sMe = self.sMb if fTe is None else self.sMb.compose(fTe)
        if J is None:
            # FK with monoid.
            poe = self.f(self.axes, q)
            return sMe.compose(poe)
        else:
            # FK + Jacobian with monoid.
            assert J.shape == (6, len(q)), f"Needs 6x{len(q)} J."
            sMeJ = sMe, np.zeros((6, 0))
            T, G = compose(sMeJ, self.g(self.axes, q))
            J[:, :] = G
            return T

    @staticmethod
    def f(A: np.ndarray, q: np.ndarray):
        """ Perform forward kinematics given q.

        Arguments:
            A (np.ndarray): joint axes.
            q (np.ndarray): joint angles.
        Returns:
            sTb (Pose3): map from body to spatial frame.
        """
        if len(q) == 0:
            return Pose3()
        T = Pose3.Expmap(A[:, 0] * q[0])
        return T if len(q) == 1 else \
            T.compose(Serial.f(A[:, 1:], q[1:]))

    @staticmethod
    def g(A: np.ndarray, q: np.ndarray):
        """ Perform forward kinematics given q, with Jacobian.

        Arguments:
            A (np.ndarray): joint axes.
            q (np.ndarray): joint angles.
        Returns:
            sTb (Pose3): map from body to spatial frame.
            J (np.ndarray): manipulator Jacobian for this arm.
        """
        if len(q) == 0:
            return Pose3(), np.zeros((6, 0))
        A0 = np.expand_dims(A[:, 0], 1)
        T = Pose3.Expmap(A0 * q[0])
        return (T, A0) if len(q) == 1 else \
            compose((T, A0), Serial.g(A[:, 1:], q[1:]))


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

    @staticmethod
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

    # def test_panda_decomposition(self):
    #     """Test composition of Panda as shoulder and arm"""
    #     q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    #     joint_angles = self.JointAngles(q)

    #     # Conventional FK with GTSAM.
    #     fk = ROBOT.forwardKinematics(joint_angles, 0, BASE_NAME)
    #     expected = gtd.Pose(fk, 7)

    #     # FK with monoid.
    #     q = np.array(q)
    #     A = self.serial.A
    #     shoulder = self.serial.f(A[:3], q[:3])
    #     arm = self.serial.f(A[3:], q[3:])
    #     f_7Ts = shoulder.compose(arm).compose(self.serial.fTe)
    #     self.gtsamAssertEquals(f_7Ts, expected, tol=1e-3)

    #     # FK + Jacobian with monoid.
    #     shoulder = self.serial.g(A[:3], q[:3])
    #     print(f"shoulder J:\n{np.round(shoulder[1],3)}")
    #     arm = self.serial.g(A[3:], q[3:])
    #     print(f"arm J:\n{np.round(arm[1],3)}")
    #     g_7Ts, g_J = compose(compose(shoulder, arm),
    #                          (self.serial.fTe, np.zeros((6, 0))))
    #     print(f"panda J:\n{np.round(g_J,3)}")
    #     self.gtsamAssertEquals(g_7Ts, expected, tol=1e-3)

    #     # Check derivatives
    #     q[0] += 0.01
    #     T_plus = self.serial.poe(np.array(q))
    #     xi_0 = expected.logmap(T_plus)/0.01
    #     np.testing.assert_allclose(g_J[:, 0], xi_0, atol=0.01)


if __name__ == "__main__":
    unittest.main()
