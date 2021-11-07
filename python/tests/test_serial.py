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
from typing import Optional

import gtdynamics as gtd
import numpy as np
from gtsam import Point3, Pose3, Rot3, Values
from gtsam.utils.test_case import GtsamTestCase


class Serial():
    """Three-link arm class."""

    def __init__(self, robot: gtd.Robot, base_name: str):
        """Initialize from a robot with given base link"""
        joints = robot.joints()
        base_link = robot.link(base_name)

        # Calculate all joint screw axes at rest, in base frame:
        self.axes = []
        sTl = base_link.bMcom()
        for joint in joints:
            # Calculate next link, at rest:
            sTl = sTl.compose(joint.pMc())
            # Translate joint axis, expressed in child frame, to base_link.
            s_axis_j = sTl.Adjoint(joint.cScrewAxis())
            self.axes.append(s_axis_j)

        # End-effector (for now) is just last link.
        self.sTe = sTl

    def poe(self, q: np.ndarray, 
            sTe: Optional[Pose3] = None,
            J: Optional[np.ndarray] = None):
        """ Perform forward kinematics given q, return Pose of end-effector.
            When q is smaller than #joints, actuates last joints in chain.

        Arguments:
            q (np.ndarray): joint angles for all joints.
            sTe: optionally, the end-effector pose at rest, default last link.
            J: optionally, the manipulator Jacobian.
        Returns:
            jTe (Pose3)
        """
        j = len(q) # joint number counting from last, base 1.
        if j == 0:
            # no more joints, return end-effector pose:
            return sTe if sTe is not None else self.sTe
        # Do recursive call and then transform according to this joint.
        axis = self.axes[-j]
        T = Pose3.Expmap(axis * q[0]).compose(self.poe(q[1:], sTe=sTe, J=J))
        # Fill in optional Jacobian, if asked.
        if J is not None:
            J[:, -j] = T.inverse().Adjoint(axis)
        return T

class TestSerial(GtsamTestCase):
    """Test Serial FK in GTD context."""

    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        model_file=Path(gtd.URDF_PATH) / "panda" / "panda.urdf"
        self.base_name="link0"
        # Crucial to fix base link or FK gives wrong result
        self.robot=gtd.CreateRobotFromFile(
            str(model_file)).fixLink(self.base_name)

        # Create serial sub-system
        self.serial=Serial(self.robot, self.base_name)

    @ staticmethod
    def JointAngles(q: list):
        """Create Values with joint angles."""
        joint_angles=Values()
        for j, q_j in enumerate(q):
            gtd.InsertJointAngle(joint_angles, j, q_j)
        return joint_angles

    def test_panda_FK(self):
        """Test GTSAM forward kinematics at rest."""

        # First check link 0 is fixed:
        self.assertTrue(self.robot.link("link0").isFixed())

        # Check FK at rest, Conventional FK with GTSAM.
        joint_angles = self.JointAngles(np.zeros((7,)))
        fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)
        # Use this to print: fk.print("fk", gtd.GTDKeyFormatter)
        sR7 = Rot3([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        expected_sT7 = Pose3(sR7, Point3(0.0882972, 0.00213401, 0.933844))
        actual_sT7 = gtd.Pose(fk, 7)
        self.gtsamAssertEquals(actual_sT7, expected_sT7, tol=1e-3)

    def check_poe(self,q):
        """Test FK with POE"""
        joint_angles=self.JointAngles(q)

        # Conventional FK with GTSAM.
        fk=self.robot.forwardKinematics(joint_angles, 0, self.base_name)

        # FK with POE.
        J=np.zeros((6, 7))
        poe_sT7=self.serial.poe(q=np.array(q), J=J)
        self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)

        # Check derivative
        q[0] += 0.01
        poe_sT7_plus=self.serial.poe(np.array(q))
        delta=poe_sT7.logmap(poe_sT7_plus)/0.01
        np.testing.assert_allclose(J[:, 0], delta, atol=0.01)

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

if __name__ == "__main__":
    unittest.main()
