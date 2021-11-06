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
        # Calculate all joint screw axes at rest, in base frame:
        self.axes = []
        sTl = robot.link(base_name).bMcom()
        for joint in robot.joints():
            # Calculate next link, at rest:
            sTl = sTl.compose(joint.pMc())
            # Translate joint axis, expressed in child frame, to base.
            axis_j = sTl.Adjoint(joint.cScrewAxis())
            self.axes.append(axis_j)

        # end-effector (for now) is just last link
        self.sTe = sTl

    def poe(self, q: np.ndarray):
        """ Forward kinematics.
            Takes numpy array of joint angles, in radians.
        """
        assert q.shape == (len(self.axes),), f"q has wrong shape {q.shape}"
        result = Pose3()
        for axis_j, q_j in zip(self.axes, q):
            result = result.expmap(axis_j * q_j)
        return result.compose(self.sTe)

    def joint_from_ee(self, q: np.ndarray, j=0, J: Optional[np.ndarray] = None):
        """ Calculate jTe, the pose of the end-effector in joint frame j.

        Arguments:
            q (np.ndarray): joint angles for all joints.
            j (int): base 1 joint index, where j==0 signified base frame.
            J: optionally, the manipulator Jacobian.
        Returns:
            jTe (Pose3)
        """
        return Pose3()


class TestSerial(GtsamTestCase):
    """Test Serial FK in GTD context."""

    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        model_file = Path(gtd.URDF_PATH) / "panda" / "panda.urdf"
        self.base_name = "link0"
        # Crucial to fix base link or FK gives wrong result
        self.robot = gtd.CreateRobotFromFile(
            str(model_file)).fixLink(self.base_name)

        # Create serial sub-system
        self.serial = Serial(self.robot, self.base_name)

    def test_joint_from_ee(self):
        """Test forward kinematics with random configuration."""
        q = [0.0]*7  # [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        joint_angles = self.JointAngles(q)

        # Conventional FK with GTSAM.
        fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)

        # FK with POE.
        poe_sT7 = self.serial.joint_from_ee(q=np.array(q))
        self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)

    @staticmethod
    def JointAngles(q: list):
        """Create Values with joint angles."""
        joint_angles = Values()
        for j, q_j in enumerate(q):
            gtd.InsertJointAngle(joint_angles, j, q_j)
        return joint_angles

    # def test_forward_kinematics_at_rest(self):
    #     """Test forward kinematics at rest."""

    #     # First check link 0 is fixed:
    #     self.assertTrue(self.robot.link("link0").isFixed())

    #     # Check FK at rest, Conventional FK with GTSAM.
    #     joint_angles = self.JointAngles(np.zeros((7,)))
    #     fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)
    #     # Use this to print: fk.print("fk", gtd.GTDKeyFormatter)
    #     sR7 = Rot3([
    #         [1, 0, 0],
    #         [0, -1, 0],
    #         [0, 0, -1]
    #     ])
    #     expected_sT7 = Pose3(sR7, Point3(0.0882972, 0.00213401, 0.933844))
    #     actual_sT7 = gtd.Pose(fk, 7)
    #     self.gtsamAssertEquals(actual_sT7, expected_sT7, tol=1e-3)

    #     # Check end-effector at rest in serial sub-system.
    #     self.gtsamAssertEquals(self.serial.sTe, expected_sT7, tol=1e-3)

    #     # FK with POE.
    #     poe_sT7 = self.serial.poe(q=np.zeros((7,)))
    #     self.gtsamAssertEquals(poe_sT7, expected_sT7, tol=1e-3)

    # def test_forward_kinematics_joint0(self):
    #     """Test forward kinematics with non-zero joint0 angle."""
    #     q = [np.pi/2, 0, 0, 0, 0, 0, 0]
    #     joint_angles = self.JointAngles(q)

    #     # Conventional FK with GTSAM.
    #     fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)

    #     # FK with POE.
    #     poe_sT7 = self.serial.poe(q=np.array(q))
    #     self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)

    # def test_forward_kinematics_middle(self):
    #     """Test forward kinematics with middle joint rotated."""
    #     q = [0, 0, 0, 0, np.pi/2, 0, 0]
    #     joint_angles = self.JointAngles(q)

    #     # Conventional FK with GTSAM.
    #     fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)

    #     # FK with POE.
    #     poe_sT7 = self.serial.poe(q=np.array(q))
    #     self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)

    # def test_forward_kinematics_random(self):
    #     """Test forward kinematics with random configuration."""
    #     q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    #     joint_angles = self.JointAngles(q)

    #     # Conventional FK with GTSAM.
    #     fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)

    #     # FK with POE.
    #     poe_sT7 = self.serial.poe(q=np.array(q))
    #     self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)


if __name__ == "__main__":
    unittest.main()
