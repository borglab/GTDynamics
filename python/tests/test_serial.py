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
        self.s_axes = []
        sTl = base_link.bMcom()
        for joint in joints:
            # Calculate next link, at rest:
            sTl = sTl.compose(joint.pMc())
            # Translate joint axis, expressed in child frame, to base_link.
            s_axis_j = sTl.Adjoint(joint.cScrewAxis())
            self.s_axes.append(s_axis_j)

        # End-effector (for now) is just last link.
        self.sTe = sTl

        # Calculate all axes in their own frame:
        self.axes = [joint.jMc().Adjoint(joint.cScrewAxis())
                     for joint in joints]

        # Calculate transforms between joints.
        # First is 0T1, from base frame to joint 1 frame
        self.next = [base_link.bMcom().compose(joints[0].jMp().inverse())]
        # Then we add 1T2, 2T3, 3T4, 4T5, 5T6, 6T7 for a 7 DOF arm:
        for joint1, joint2 in zip(joints[:-1], joints[1:]):
            self.next.append(joint1.jMc().compose(joint2.jMp().inverse()))
        # We then add one more to get 7TE
        self.next.append(joints[-1].jMc())

    def poe(self, q: np.ndarray):
        """ Forward kinematics.
            Takes numpy array of joint angles, in radians.
        """
        assert q.shape == (len(self.s_axes),), f"q has wrong shape {q.shape}"
        result = Pose3()
        for axis_j, q_j in zip(self.s_axes, q):
            result = result.expmap(axis_j * q_j)
        return result.compose(self.sTe)

    def joint_from_next(self, q: np.ndarray, j=0):
        """Return pose of next joint j+1 in this joint's frame.

        Arguments:
            q (np.ndarray): joint angles for all joints.
            j (int): base 1 joint index, where j==0 signified base frame.
        """
        assert j >= 0 and j <= len(q)
        return self.next[j].expmap(self.axes[j] * q[j])

    def joint_from_ee(self, q: np.ndarray, j=0, J: Optional[np.ndarray] = None):
        """ Calculate jTe, the pose of the end-effector in joint frame j.

        Arguments:
            q (np.ndarray): joint angles for all joints.
            j (int): base 1 joint index, where j==0 signified base frame.
            J: optionally, the manipulator Jacobian.
        Returns:
            jTe (Pose3)
        """
        assert(type(q) == np.ndarray)
        assert(type(j) == int)
        num_joints = len(q)
        assert q.shape == (num_joints,)
        assert J is None or J.shape == (6, num_joints)

        jTe = self.next[-1] if j == num_joints else \
            self.joint_from_next(q, j).compose(self.joint_from_ee(q, j+1, J))
        if j != 0 and J is not None:
            J[:, j-1] = jTe.inverse().Adjoint(self.axes[j-1])
        return jTe


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
        """Test joint_from_ee at rest."""
        q = [0.0]*7
        joint_angles = self.JointAngles(q)

        # Conventional FK with GTSAM.
        fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)

        # FK with POE.
        J = np.ones((6, 7))
        poe_sT7 = self.serial.joint_from_ee(np.array(q), 0, J)
        self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)

        # Check derivative
        q[0]+=0.01
        poe_sT7_plus = self.serial.joint_from_ee(np.array(q), 0, J)
        delta = poe_sT7.logmap(poe_sT7_plus)/0.01
        np.testing.assert_allclose(J[:,0], delta, atol=0.01)


    def test_joint_from_ee_random(self):
        """Test joint_from_ee with random configuration."""
        q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        joint_angles = self.JointAngles(q)

        # Conventional FK with GTSAM.
        fk = self.robot.forwardKinematics(joint_angles, 0, self.base_name)

        # FK with POE.
        J = np.full((6, 7), np.NaN, float)
        poe_sT7 = self.serial.joint_from_ee(np.array(q), 0, J)
        self.gtsamAssertEquals(poe_sT7, gtd.Pose(fk, 7), tol=1e-3)

        # Check derivative
        q[0]+=0.01
        poe_sT7_plus = self.serial.joint_from_ee(np.array(q), 0, J)
        delta = poe_sT7.logmap(poe_sT7_plus)/0.01
        np.testing.assert_allclose(J[:,0], delta)
        
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
