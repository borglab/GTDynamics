"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_dynamics_graph.py
 * @brief Unit tests for dynamics graph.
 * @author Varun Agrawal
"""

import os.path as osp
import unittest

import gtsam
import numpy as np

import gtdynamics as gtd


class TestJointMeasurementFactor(unittest.TestCase):
    """Test suite for various versions of the JointMeasurementFactor."""
    def setUp(self):
        self.k = 0
        self.wTp_key = gtd.internal.PoseKey(0, self.k).key()
        self.wTc_key = gtd.internal.PoseKey(1, self.k).key()

        ROBOT_FILE = osp.join(gtd.SDF_PATH, "test", "simple_rr.sdf")
        self.robot = gtd.CreateRobotFromFile(str(ROBOT_FILE), "simple_rr_sdf")

    def test_revolute_joint_measurement_factor(self):
        """Test RevoluteJointMeasurementFactor."""
        factor = gtd.JointMeasurementFactor(gtsam.noiseModel.Isotropic.Sigma(6, 0.1),
                                            self.robot.joint("joint_1"), np.pi / 4, self.k)

        self.assertTrue(isinstance(factor, gtd.JointMeasurementFactor))

class TestDynamicsFactors(unittest.TestCase):
    def setUp(self):
        self.k = 0
        ROBOT_FILE = osp.join(gtd.SDF_PATH, "test", "simple_rr.sdf")
        self.robot = gtd.CreateRobotFromFile(str(ROBOT_FILE), "simple_rr_sdf")
        self.link0 = self.robot.link("link_0")
        self.link1 = self.robot.link("link_1")
        self.joint1 = self.robot.joint("joint_1")
        self.noise6 = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        self.noise3 = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.noise1 = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
    
        self.values = gtsam.Values()
        gtd.InsertJointAngle(self.values, 0, self.k, 0.0)
        gtd.InsertJointAngle(self.values, 1, self.k, 0.0)
        self.values = self.robot.forwardKinematics(self.values, self.k, "link_0")
        gtd.InsertTorque(self.values, 0, self.k, 0.0)
        gtd.InsertWrench(self.values, 0, 0, self.k, np.zeros(6))
        gtd.InsertWrench(self.values, 1, 0, self.k, np.zeros(6))

    def test_pose_factor(self):
        """Test PoseFactor."""
        factor = gtd.PoseFactor(self.noise6, self.joint1, self.k)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

    def test_twist_factor(self):
        """Test TwistFactor."""
        factor = gtd.TwistFactor(self.noise6, self.joint1, self.k)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

    def test_twistAccel_factor(self):
        """Test TwistAccelFactor."""
        factor = gtd.TwistFactor(self.noise6, self.joint1, self.k)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

    def test_torque_factor(self):
        """Test TorqueFactor."""
        factor = gtd.TorqueFactor(self.noise1, self.joint1, self.k)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

    def test_wrench_eq_factor(self):
        """Test WrenchEquivalenceFactor."""
        factor = gtd.WrenchEquivalenceFactor(self.noise6, self.joint1, self.k)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

    def test_wrench_factor(self):
        """Test WrenchFactor."""
        gravity = np.zeros((3, 1))
        factor = gtd.WrenchFactor(self.noise6, self.link1, [], self.k, gravity)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

    def test_wrench_planar_factor(self):
        """Test WrenchPlanarFactor."""
        planar_axis = np.array([[1], [0], [0]])
        factor = gtd.WrenchPlanarFactor(self.noise3, planar_axis, self.joint1, self.k)
        np.testing.assert_almost_equal(factor.error(self.values), 0)

if __name__ == "__main__":
    unittest.main()
