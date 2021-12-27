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

import gtdynamics as gtd
import gtsam
import numpy as np


class TestFactors(unittest.TestCase):
    """
    Base class for testing various factors.
    Provides needed fixtures and common functions.
    """
    def setUp(self):
        self.k = 0
        self.wTp_key = gtd.internal.PoseKey(0, self.k).key()
        self.wTc_key = gtd.internal.PoseKey(1, self.k).key()

        ROBOT_FILE = osp.join(gtd.SDF_PATH, "test", "simple_rr.sdf")
        self.robot = gtd.CreateRobotFromFile(str(ROBOT_FILE), "simple_rr_sdf")


class TestJointMeasurementFactor(TestFactors):
    """Test suite for various versions of the JointMeasurementFactor."""
    def test_revolute_joint_measurement_factor(self):
        """Test RevoluteJointMeasurementFactor."""
        factor = gtd.JointMeasurementFactor(
            gtsam.noiseModel.Isotropic.Sigma(6, 0.1),
            self.robot.joint("joint_1"), np.pi / 4, self.k)

        self.assertIsInstance(factor, gtd.JointMeasurementFactor)

