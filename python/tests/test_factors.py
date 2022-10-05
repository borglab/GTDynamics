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
        self.wTp_key = gtd.PoseKey(0, self.k)
        self.wTc_key = gtd.PoseKey(1, self.k)

        ROBOT_FILE = osp.join(gtd.SDF_PATH, "test", "simple_rr.sdf")
        self.robot = gtd.CreateRobotFromFile(str(ROBOT_FILE), "simple_rr_sdf")


class TestJointMeasurementFactor(TestFactors):
    """Test suite for various versions of the JointMeasurementFactor."""
    def test_revolute_joint_measurement_factor(self):
        """Test JointMeasurementFactor for revolute joint."""
        factor = gtd.JointMeasurementFactor(
            gtsam.noiseModel.Isotropic.Sigma(6, 0.1),
            self.robot.joint("joint_1"), np.pi / 4, self.k)

        self.assertIsInstance(factor, gtd.JointMeasurementFactor)


class TestContactEqualityFactor(TestFactors):
    """Test suite for various versions of the ContactEqualityFactor."""
    def test_print(self):
        """Test ContactEqualityFactor print."""
        point_on_link = gtd.PointOnLink(self.robot.link("link_1"),
                                        gtsam.Point3(0, 0, 0))
        factor = gtd.ContactEqualityFactor(
            point_on_link, gtsam.noiseModel.Isotropic.Sigma(3, 0.1), 0, 1)

        self.assertIsInstance(factor, gtd.ContactEqualityFactor)
