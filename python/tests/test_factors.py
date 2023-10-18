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
import numpy.testing as npt

import gtdynamics as gtd


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


class TestPointGoalFactor(TestFactors):
    """Test suite for the PointGoalFactor."""

    def setUp(self):
        super().setUp()
        self.noise_model = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)

        ROBOT_FILE = osp.join(gtd.URDF_PATH, "test", "simple_urdf.urdf")
        self.robot = gtd.CreateRobotFromFile(str(ROBOT_FILE),
                                             "simple_urdf_urdf")

    def test_error(self):
        """Test RevoluteJointMeasurementFactor."""
        point_com = np.asarray([0, 0, 1])
        goal_point = np.asarray([0, 0, 2])
        factor = gtd.PointGoalFactor(self.wTp_key, self.noise_model, point_com,
                                     goal_point)

        values = gtsam.Values()
        values.insert(self.wTp_key, self.robot.link("l1").bMcom())

        npt.assert_array_almost_equal(np.zeros(3),
                                      factor.unwhitenedError(values))

        values.clear()
        values.insert(self.wTp_key, self.robot.link("l2").bMcom())
        npt.assert_array_almost_equal(np.asarray([0, 0, 2]),
                                      factor.unwhitenedError(values))
