"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_link.py
 * @brief Test Link class.
 * @author Frank Dellaert, Varun Agrawal, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
"""

# pylint: disable=no-name-in-module, import-error, no-member
import os.path as osp
import unittest

import numpy as np
from gtsam import Point3, Pose3, Rot3
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd


class TestLink(GtsamTestCase):
    """Tests for the Link class."""
    def setUp(self):
        """Set up the fixtures."""
        # load example robot
        SDF_PATH = osp.join(osp.dirname(osp.realpath(__file__)), "..", "..",
                            "sdfs")
        self.simple_rr = gtd.CreateRobotFromFile(
            osp.join(SDF_PATH, "test", "simple_rr.sdf"), "simple_rr_sdf")

    def test_params_constructor(self):
        """Check the links in the simple RR robot."""

        l0 = self.simple_rr.link("link_0")
        l1 = self.simple_rr.link("link_1")

        # Both link frames are defined in the world frame.
        self.gtsamAssertEquals(l0.wTl(), Pose3())
        self.gtsamAssertEquals(l1.wTl(), Pose3())

        # Verify center of mass defined in the link frame is correct.
        self.gtsamAssertEquals(l0.lTcom(), Pose3(Rot3(), Point3(0, 0, 0.1)))
        self.gtsamAssertEquals(l1.lTcom(), Pose3(Rot3(), Point3(0, 0, 0.5)))

        # Verify center of mass defined in the world frame is correct.
        self.gtsamAssertEquals(l0.wTcom(), Pose3(Rot3(), Point3(0, 0, 0.1)))
        self.gtsamAssertEquals(l1.wTcom(), Pose3(Rot3(), Point3(0, 0, 0.5)))

        # Verify that mass is correct.
        self.assertEqual(l0.mass(), 0.01)
        self.assertEqual(l1.mass(), 0.01)

        # Verify that inertia elements are correct.
        np.testing.assert_allclose(
            l0.inertia(), np.array([[0.05, 0, 0], [0, 0.06, 0], [0, 0, 0.03]]))
        np.testing.assert_allclose(
            l1.inertia(), np.array([[0.05, 0, 0], [0, 0.06, 0], [0, 0, 0.03]]))

    def test_get_joints(self):
        """Test the joints method."""
        l0 = self.simple_rr.link("link_0")
        l1 = self.simple_rr.link("link_1")

        self.assertIsInstance(l0.joints(), list)
        self.assertIsInstance(l0.joints()[0], gtd.Joint)

        self.assertEqual(len(l0.joints()), 1)
        self.assertEqual(len(l1.joints()), 2)


if __name__ == "__main__":
    unittest.main()
