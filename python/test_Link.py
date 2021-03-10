"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  testLink.py
 * @brief Test Link class.
 * @author Frank Dellaert, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
"""

# pylint: disable=no-name-in-module, import-error
import unittest

import numpy as np
from gtsam import Point3, Pose3, Rot3
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd


class TestLink(GtsamTestCase):
    def test_params_constructor(self):
        """ Construct a Revolute joint via Parameters and ensure all values are as expected."""
        # load example robot
        simple_rr = gtd.CreateRobotFromFile(
            "sdfs/test/simple_rr.sdf", "simple_rr_sdf")

        l0 = simple_rr.getLinkByName("link_0")
        l1 = simple_rr.getLinkByName("link_1")

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
        np.testing.assert_allclose(l0.inertia(), np.array(
            [[0.05, 0, 0], [0, 0.06, 0], [0, 0, 0.03]]))
        np.testing.assert_allclose(l1.inertia(), np.array(
            [[0.05, 0, 0], [0, 0.06, 0], [0, 0, 0.03]]))


if __name__ == "__main__":
    unittest.main()
