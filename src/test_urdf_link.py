#!/usr/bin/env python
"""
Test Link with Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401, C0412

from __future__ import print_function

import unittest

import numpy as np

import utils
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3, VectorValues
from urdf_link import URDF_Link, read_urdf
from serial_link import SerialLink
from link import F, Link, T, a
from utils import GtsamTestCase

ZERO1 = utils.vector(0)
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


class TestURDFLink(GtsamTestCase):
    """Unit tests for Link in RRR."""

    # The joint screw axis, in the COM frame, is the same for all joints
    AXIS = utils.unit_twist([0, 0, 1], [-1, 0, 0])

    def setUp(self):
        origin = Pose3(Rot3(), Point3(1, 0, 0))
        axis = utils.vector(0, 0, 1)
        center_of_mass = Pose3(Rot3(), Point3(1, 0, 0))
        self.link = URDF_Link(origin, axis, 'R', 1,
                              center_of_mass, np.diag([0, 1 / 6., 1 / 6.]))

    def test_constructor(self):
        """Test constructor."""
        self.assertIsInstance(self.link, URDF_Link)


class TestURDFFetch(GtsamTestCase):
    """Unit tests for urdf link of the fetch robot."""

    def setUp(self):
        # load the urdf file
        file_name = "fetch.urdf"
        self.link_dict = read_urdf(file_name,)
        self.serial_link = SerialLink.from_urdf(self.link_dict, leaf_link_name="r_gripper_finger_link")

    def test_constructor(self):
        self.assertIsInstance(self.serial_link, SerialLink)
        self.assertEqual(len(self.link_dict), 21)
        self.assertEqual(self.link_dict["r_gripper_finger_link"][1], "gripper_link")
        for name, (link, parent_name) in self.link_dict.items():
            self.assertIsInstance(link, URDF_Link)
        self.assertEqual(self.serial_link._links[0].mass, 70.1294)
        # TODO: add more tests here


if __name__ == "__main__":
    unittest.main()
