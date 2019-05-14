#!/usr/bin/env python
"""
Test SerialLink built from URDF file.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401, C0412

from __future__ import print_function

import os
import unittest

import numpy as np
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3, VectorValues

import utils
from link import F, Link, T, a
from serial_link import SerialLink
from urdf_link import URDF_Link, read_urdf
from utils import GtsamTestCase

MY_PATH = os.path.dirname(os.path.realpath(__file__))
URDFS_PATH = os.path.join(MY_PATH, '../urdfs')


class TestURDFLink(GtsamTestCase):
    """Unit tests for Link in RRR."""

    # The joint screw axis, in the COM frame, is the same for all joints
    AXIS = utils.unit_twist([0, 0, 1], [-1, 0, 0])

    def test_constructor(self):
        origin = Pose3(Rot3(), Point3(1, 0, 0))
        axis = utils.vector(0, 0, 1)
        center_of_mass = Pose3(Rot3(), Point3(1, 0, 0))
        link = URDF_Link(origin, axis, 'R', 1,
                         center_of_mass, np.diag([0, 1 / 6., 1 / 6.]))
        self.assertIsInstance(link, URDF_Link)


class TestURDFFetch(GtsamTestCase):
    """Unit tests for urdf link of the fetch robot."""

    def test_load(self):
        # load the urdf file
        file_name = os.path.join(URDFS_PATH, "fetch.urdf")
        link_dict = read_urdf(file_name)
        self.assertEqual(len(link_dict), 21)

        robot = SerialLink.from_urdf(
            link_dict, leaf_link_name="r_gripper_finger_link")
        self.assertIsInstance(robot, SerialLink)
        self.assertEqual(
            link_dict["r_gripper_finger_link"][1], "gripper_link")
        for link_info in link_dict.values():
            self.assertIsInstance(link_info[0], URDF_Link)
        self.assertEqual(robot.link(0).mass, 70.1294)
        self.assertEqual(len(robot._links), 11)


class TestURDFFanuc(GtsamTestCase):
    """Unit tests for the Fanuc URDF."""

    def test_load(self):
        # load the urdf file
        file_name = os.path.join(URDFS_PATH, "fanuc_lrmate200id.urdf")
        link_dict = read_urdf(file_name)
        self.assertEqual(len(link_dict), 6)

        robot = SerialLink.from_urdf(link_dict, leaf_link_name="Part6")
        self.assertEqual(robot.link(0).mass, 4.85331)
        # print(robot.link_frames(utils.vector(0,np.pi/2, 0, 0, 0, 0, 0)))
        # TODO(frank): fix the mess
        self.gtsamAssertEquals(robot.link(1).A(0), Pose3(
            Rot3(), Point3(0.105, 0.09, 0.16859)))
        self.gtsamAssertEquals(robot.link(1).A(np.pi/2), Pose3(
            Rot3.Rz(np.pi/2), Point3(0.105, 0.09, 0.16859)))


if __name__ == "__main__":
    unittest.main()
