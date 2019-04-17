#!/usr/bin/env python
"""
Test Link with Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401, C0412

from __future__ import print_function

import unittest

import utils
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3, VectorValues
from urdf_link import URDF_Link
from serial_link import SerialLink
from link import F, Link, T, a
from utils import GtsamTestCase
from urdf_parser_py.urdf import URDF
import numpy as np

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
                              center_of_mass, [0, 1 / 6., 1 / 6.])

    def test_constructor(self):
        """Test constructor."""
        self.assertIsInstance(self.link, URDF_Link)


def compose_pose(rpy, xyz):
    # TODO: check if the rotation is correct
    rotation = Rot3.rpy(rpy[0], rpy[1], rpy[2])
    point = Point3(xyz[0], xyz[1], xyz[2])
    return Pose3(rotation, point)


class TestURDFFetch(GtsamTestCase):
    """Unit tests for urdf link of the fetch robot."""

    def setUp(self):
        # load the urdf file
        file_name = "fetch.urdf"
        robot = URDF.from_xml_file(file_name)
        link_dict = {link.name: link for link in robot.links}
        joint_dict = {joint.name: joint for joint in robot.joints}

        # sort the links in topological order
        link_names = [link.name for link in robot.links]
        ordered_link_names = [link_name for link_name in link_names if link_name not in robot.parent_map]
        while len(ordered_link_names) < len(link_names):
            for link_name in link_names:
                if (link_name not in ordered_link_names) and robot.parent_map[link_name][1] in ordered_link_names:
                    ordered_link_names.append(link_name)

        # TODO: build the links in a tree structure
        # The serial link class only supports serial connection (one child policy)
        # However, there may be several children connected to the parent in the fetch robot, and the links form a tree structure
        # Currently, I only choose one path in the tree structure, and build a serial link

        link_name = 'r_gripper_finger_link'  # leaf link
        path_links = [link_name]
        while link_name in robot.parent_map:
            parent_name = robot.parent_map[link_name][1]
            path_links = [parent_name] + path_links
            link_name = parent_name

        # create each link with the urdf specs
        fetch_calibration_urdf = []
        for link_name in path_links:
            link = link_dict[link_name]

            # ignore imaginary links
            if link.inertial is None:
                continue

            inertia = link.inertial.inertia
            inertia_matrix = np.array(
                [[inertia.ixx, inertia.ixy, inertia.ixz], [inertia.ixy, inertia.iyy, inertia.iyz], [inertia.ixz, inertia.iyz, inertia.izz]])

            mass = link.inertial.mass
            center_of_mass = compose_pose(link.inertial.origin.rpy, link.inertial.origin.xyz)

            # find the joint attached to the link
            if link_name in robot.parent_map:
                joint_name, parent_name = robot.parent_map[link_name]
                joint = joint_dict[joint_name]
                joint_type = 'R' if joint.joint_type in ['revolute', 'continuous'] else 'P'
                # TODO: special case for fixed joint, combine both links as the same link, or set the joint angle to be fixed
                axis = utils.vector(0, 0, 1) if joint.axis is None else utils.vector(joint.axis[0], joint.axis[1], joint.axis[2])
                origin = compose_pose(joint.origin.rpy, joint.origin.xyz)
            else:  # base link
                joint_type = 'R'
                axis = utils.vector(0, 0, 1)
                origin = compose_pose([0, 0, 0], [0, 0, 0])

            # create the link
            fetch_calibration_urdf.append(URDF_Link(origin, axis, joint_type, mass, center_of_mass, inertia_matrix))
            self.robot = SerialLink(fetch_calibration_urdf)

    def test_constructor(self):
        self.assertIsInstance(self.robot, SerialLink)


if __name__ == "__main__":
    unittest.main()
