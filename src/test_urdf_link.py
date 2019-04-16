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
from link import F, Link, T, a
from utils import GtsamTestCase
from urdf_parser_py.urdf import URDF

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
    # TODO: create the rotation
    rotation = Rot3()
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

        # create each link with the urdf specs
        self.links = {}
        for link_name in ordered_link_names:
            link = link_dict[link_name]

            # ignore imaginary links
            if link.inertial is None:
                continue

            # TODO: include the whole inertia matrix (6 elements)
            principal_inertia = [link.inertial.inertia.ixx, link.inertial.inertia.iyy, link.inertial.inertia.izz]
            mass = link.inertial.mass
            center_of_mass = compose_pose(link.inertial.origin.rpy, link.inertial.origin.xyz)

            # find the joint attached to the link
            if link_name in robot.parent_map:
                joint_name, parent_name = robot.parent_map[link_name]
                joint = joint_dict[joint_name]
                joint_type = 'R' if joint.joint_type in ['revolute', 'continuous'] else 'P'
                # TODO: special case for fixed joint
                axis = utils.vector(0, 0, 1) if joint.axis is None else utils.vector(joint.axis[0], joint.axis[1], joint.axis[2])
                origin = compose_pose(joint.origin.rpy, joint.origin.xyz)
            else:  # base link
                joint_type = 'R'
                axis = utils.vector(0, 0, 1)
                origin = compose_pose([0, 0, 0], [0, 0, 0])

            # TODO: attach the link to its parent link

            # create the link
            self.links[link_name] = URDF_Link(origin, axis, joint_type, mass, center_of_mass, principal_inertia)

    def test_constructor(self):
        for name, link in self.links.items():
            self.assertIsInstance(link, URDF_Link)


if __name__ == "__main__":
    unittest.main()
