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
from urdf_link import URDF_Link
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
                         center_of_mass, np.diag([0, 1/6., 1/6.]))

    def test_constructor(self):
        """Test constructor."""
        self.assertIsInstance(self.link, URDF_Link)


if __name__ == "__main__":
    unittest.main()
