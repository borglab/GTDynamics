#!/usr/bin/env python
"""
Test Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401, C0412

from __future__ import print_function

import unittest

import utils
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3, VectorValues
from link import Link, V
from utils import GtsamTestCase


class TestLink(GtsamTestCase):
    """Unit tests for Link in RRR."""

    # The joint screw axis, in the COM frame, is the same for all joints
    AXIS = utils.unit_twist([0, 0, 1], [-1, 0, 0])

    def setUp(self):
        self.link = Link(0, 0, 2, 0, 'R', 1,
                         Point3(-1, 0, 0), [0, 1/6., 1/6.])

    def test_constructor(self):
        """Test constructor."""
        self.assertIsInstance(self.link, Link)

    def test_forward_factors(self):
        """Test factors for forward dynamics, middle link of RRR example."""

        # Create ground truth situation for RRR
        v1, v2, v3 = 3, 2, 4

        # Create all factors
        jTi = Pose3(Rot3(), Point3(-2, 0, 0))
        factors = self.link.forward_factors(2, jTi, v2)
        self.assertIsInstance(factors, GaussianFactorGraph)
        self.assertEqual(factors.size(), 1)

        # Create ground truth Values
        ground_truth = VectorValues()
        ground_truth.insert(V(1), v1 * self.AXIS)
        ground_truth.insert(V(2),  utils.unit_twist(
            [0, 0, v1], [-3, 0, 0]) + utils.unit_twist([0, 0, v2], [-1, 0, 0]))

        # Assert that error is zero for ground-truth
        self.assertAlmostEqual(factors.error(ground_truth), 0)


if __name__ == "__main__":
    unittest.main()
