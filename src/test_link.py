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
from link import Link, T, a
from utils import GtsamTestCase

ZERO1 = utils.vector(0)
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


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
        """Test factors for forward dynamics, middle link of stationary RRR example."""

        # Create stationary state
        v1, v2, v3 = 0, 0, 0
        twist_1 = ZERO6
        twist_2 = ZERO6

        # Create all factors
        jTi = Pose3(Rot3(), Point3(-2, 0, 0))
        factors = self.link.forward_factors(2, v2, twist_2, jTi)
        self.assertIsInstance(factors, GaussianFactorGraph)
        self.assertEqual(factors.size(), 1)

        # Create ground truth values
        ground_truth = VectorValues()
        ground_truth.insert(a(1), ZERO1)
        ground_truth.insert(a(2), ZERO1)
        ground_truth.insert(T(1), ZERO6)
        ground_truth.insert(T(2), ZERO6)

        # Assert that error is zero for ground-truth
        self.assertAlmostEqual(factors.error(ground_truth), 0)


if __name__ == "__main__":
    unittest.main()
