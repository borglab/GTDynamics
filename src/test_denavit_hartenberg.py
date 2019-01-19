#!/usr/bin/env python
"""
Test Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

from denavit_hartenberg import Link
from dh_parameters import PUMA_calibration, RR_calibration
from gtsam import Point3, Pose3, Rot3
from utils import GtsamTestCase


class TestLink(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_constructor(self):
        """Test constructor."""
        link = Link(0, 0, 0, 0, 'B', 0, Point3(0, 0, 0), [0, 0, 0])
        self.assertIsInstance(link, Link)


class TestRR(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_link_frames(self):
        """Try link_frames."""
        configuration = RR_calibration.link_frames()
        self.assertIsInstance(configuration, list)
        self.assertEquals(len(configuration), 2)
        self.gtsamAssertEquals(
            configuration[0], Pose3(Rot3(), Point3(2, 0, 0)))
        self.gtsamAssertEquals(
            configuration[1], Pose3(Rot3(), Point3(4, 0, 0)))


class TestPuma(GtsamTestCase):
    """Unit tests for DH Puma."""

    def test_link_frames(self):
        """Try link_frames."""
        configuration = PUMA_calibration.link_frames()
        self.assertIsInstance(configuration, list)
        self.assertEquals(len(configuration), 6)
        self.gtsamAssertEquals(
            configuration[0], Pose3(Rot3.Rx(math.radians(90)), Point3(0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
