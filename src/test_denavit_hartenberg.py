#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest
import numpy as np
import gtsam
from gtsam import Point3, Pose3, Rot3, symbol

import utils
from utils import vector, GtsamTestCase

from denavit_hartenberg import DenavitHartenberg, LinkParameters


class TestLinkParameters(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_constructor(self):
        link_parameters = LinkParameters(
            0, 0, 0, 0, 'B', 0, Point3(0, 0, 0), [0, 0, 0])
        # TODO


class TestRR(GtsamTestCase):
    """Unit tests for DH RR."""

    def setUp(self):
        """setup."""
        # Denavit-Hartenberg parameters for RR manipulator
        self.link_parameters = [
            LinkParameters(0, 0, 0, 0, 'B', 0, Point3(0, 0, 0), [0, 0, 0]),
            LinkParameters(0, 0, 2, 0, 'R', 1, Point3(
                1, 0, 0), [0, 1/6., 1/6.]),
            LinkParameters(0, 0, 2, 0, 'R', 1, Point3(
                1, 0, 0), [0, 1/6., 1/6.]),
            LinkParameters(0, 90, 0, 0, 'G', 0, Point3(0, 0, 0), [0, 0, 0])
        ]

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        RR_calibration = DenavitHartenberg(self.link_parameters, 2)
        # TODO


class TestPuma(GtsamTestCase):
    """Unit tests for DH Puma."""

    def setUp(self):
        """setup."""
        # Denavit-Hartenberg parameters for PUMA manipulator
        self.link_parameters = [
            LinkParameters(0, 0, 0, 0, 'B', 0,
                           Point3(0, 0, 0), [0, 0, 0]),
            LinkParameters(1, 5, 0, -90, 'R', 0,
                           Point3(0, 0, 0), [0, 0, 0.35]),
            LinkParameters(0.2435, 10, 0.4318, 0, 'R', 17.40, Point3(
                0.068, 0.006, -0.016), [0.130, 0.524, 0.539]),
            LinkParameters(-0.0934, 15, 0.0203, -90, 'R', 4.80,
                           Point3(0, -0.070, 0.014), [0.066, 0.0125, 0.086]),
            LinkParameters(0.4331, 20, 0, 90, 'R', 0.82, Point3(
                0, 0, -0.019), [0.0018, 0.0018, 0.00130]),
            LinkParameters(0, 25, 0, -90, 'R', 0.34,
                           Point3(0, 0, 0), [0.00030, 0.00030, 0.00040]),
            LinkParameters(0.2000, 30, 0, 90, 'R', 0.09, Point3(
                0, 0, 0.032), [0.00015, 0.00015, 0.00004]),
            LinkParameters(0, 90, 0, 0, 'G', 0,
                           Point3(0, 0, 0), [0, 0, 0]),
        ]

    def test_PUMA_forward_dynamics(self):
        """Try a PUMA robot."""
        PUMA_calibration = DenavitHartenberg(self.link_parameters, 6)
        # TODO


if __name__ == "__main__":
    unittest.main()
