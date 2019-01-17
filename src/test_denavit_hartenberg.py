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
import utils
from denavit_hartenberg import DenavitHartenberg, LinkParameters
from dh_parameters import PUMA_calibration, RR_calibration
from gtsam import Point3, Pose3, Rot3, symbol
from utils import GtsamTestCase, vector


class TestLinkParameters(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_constructor(self):
        link_parameters = LinkParameters(
            0, 0, 0, 0, 'B', 0, Point3(0, 0, 0), [0, 0, 0])
        # TODO


class TestRR(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_some_method(self):
        """TODO."""
        pass


class TestPuma(GtsamTestCase):
    """Unit tests for DH Puma."""

    def test_some_method(self):
        """TODO."""
        pass


if __name__ == "__main__":
    unittest.main()
