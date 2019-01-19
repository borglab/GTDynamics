#!/usr/bin/env python
"""
Test Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest

from denavit_hartenberg import LinkParameters
from dh_parameters import PUMA_calibration, RR_calibration
from gtsam import Point3
from utils import GtsamTestCase


class TestLinkParameters(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_constructor(self):
        """Test constructor."""
        link = LinkParameters(0, 0, 0, 0, 'B', 0, Point3(0, 0, 0), [0, 0, 0])
        self.assertIsInstance(link, LinkParameters)


class TestRR(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_link_configuration_home(self):
        """TODO."""
        link_config_home = RR_calibration.link_configuration_home()
        self.assertIsInstance(link_config_home, list)


class TestPuma(GtsamTestCase):
    """Unit tests for DH Puma."""

    def test_link_configuration_home(self):
        """TODO."""
        link_config_home = PUMA_calibration.link_configuration_home()
        self.assertIsInstance(link_config_home, list)


if __name__ == "__main__":
    unittest.main()
