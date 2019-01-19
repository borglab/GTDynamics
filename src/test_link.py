#!/usr/bin/env python
"""
Test Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest
import gtsam

from link import Link
from utils import GtsamTestCase


class TestLink(GtsamTestCase):
    """Unit tests for DH RR."""

    def test_constructor(self):
        """Test constructor."""
        link = Link(0, 0, 0, 0, 'B', 0, gtsam.Point3(0, 0, 0), [0, 0, 0])
        self.assertIsInstance(link, Link)


if __name__ == "__main__":
    unittest.main()
