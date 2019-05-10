#!/usr/bin/env python

"""
    Tests for DynamicsModel class
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import utils
from dynamics_model import DynamicsModel
from utils import GtsamTestCase


class TestDynamicsModel(GtsamTestCase):

    def test_empty_model(self):
        """Test creating an empty DynamicsModel object."""
        model_empty = DynamicsModel()
        self.assertIsInstance(model_empty, DynamicsModel)

    def test_loading_urdf(self):
        """Test reading in a URDF file."""
        model = DynamicsModel("src/turtle.urdf")
        self.assertIsInstance(model, DynamicsModel)
        self.assertIsInstance(model.urdf_model, dict)


if __name__ == "__main__":
    unittest.main()
