#!/usr/bin/env python
"""
    Dynamics model file
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import urdf_link
import utils
from utils import GtsamTestCase


class DynamicsModel(object):
    """Specifies the dynamics parameters."""

    def __init__(self, urdfFilePath=None):
        """Constructor."""

        if urdfFilePath:
            self.read_urdf(urdfFilePath)

        self.end_effector_positions = [0. for _ in range(self.number_of_legs)]

    def read_urdf(self, filePath):
        """ Read in a URDF and extract.
            Keyword arguments:
                filePath - the URDF (that is, XML) file to open
            Returns an XML tree of the URDF.
        """
        self.urdf_model = urdf_link.read_urdf(filePath)


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
