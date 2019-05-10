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
