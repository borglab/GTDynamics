#!/usr/bin/env python
"""
    Fourier decomposition class.
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np

import utils
from dynamics_model import DynamicsModel
from utils import GtsamTestCase



class FourierDecomposition(object):
    """Obtain Fourier coefficients for the given model."""

    def __init__(self, numberFourierCoefficients):
        """Constructor."""
        self.number_fourier_coefficients = numberFourierCoefficients

    def get_coefficients(self, model):
        """ Obtain the Fourier coefficients for the model.
            Keyword arguments:
                model - the model for which Fourier coefficients will be found
            Returns a dictionary of numpy arrays of Fourier coefficients.
                Each key is the index of a particular leg.
        """
        return {str(i): utils.vector() for i in range(self.number_fourier_coefficients)}
