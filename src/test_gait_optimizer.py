#!/usr/bin/env python
"""
    Test Gait Optimizer class.
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np

import gtsam

import utils
from utils import GtsamTestCase


class DynamicsModel(object):
    """Specifies the dynamics parameters."""

    def __init__(self):
        """Constructor."""
        pass


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


class GaitOptimizer(object):
    """Class that optimizes gaits for a particular walking robot."""

    def __init__(self):
        """Constructor."""
        pass

    def evaluate(self, model, fourier_coefficients):
        """ Evaluate the Fourier coefficients on the given model.
            Keyword arguments:
                model - the model whose gait is being optimized
                fourier_coefficients - dictionary of lists of Fourier coefficients describing a gait
            Returns a value indicating how good the gait is.
        """
        return 0.


class TestGaitOptimizer(GtsamTestCase):
    """Unit tests for GaitOptimizer."""

    def setUp(self):
        self.numberFourierCoefficients = 3
        expected_coeffs = {"0": utils.vector(1.5661, 1.2717, 1.2717),
                           "1": utils.vector(1.5661, 1.2717, 1.2717)}

        # Ensure number of Fourier coefficients is an odd number.
        self.assert_(self.numberFourierCoefficients % 2 == 1)

    def test_fourier_coefficients(self):
        """Test getting the Fourier coefficients from a model."""
        model = DynamicsModel()
        fourier = FourierDecomposition(self.numberFourierCoefficients)
        coefficients = fourier.get_coefficients(model)

        self.assertIsInstance(coefficients, dict)
        for value in coefficients.values():
            self.assertIsInstance(value, type(utils.vector()))

    def test_optimizer(self):
        """Test the gait optimization using the Fourier coefficients."""
        model = DynamicsModel()
        fourier = FourierDecomposition(self.numberFourierCoefficients)
        optimizer = GaitOptimizer()
        coefficients = fourier.get_coefficients(model)

        coefficient_goodness = optimizer.evaluate(model, coefficients)

        self.assertIsInstance(coefficient_goodness, float)


if __name__ == "__main__":
    unittest.main()
