#!/usr/bin/env python
"""
Tests for FitFourier class.
Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np

import gtsam
import src.utils as utils
from src.fit_fourier import FitFourier
from src.utils import GtsamTestCase


class TestFitFourier(GtsamTestCase):
    """Class to test FitFourier class."""

    def test_decomposition(self):
        """ Test Fourier decomposition. """
        # Create example sequence
        sequence = {}
        for i in range(16):
            x = i * math.pi / 8
            y = math.exp(math.sin(x) + math.cos(x))
            sequence[x] = y

        # Do Fourier Decomposition
        number_of_coefficients = 7
        model = gtsam.noiseModel_Unit.Create(1)
        actual = FitFourier(number_of_coefficients, sequence, model)

        # Check
        expected = utils.vector(1.5661, 1.2717, 1.2717,
                                0.58872, 0., 0.094286, -0.094286)
        np.testing.assert_array_almost_equal(
            expected, actual.parameters, decimal=4)


if __name__ == "__main__":
    unittest.main()
 