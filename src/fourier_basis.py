#!/usr/bin/env python
"""
Fourier basis class.
Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np

import utils
from utils import GtsamTestCase


class FourierBasis(object):
    """Fourier basis."""

    def __init__(self, N):
        """ Constructor.
            Keyword arguments:
                N -- number of weights to be evaluated
        """
        self._N = N

    def calculate_weights(self, x):
        """ Evaluate the real Fourier weights """
        weights = np.empty((1, self._N), np.float)
        weights[0, 0] = 1
        for i in range(self._N/2):
            n = i + 1
            weights[0, 2 * i + 1] = math.sin(n * x)
            weights[0, 2 * i + 2] = math.cos(n * x)
        return weights


class TestFourierBasis(GtsamTestCase):
    """Test Fourier basis."""

    def test_calculate_weights(self):
        """Test calculating the weights."""
        basis = FourierBasis(3)
        weights = basis.calculate_weights(math.pi)
        self.assertEqual(weights.shape, (1, 3))
        expected = utils.vector(1, 0, -1).reshape((1, 3))
        np.testing.assert_array_almost_equal(expected, weights)


if __name__ == "__main__":
    unittest.main()
