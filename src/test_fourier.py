#!/usr/bin/env python
"""
Test Fourier Decomposition class.
Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

import math
import unittest

import numpy as np

import gtsam
import utils

from utils import GtsamTestCase

PARAMETERS_KEY = 0
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
        weights[0,0] = 1
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


class FitFourier(object):
    """Class that does Fourier Decomposition via least squares."""

    def __init__(self, N, sequence, model):
        """ Constructor.
            Keyword arguments:
                N -- number of samples
                sequence -- signal to be decomposed
                model -- noise model for least squares optimization
        """
        gfg = self.linear_graph(N, sequence, model)
        solution = gfg.optimize()
        self._parameters = solution.at(PARAMETERS_KEY)

    def linear_graph(self, N, sequence, model):
        """Create linear FG from Sequence."""
        gfg = gtsam.GaussianFactorGraph()
        basis = FourierBasis(N)
        for x, y in sequence.items():
            weights_x = basis.calculate_weights(x)
            gfg.add(PARAMETERS_KEY, weights_x, utils.vector(y), model)
        return gfg

    def parameters(self):
        """Return Fourier coefficients."""
        return self._parameters


class TestFourier(GtsamTestCase):

    def test_decomposition(self):
        """ Test Fourier decomposition. """
        # Create example sequence
        sequence = {}
        for i in range(16):
            x = i * math.pi / 8
            y = math.exp(math.sin(x) + math.cos(x))
            sequence[x] = y

        # Do Fourier Decomposition
        model = gtsam.noiseModel_Unit.Create(1)
        actual = FitFourier(3, sequence, model)

        # Check
        expected = utils.vector(1.5661, 1.2717, 1.2717)
        np.testing.assert_array_almost_equal(expected, actual.parameters(), decimal=4)


if __name__ == "__main__":
    unittest.main()
