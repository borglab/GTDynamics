#!/usr/bin/env python
"""
Fit Fourier class.
Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np

import gtsam
import utils
from fourier_basis import FourierBasis
from gtsam import GaussianFactorGraph
from utils import GtsamTestCase

PARAMETERS_KEY = 0


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
        gfg = GaussianFactorGraph()
        basis = FourierBasis(N)
        for x, y in sequence.items():
            weights_x = basis.calculate_weights(x)
            gfg.add(PARAMETERS_KEY, weights_x, utils.vector(y), model)
        return gfg

    @property
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
