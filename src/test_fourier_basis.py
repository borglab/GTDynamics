#!/usr/bin/env python
"""
    Tests for FourierBasis class.
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np

import src.utils as utils
from src.fourier_basis import FourierBasis
from src.utils import GtsamTestCase


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
