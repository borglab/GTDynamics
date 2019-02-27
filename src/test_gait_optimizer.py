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
from test_fourier import FitFourier
from utils import GtsamTestCase


class DynamicsModel(object):
    """Specifies the dynamics parameters."""

    def __init__(self):
        """Constructor."""
        pass
    

class GaitOptimizer(object):
    """Class that optimizes gaits for a particular walking robot."""

    def __init__(self, model):
        """Constructor."""
        pass

    def run(self, desired_velocity, stance_fraction):
        """ Compute coefficients for optimal joint trajectories.
            Keyword arguments:
                desired_velocity (m/s) -- the velocity which is desired
                stance_fraction (%) -- fraction of interval for which foot is in stance
        """
        # Build factor graph
        # optimize
        # return result
        pass


class TestGaitOptimizer(GtsamTestCase):
    """Unit tests for GaitOptimizer."""

    def setUp(self):
        """Create optimizer."""
        human = DynamicsModel()
        self.optimizer = GaitOptimizer(human)

    def test_fourier(self):
        """Test the Fourier decomposition."""
        fourierWeights = 7
        
        # Create example sequence
        sequence = {}
        for i in range(16):
            x = i * math.pi / 8
            y = math.exp(math.sin(x) + math.cos(x))
            sequence[x] = y

        # Do Fourier Decomposition
        model = gtsam.noiseModel_Unit.Create(1)
        ff = FitFourier(fourierWeights, sequence, model)
        self.assertEqual(ff.parameters().shape, (fourierWeights,))
        
    def test_run(self):
        """Test the whole enchilada."""
        
        desired_velocity = 3
        stance_fraction = .4
        actual = self.optimizer.run(desired_velocity, stance_fraction)


        

if __name__ == "__main__":
    unittest.main()
