#!/usr/bin/env python
"""
Test Gait Optimizer class.
Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

from math import sin, cos
import unittest

import numpy as np

from utils import GtsamTestCase


class Basis(object):
    """Caller for basis decompositions"""
    
    def __init__(self):
        """Constructor"""
        pass

    def weightMatrix(self, x):
        """Calculate the weight matrix for a given basis"""
        return self.calculateWeights(x)


class FourierBasis(Basis):
    """ Class for Fourier analysis """

    def __init__(self, N):
        """ Constructor
            Keyword arguments:
                N -- number of values to evaluate
        """
        Basis.__init__(self)
        self._N = N
    
    def calculateWeights(self, x):
        """Evaluate Fourier weights at a given x"""
        weights = []
        weights.append(1)
        for n in range(1, self._N / 2 + 1):
            weights.append(sin(n * x))
            weights.append(cos(n * x))
        return np.array(weights)

    class EvaluationFunctor(object):
        
        def __init__(self, N, x):
            """Constructor"""
            FourierBasis.__init__(N)
            self._weights = self.weightMatrix(x)
            
        def __call__(self, parameters):
            """Do Fourier synthesis at a given x"""
            return self._weights * parameters



class DynamicsModel(object):
    """Specifies the dynamics parameters."""

    def __init__(self):
        """Constructor."""
        pass


class FourierCoefficients(object):
    """Set of Fourier coefficients."""

    def __init__(self, N):
        """ Constructor.
            Keyword arguments:
                N -- number of weights to be evaluated
        """
        self._N = N

    def calculateWeights(self):
        """ Evaluate the real Fourier weights """
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
        fourierWeights = 5
        self.optimizer = GaitOptimizer(human)
        self.coefficients = FourierCoefficients(fourierWeights)

    def test_run(self):
        """Test the whole enchilada."""
        desired_velocity = 3
        stance_fraction = .4
        self.coefficients.calculateWeights()
        actual = self.optimizer.run(desired_velocity, stance_fraction)
        self.assertEqual(actual.shape, (7,))

    def test_functors(self):
        """Testing to make sure the functor system works as expected"""
        numberOfParameters = 7
        x = 1
        basis = FourierBasis.EvaluationFunctor(numberOfParameters, x)
        self.assertIsNotNone(basis)
        val = basis(np.array([1, 2, 3]))
        print(val)
        

if __name__ == "__main__":
    unittest.main()
