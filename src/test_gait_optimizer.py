#!/usr/bin/env python
"""
    Test Gait Optimizer class.
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import pdb
import unittest
import xml.etree.ElementTree as ET
from collections import namedtuple

import numpy as np

import gtsam
import utils
from gtsam import Point3, Pose3, Rot3
from gtsam_unstable import PaddleFactor
from urdf_link import URDF_Link
from utils import GtsamTestCase


class DynamicsModel(object):
    """Specifies the dynamics parameters."""

    def __init__(self, urdfFilePath=None):
        """Constructor."""

        if urdfFilePath:
            self.read_urdf(urdfFilePath)

        self.end_effector_positions = [0. for _ in range(self.number_of_legs)]

    def read_urdf(self, filePath):
        """ Read in a URDF and extract .
            Keyword arguments:
                filePath - the URDF (that is, XML) file to open
            Returns an XML tree of the URDF.
        """

        # TODO: use Yetong's read_urdf() in urdf_link.py instead of this code
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

    def __init__(self, model, args):
        """Constructor."""
        self._args = args

    def create_graph(self):
        """ Build the factor graph.
            Return NonlinearFactorGraph
        """
        graph = gtsam.NonlinearFactorGraph()

        # t is time period (seconds)
        period = 1.
        omega = 2 * math.pi / period

        for leg in range(4):
            for k in range(self._args.num_samples):
                t = k * period / self._args.num_samples

                paddle_factor = PaddleFactor(leg, t, omega, self._args.length,
                                             self._args.alpha, self._args.beta)
                #tau = k * 2 * math.pi / self._args.num_samples
                # paddle_factor = PaddleFactor(leg, tau, self._args.length,
                #                       self._args.alpha, self._args.beta)
                graph.add(paddle_factor)
        return graph

    def run(self):
        """ Find the best gait.

            Returns a set of Fourier coefficients.
        """
        # Create the graph.
        graph = self.create_graph()

        # Create initial values.
        initial = gtsam.Values()
        # initial.insert(0, utils.vector(1.5661, 1.2717, 1.2717))
        # initial.insert(1, utils.vector(1.5661, 1.2717, 1.2717))
        # initial.insert(2, utils.vector(1.5661, 1.2717, 1.2717))
        # initial.insert(3, utils.vector(1.5661, 1.2717, 1.2717))
        initial.insert(0, utils.vector(1.0, 1.0, 1.0))
        initial.insert(1, utils.vector(1.0, 1.0, 1.0))
        initial.insert(2, utils.vector(1.0, 1.0, 1.0))
        initial.insert(3, utils.vector(1.0, 1.0, 1.0))

        # Call GTSAM optimizer
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
        result = optimizer.optimize()

        return result

    def evaluate(self, model, fourier_coefficients):
        """ Evaluate the Fourier coefficients on the given model.
            Keyword arguments:
                model - the model whose gait is being optimized
                fourier_coefficients - dictionary of lists of Fourier coefficients describing a gait
            Returns a value indicating how good the gait is.
        """
        epsilon = 1e-4

        depth_of_legs = model.end_effector_positions

        return 0.


class TestGaitOptimizer(GtsamTestCase):
    """Unit tests for GaitOptimizer."""

    def setUp(self):
        """Create common variables for all tests."""
        model = DynamicsModel("src/turtle.urdf")
        Args = namedtuple('Args', ['num_samples', 'length', 'alpha', 'beta'])
        args = Args(8, 1.0, 1.0, 1.0)
        self.optimizer = GaitOptimizer(model, args)
        self.numberFourierCoefficients = 3
        # Ensure number of Fourier coefficients is an odd number.
        self.assert_(self.numberFourierCoefficients % 2 == 1)

    def test_example_urdf_link(self):
        """Test ability to create URDF links."""
        test_origin = Pose3(Rot3(), Point3(0, 0, 0))
        test_axis = np.array([0, 0, 0])
        test_joint_type = 'R'
        test_mass = 0.01
        test_center_of_mass = Pose3(Rot3(), Point3(0, 0, 0))
        test_inertia = np.array([0., 0., 0])
        test_link = URDF_Link(origin=test_origin, axis=test_axis, joint_type=test_joint_type,
                              mass=test_mass, center_of_mass=test_center_of_mass, inertia=test_inertia)
        self.assertIsInstance(test_link, URDF_Link)

    def test_dynamics_model(self):
        """Test the DynamicsModel object."""
        # Instantiate empty.braces
        model_empty = DynamicsModel()

        # Instantiate with URDF.
        model = DynamicsModel("src/turtle.urdf")
        # Read in URDF

    def test_fourier_coefficients(self):
        """Test getting the Fourier coefficients from a model."""
        model = DynamicsModel()
        fourier = FourierDecomposition(self.numberFourierCoefficients)
        coefficients = fourier.get_coefficients(model)

        self.assertIsInstance(coefficients, dict)
        for value in coefficients.values():
            self.assertIsInstance(value, type(utils.vector()))

    def test_create_graph(self):
        """Check that we can create a nice factor graph."""
        graph = self.optimizer.create_graph()
        self.assertIsInstance(graph, gtsam.NonlinearFactorGraph)
        self.assertEqual(32, graph.size())

    # @unittest.skip("Not yet")
    def test_run(self):
        """Test the gait optimization using the Fourier coefficients."""
        actual = self.optimizer.run()
        self.assertIsInstance(actual, gtsam.Values)

        # We expect four joint trajectories, represented by their Fourier
        # coefficients, one for each leg.
        expected = gtsam.Values()
        expected.insert(0, utils.vector(1.5661, 1.2717, 1.2717))
        expected.insert(1, utils.vector(1.5661, 1.2717, 1.2717))
        expected.insert(2, utils.vector(1.5661, 1.2717, 1.2717))
        expected.insert(3, utils.vector(1.5661, 1.2717, 1.2717))
        keys = expected.keys()
        for i in range(keys.size()):
            key = keys.at(i)
            print(key)
            np.testing.assert_array_almost_equal(
                expected.atVector(key), actual.atVector(key))


if __name__ == "__main__":
    unittest.main()
