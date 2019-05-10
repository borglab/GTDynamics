"""
    Gait Optimizer class.
    Author: Frank Dellaert and Stephen Eick
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math

import gtsam
import src.utils as utils
from gtsam_unstable import PaddleFactor


class GaitOptimizer(object):
    """Class that optimizes gaits for a particular walking robot."""

    def __init__(self, model, args):
        """Constructor."""
        self._model = model
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
                graph.add(paddle_factor)
        return graph

    def run(self, guess=1):
        """ Find the best gait.

            Returns a set of Fourier coefficients.
        """
        # Create the graph.
        graph = self.create_graph()

        def initial_guess_1(initial):
            """Initial guess 1 for optimization."""
            initial.insert(0, utils.vector(1.0, 1.0, 1.0))
            initial.insert(1, utils.vector(1.0, 1.0, 1.0))
            initial.insert(2, utils.vector(1.0, 1.0, 1.0))
            initial.insert(3, utils.vector(1.0, 1.0, 1.0))
            return initial

        def initial_guess_2(initial):
            """Initial guess 2 for optimization."""
            initial.insert(0, utils.vector(1.5661, 1.2717, 1.2717))
            initial.insert(1, utils.vector(1.5661, 1.2717, 1.2717))
            initial.insert(2, utils.vector(1.5661, 1.2717, 1.2717))
            initial.insert(3, utils.vector(1.5661, 1.2717, 1.2717))
            return initial

        initial_guesses = {
            1: initial_guess_1,
            2: initial_guess_2,
        }

        # Create initial values.
        initial = gtsam.Values()
        initial = initial_guesses[guess](initial)

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
