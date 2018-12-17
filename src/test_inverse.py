#!/usr/bin/env python
"""
Test inverse dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import unittest

import numpy as np
from gtsam import Point3, Pose3, Rot3
from utils import vector, unit_twist, compose, spatial_velocity, FetchTestCase


def forward_backward_R():
    # Calculate torque for R manipulator with forward_backward
    return vector(0)

def inverse_factor_graph_way_R():
    # Setup factor graph
    # Optimize it
    # return torque
    return vector(0)

class TestInverseDynamics(unittest.TestCase):
    """Unit tests for FetchArm class."""

    def gtsamAssertEquals(self, actual, expected, tol=1e-2):
        """Helper function that prints out actual and expected if not equal."""
        # TODO: make a unittest.TestCase class hat has this in GTSAM
        equal = actual.equals(expected, tol)
        if not equal:
            raise self.failureException(
                "Values are not equal:\n{}!={}".format(actual, expected))

    def setUp(self):
        """setup."""
        pass

    def test_R_inverse_dynamics(self):
        """Try a simple R robot."""
        expected_torques = forward_backward_R()
        # Call a function with appropriate arguments to co compute them 
        actual_torques = inverse_factor_graph_way_R()
        np.testing.assert_array_almost_equal(actual_torques, expected_torques)


if __name__ == "__main__":
    unittest.main()
