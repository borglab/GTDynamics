#!/usr/bin/env python
"""
Test inverse dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest

import numpy as np
from utils import vector, GtsamTestCase


def forward_backward_R():
    """Calculate torque for R manipulator with forward_backward"""
    return vector(0)


def inverse_factor_graph_way_R():
    """TODO(Mandy): docstring."""
    # Setup factor graph
    # Optimize it
    # return torque
    return vector(0)


class TestInverseDynamics(GtsamTestCase):
    """Unit tests for FetchArm class."""

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
