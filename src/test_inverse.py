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
# from gtsam import Point3, Pose3, Rot3
from gtsam import *                                      
from utils import vector, unit_twist, compose, spatial_velocity, FetchTestCase

def forward_backward_R():
    # Calculate torque for R manipulator with forward_backward
    return vector(0)

def inverse_factor_graph_way_R():
    # Setup factor graph
    # Optimize it
    # return torque

    # Create noise models
    TWIST_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
    TWIST_ACCEL_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
    WRECH_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
    PRIOR_TWIST_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
    PRIOR_TWIST_ACCEL_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))
    PRIOR_WRECH_NOISE = noiseModel_Diagonal.Sigmas(np.array([0.0, 0.0, 0.0]))

    # Create a factor graph container and add factors to it
    graph = NonlinearFactorGraph()
    
    # Add a prior
    priorTwistMean = Pose2(0.0, 0.0, 0.0)
    priorTwistAccelMean = Pose2(0.0, 0.0, 0.0)
    priorWrechMean = Pose2(0.0, 0.0, 0.0)
    graph.add(PriorFactorPose2(1, priorTwistMean, PRIOR_TWIST_NOISE))
    graph.add(PriorFactorPose2(1, priorTwistAccelMean, PRIOR_TWIST_ACCEL_NOISE))
    graph.add(PriorFactorPose2(3, priorWrechMean, PRIOR_WRECH_NOISE))

    # Add twist factors
    twist = Pose2(1.0, 0.0, 1.0)
    graph.add(BetweenFactorPose2(1, 2, twist, TWIST_NOISE))
    
    # Add twist acceleration factors
    twist_accel = Pose2(0.0, 0.0, 0.0)
    grash.add(BetweenFactorPose2(3, 4, twist_accel, TWIST_ACCEL_NOISE))

    # Add wrech factors
    wrech = Pose2(0.0, 1.0, 0.0)
    graph.add(BetweenFactorPose2(5, 6))

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
