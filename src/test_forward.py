#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest
import numpy as np
import gtsam
from gtsam import Point3, Pose3, Rot3, symbol

import utils
from utils import vector, GtsamTestCase

from denavit_hartenberg import DenavitHartenberg, LinkParameters

def joint_accel_result(num_of_links, results):
    joint_accel_result = np.array([])
    for i in range(1, num_of_links+1):
        joint_accel_result = np.append(joint_accel_result, 
                                    results.at(symbol(ord('j'), i)))
    return joint_accel_result

class forward_factor_graph_way():
    """
        Calculate forward dynamics for manipulator using factor graph method
    """
    def __init__(self, calibration):
        """
        Constructor that initializes forward factor graph
        for dynamics of manipulator (Puma robot arm or RR)
        arguments: calibration:
                   Denavit-Hartenberg parameters and
                   input values for PUMA manipulator
        """
        # number of revolute joint
        self.calibration = calibration
        # configuration of base com frame in space ffrom math import pime s
        self.link_config = calibration.link_configuration()
        # screw axis for each joints expressed in itfrom math import pilink frame 
        self.screw_axis = calibration.screw_axis()
        # twist of link 0
        self.twist_i_mius_1 = vector(0, 0, 0, 0, 0, 0)

    def twist_accel_factor(self, i_T_i_minus_1, i):
        """
        Return factor based on twist acceleration equation of the ith link
        """
        # factor graph keys
        key_twist_accel_i_minus_1 = symbol(ord('t'), i - 1)
        key_twist_accel_i = symbol(ord('t'), i)
        key_joint_accel_i = symbol(ord('j'), i)

        # LHS of acceleration equation
        J_twist_accel_i = np.identity(6)
        J_twist_accel_i_mius_1 = -i_T_i_minus_1.AdjointMap()
        J_joint_accel_i = -np.reshape(self.screw_axis[i], (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(Pose3.adjointMap(self.twist_i), 
                    self.screw_axis[i]*self.calibration._link_parameters[i].joint_vel)

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(key_twist_accel_i_minus_1, J_twist_accel_i_mius_1,
                                    key_twist_accel_i, J_twist_accel_i,
                                    key_joint_accel_i, J_joint_accel_i,
                                    b_accel, model)

    def joint_accel_factor(self, i):
        """
        Return factor based on joint acceleration equation of the ith link
        """
        # factor graph keys
        key_twist_accel_i = symbol(ord('t'), i)
        key_wrench_i = symbol(ord('w'), i)
        key_wrench_i_plus_1 = symbol(ord('w'), i + 1)

        # configuration of link frame i relative to link frame i+1 for joint i+1 angle 0
        i_plus_1_T_i = self.link_config[i+1].between(self.link_config[i])

        # LHS of wrench equation
        J_wrench_i = np.identity(6)
        J_wrench_i_plus_1 = -i_plus_1_T_i.AdjointMap().transpose()
        J_twist_accel_i = -utils.inertia_matrix(self.calibration._link_parameters[i].inertia, 
                                                self.calibration._link_parameters[i].mass)
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(Pose3.adjointMap(self.twist_i).transpose(),
                        utils.inertia_matrix(self.calibration._link_parameters[i].inertia, 
                                            self.calibration._link_parameters[i].mass)), self.twist_i)

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(key_wrench_i, J_wrench_i,
                                    key_wrench_i_plus_1, J_wrench_i_plus_1,
                                    key_twist_accel_i, J_twist_accel_i,
                                    b_wrench, model)

    def wrench_factor(self, i):
        """
        Return factor based on wrench equation of the ith link
        """
        # factor graph keys
        key_wrench_i = symbol(ord('w'), i)

        # LHS of torque equation
        J_wrench_i = np.array([self.screw_axis[i]])
        # RHS of torque equation
        b_torque = np.array([self.calibration._link_parameters[i].torque])
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.0]))
        return gtsam.JacobianFactor(key_wrench_i, J_wrench_i, b_torque, model)

    def prior_factor_base(self):
        """
        Return prior factor that twist acceleration 
        on base link is assumed to be zero for 
        angular and gravity accelration for linear 
        """
        # LHS
        J_twist_accel_i_mius_1 = np.identity(6)
        # RHS
        b_twist_accel = np.array([0, 0, 0, 0, 0, 9.8])
        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(symbol(ord('t'), 0),
                                    J_twist_accel_i_mius_1, b_twist_accel, model)

    def prior_factor_eef(self):
        """
        Return prior factor that wrench
        on end effector is assumed to be zero
        """
        # LHS
        J_wrench_i_plus_1 = np.identity(6)
        # RHS
        b_wrench = np.zeros(6)
        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(symbol(ord('w'), self.calibration._num_of_links + 1),
                                    J_wrench_i_plus_1, b_wrench, model)

    def calculate_twist_i(self, i_T_i_minus_1, i):
        """
        Calculate twist on the ith link based on the dynamic equation
        Return twist on the ith link
        """
        twist_i = np.dot(i_T_i_minus_1.AdjointMap(),
                        self.twist_i_mius_1) + self.screw_axis[i]*self.calibration._link_parameters[i].joint_vel
        return twist_i


    def forward_factor_graph(self):
        """
        build factor graph for RR manipulator forward dynamics
        Return Gaussian factor graph
        """
        # Setup factor graph
        # Gaussian Factor Graph
        gfg = gtsam.GaussianFactorGraph()

        # prior factor, link 0 twist acceleration = 0
        gfg.add(self.prior_factor_base())

        for i in range(1, self.calibration._num_of_links + 1):
            # configuration of link frame i-1 relative to link frame i for joint i angle 0
            i_T_i_minus_1 = self.link_config[i].between(self.link_config[i-1])

            self.twist_i = self.calculate_twist_i(i_T_i_minus_1, i)

            # factor 1
            gfg.add(self.twist_accel_factor(i_T_i_minus_1, i))

            # factor 2
            gfg.add(self.joint_accel_factor(i))

            # factor 3
            gfg.add(self.wrench_factor(i))

            self.twist_i_mius_1 = self.twist_i

        # prior factor, end effector wrench = 0
        gfg.add(self.prior_factor_eef())
        return gfg

    def factor_graph_optimization(self):
        """
        optimize factor graph for manipulator forward dynamics
        Return wrench on first link
        Return joint accelerations
        """
        gfg = self.forward_factor_graph()
        results = gfg.optimize()
        return joint_accel_result(self.calibration._num_of_links, results)


class TestForwardDynamics(GtsamTestCase):
    """Unit tests for different manipulators class."""

    def setUp(self):
        """setup."""
        pass

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        # Denavit-Hartenberg parameters and input values for RR manipulator
        rr_link_parameters = [
            LinkParameters( 0, 0, 0, 0, 'B', 0, 0, 0, [0, 0, 0], [0, 0, 0], 0),
            LinkParameters( 0, 0, 2, 0, 'R', 1, 0, 1, [1, 0, 0], [0, 1/6., 1/6.], 0),
            LinkParameters( 0, 0, 2, 0, 'R', 1, 0, 1, [1, 0, 0], [0, 1/6., 1/6.], 0),
            LinkParameters( 0, 90, 0, 0, 'G', 0, 0, 0, [0, 0, 0], [0, 0, 0], 0)
        ]

        RR_calibration = DenavitHartenberg(rr_link_parameters, 2)
        expected_joint_accels = [0, 0]
        # Call a function with appropriate arguments to co compute them
        ffg_way = forward_factor_graph_way(RR_calibration)
        actual_joint_accels = ffg_way.factor_graph_optimization()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)

    def test_PUMA_forward_dynamics(self):
        """Try a PUMA robot."""
        # Denavit-Hartenberg parameters and input values for PUMA manipulator
        puma_link_parameters = [
            LinkParameters( 0, 0, 0, 0, 'B', 0, 0, 0,     
                        [0, 0, 0], [0, 0, 0], 0),
            LinkParameters( 1, 1*5, 0, -90, 'R', 1*(-5), 1*(10), 0,     
                        [0, 0, 0], [0, 0, 0.35], 0.626950752326773),
            LinkParameters( 0.2435, 2*5, 0.4318, 0, 'R', 2*(-5), 2*10, 17.40, 
                        [0.068, 0.006, -0.016], [0.130, 0.524, 0.539], -34.8262338725151),
            LinkParameters(-0.0934, 3*5, 0.0203, -90, 'R', 3*(-5), 3*10, 4.80,  
                        [0, -0.070, 0.014], [0.066, 0.0125, 0.086], 1.02920598714973),
            LinkParameters( 0.4331, 4*5, 0, 90, 'R', 4*(-5), 4*10, 0.82,  
                        [0, 0, -0.019], [0.0018, 0.0018, 0.00130], -0.0122426673731905),
            LinkParameters( 0, 5*5, 0, -90, 'R', 5*(-5), 5*10, 0.34,  
                        [0, 0, 0], [0.00030, 0.00030, 0.00040], 0.166693973271978),
            LinkParameters( 0.2000, 6*5, 0, 90, 'R', 6*(-5), 6*10, 0.09,  
                        [0, 0, 0.032], [0.00015, 0.00015, 0.00004], 7.20736555357164e-05),
            LinkParameters( 0, 90, 0, 0, 'G', 0, 0, 0,     
                        [0, 0, 0], [0, 0, 0], 0),
            ]

        PUMA_calibration = DenavitHartenberg(puma_link_parameters, 6)
        expected_joint_accels = np.array([ 0.174533,  0.349066,  0.523599,  0.698132,  0.872665,  1.047198])
        # Call a function with appropriate arguments to co compute them
        ffg_way = forward_factor_graph_way(PUMA_calibration)
        actual_joint_accels = ffg_way.factor_graph_optimization()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)

if __name__ == "__main__":
    unittest.main()
