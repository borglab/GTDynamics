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

def forward_traditional_way_RR():
    """
    Calculate joint accelerations for RR manipulator with traditional method
    Return wrench on first link
    """
    return vector(0, 0, 0, -7, 0, 0)

class forward_factor_graph_way_RR():
    """
        Calculate forward dynamics for RR manipulator using factor graph method
    """
    def __init__(self, num_of_links):
        """
        Constructor that initializes forward factor graph for RR manipulator dynamics
        Arguments: 
            number_of_links: number of links of RR manipulator
        """
        # number of revolute joint
        self.num_of_links = num_of_links

        self.twist_i_mius_1 = vector(0, 0, 0, 0, 0, 0)
        # configuration of link 0 frame in space frame 0
        pose3_0 = Pose3(Rot3(), Point3(0, 0, 0))
        # configuration of link 1 frame in space frame 0
        pose3_1 = Pose3(Rot3(), Point3(1, 0, 0))
        # configuration of link 2 frame in space frame 0
        pose3_2 = Pose3(Rot3(), Point3(3, 0, 0))
        # configuration of end effector frame in space frame 0
        pose3_eef = Pose3(Rot3(), Point3(4, 0, 0))
        self.link_config = np.array([pose3_0, pose3_1, pose3_2, pose3_eef])

        # joint velocities
        self.joint_vel = np.array([0.0, 1.0, 1.0])

        # screw axis for joint 0 expressed in link frame 0
        screw_axis_0 = utils.unit_twist(vector(0, 0, 0), vector(0, 0, 0))
        # screw axis for joint 1 expressed in link frame 1
        screw_axis_1 = utils.unit_twist(vector(0, 0, 1), vector(-1, 0, 0))
        # screw axis for joint 2 expressed in link frame 2
        screw_axis_2 = utils.unit_twist(vector(0, 0, 1), vector(-1, 0, 0))
        self.screw_axis = np.array([screw_axis_0, screw_axis_1, screw_axis_2])

        # inertial matrix of link i expressed in link frame i
        I0 = np.zeros((3, 3))
        I1 = np.diag([0, 1/6., 1/6.])
        I2 = np.diag([0, 1/6., 1/6.])
        self.I = np.array([I0, I1, I2])
        # mass of link i
        m0 = 0
        m1 = 1
        m2 = 1
        self.m = np.array([m0, m1, m2])

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
        b_accel = np.dot(utils.adtwist(self.twist_i), self.screw_axis[i]*self.joint_vel[i])

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
        J_twist_accel_i = -utils.genaral_mass_matrix(self.I[i], self.m[i])
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(utils.adtwist(self.twist_i).transpose(),
                                utils.genaral_mass_matrix(self.I[i], self.m[i])), self.twist_i)

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
        b_torque = np.array([0])
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.0]))
        return gtsam.JacobianFactor(key_wrench_i, J_wrench_i, b_torque, model)

    def prior_factor_base(self):
        """
        Return prior factor that twist acceleration 
        on base link is assumed to be zero
        """
        # LHS
        J_twist_accel_i_mius_1 = np.identity(6)
        # RHS
        b_twist_accel = np.zeros(6)
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
        return gtsam.JacobianFactor(symbol(ord('w'), self.num_of_links + 1),
                                    J_wrench_i_plus_1, b_wrench, model)

    def calculate_twist_i(self, i_T_i_minus_1, i):
        """
        Calculate twist on the ith link based on the dynamic equation
        Return twist on the ith link
        """
        twist_i = np.dot(i_T_i_minus_1.AdjointMap(),
                        self.twist_i_mius_1) + self.screw_axis[i]*self.joint_vel[i]
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

        for i in range(1, self.num_of_links + 1):
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
        optimize factor graph for RR manipulator forward dynamics
        Return wrench on first link
        """
        gfg = self.forward_factor_graph()
        results = gfg.optimize()
        return results.at(symbol(ord('w'), 1))


class TestForwardDynamics(GtsamTestCase):
    """Unit tests for R manipulator class."""

    def setUp(self):
        """setup."""
        pass

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        expected_joint_accels = forward_traditional_way_RR()
        # Call a function with appropriate arguments to co compute them
        ffg_way_RR = forward_factor_graph_way_RR(2)
        actual_joint_accels = ffg_way_RR.factor_graph_optimization()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)


if __name__ == "__main__":
    unittest.main()
