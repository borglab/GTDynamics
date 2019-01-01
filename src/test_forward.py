#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest
import numpy as np
from gtsam import Point3, Pose3, Rot3, symbol, \
                    GaussianFactorGraph, noiseModel_Diagonal

import utils
from utils import vector, GtsamTestCase


def helper_calculate_twist_i(i_T_i_minus_1, joint_vel_i, twist_i_mius_1, screw_axis_i):
    """
    Calculate twist on the ith link based on the dynamic equation
    """
    twist_i = np.dot(i_T_i_minus_1.AdjointMap(),
                     twist_i_mius_1) + screw_axis_i*joint_vel_i
    return twist_i


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

        # Setup factor graph
        # Gaussian Factor Graph
        self.gfg = GaussianFactorGraph()

    def twist_accel_factor(self, i_T_i_minus_1, i):
        # LHS of acceleration equation
        J_twist_accel_i = np.identity(6)
        J_twist_accel_i_mius_1 = -i_T_i_minus_1.AdjointMap()
        J_joint_accel_i = -np.reshape(self.screw_axis[i], (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(utils.adtwist(self.twist_i), self.screw_axis[i]*self.joint_vel[i])

        sigmas = np.zeros(6)
        model = noiseModel_Diagonal.Sigmas(sigmas)
        self.gfg.add(self.key_twist_accel_i_minus_1, J_twist_accel_i_mius_1,
                    self.key_twist_accel_i, J_twist_accel_i,
                    self.key_joint_accel_i, J_joint_accel_i,
                    b_accel, model)

    def joint_accel_factor(self, i_plus_1_T_i, i):
        # LHS of wrench equation
        J_wrench_i = np.identity(6)
        J_wrench_i_plus_1 = -i_plus_1_T_i.AdjointMap().transpose()
        J_twist_accel_i = -utils.genaral_mass_matrix(self.I[i], self.m[i])
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(utils.adtwist(self.twist_i).transpose(),
                                utils.genaral_mass_matrix(self.I[i], self.m[i])), self.twist_i)

        sigmas = np.zeros(6)
        model = noiseModel_Diagonal.Sigmas(sigmas)
        self.gfg.add(self.key_wrench_i, J_wrench_i,
                    self.key_wrench_i_plus_1, J_wrench_i_plus_1,
                    self.key_twist_accel_i, J_twist_accel_i,
                    b_wrench, model)

    def wrench_factor(self, i):
        # LHS of torque equation
        J_wrench_i = np.array([self.screw_axis[i]])
        # RHS of torque equation
        b_torque = np.array([0])
        model = noiseModel_Diagonal.Sigmas(np.array([0.0]))
        self.gfg.add(self.key_wrench_i, J_wrench_i, b_torque, model)

    def prior_factor_base(self):
        # LHS
        J_twist_accel_i_mius_1 = np.identity(6)
        # RHS
        b_twist_accel = np.zeros(6)
        sigmas = np.zeros(6)
        model = noiseModel_Diagonal.Sigmas(sigmas)
        self.gfg.add(self.key_twist_accel_i_minus_1,
                    J_twist_accel_i_mius_1, b_twist_accel, model)

    def prior_factor_eef(self):
        # LHS
        J_wrench_i_plus_1 = np.identity(6)
        # RHS
        b_wrench = np.zeros(6)
        sigmas = np.zeros(6)
        model = noiseModel_Diagonal.Sigmas(sigmas)
        self.gfg.add(self.key_wrench_i_plus_1, J_wrench_i_plus_1, b_wrench, model)

    def forward_factor_graph(self):
        """
        Calculate joint accelerations for RR manipulator with factor graph method
        Return wrench on first link
        """

        for i in range(1, self.num_of_links + 1):
            # factor graph keys
            self.key_twist_accel_i_minus_1 = symbol(ord('t'), i - 1)
            self.key_twist_accel_i = symbol(ord('t'), i)
            self.key_joint_accel_i = symbol(ord('j'), i)
            self.key_wrench_i = symbol(ord('w'), i)
            self.key_wrench_i_plus_1 = symbol(ord('w'), i + 1)

            # configuration of link frame i-1 relative to link frame i for joint i angle 0
            i_T_i_minus_1 = self.link_config[i].between(self.link_config[i-1])
            # configuration of link frame i relative to link frame i+1 for joint i+1 angle 0
            i_plus_1_T_i = self.link_config[i+1].between(self.link_config[i])

            self.twist_i = helper_calculate_twist_i(
                i_T_i_minus_1, self.joint_vel[i], self.twist_i_mius_1, self.screw_axis[i])

            # factor 1
            self.twist_accel_factor(i_T_i_minus_1, i)

            # factor 2
            self.joint_accel_factor(i_plus_1_T_i, i)

            # factor 3
            self.wrench_factor(i)

            if i == 1:
                # prior factor, link 0 twist acceleration = 0
                self.prior_factor_base()

            if i == self.num_of_links:
                # prior factor, end effector wrench = 0
                self.prior_factor_eef()

            self.twist_i_mius_1 = self.twist_i

        results = self.gfg.optimize()
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
        ffg_RR = forward_factor_graph_way_RR(2)
        actual_joint_accels = ffg_RR.forward_factor_graph()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)


if __name__ == "__main__":
    unittest.main()
