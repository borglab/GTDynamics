#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import unittest
import numpy as np
from gtsam import Point3, Pose3, Rot3, GaussianFactorGraph, noiseModel_Diagonal

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
    Return wrench on last link
    """
    return vector(0, 0, 0, -7, 0, 0)


def forward_factor_graph_way_RR():
    """
    Calculate joint accelerations for RR manipulator with factor graph method
    Return wrench on last link
    """

    # number of revolute joint
    num = 2

    twist_i_mius_1 = vector(0., 0., 0., 0., 0., 0.)
    # configuration of link 0 frame in space frame 0
    pose3_0 = Pose3(Rot3(np.identity(3)), Point3(0, 0, 0))
    # configuration of link 1 frame in space frame 0
    pose3_1 = Pose3(Rot3(np.identity(3)), Point3(1, 0, 0))
    # configuration of link 2 frame in space frame 0
    pose3_2 = Pose3(Rot3(np.identity(3)), Point3(3, 0, 0))
    # configuration of end effector frame in space frame 0
    pose3_eef = Pose3(Rot3(np.identity(3)), Point3(4, 0, 0))
    link_config = np.array([pose3_0, pose3_1, pose3_2, pose3_eef])

    # joint velocity of link 1
    joint_vel = np.array([0.0, 1.0, 1.0])

    # screw axis for joint 0 expressed in link frame 0
    screw_axis_0 = utils.unit_twist(vector(0, 0, 0), vector(0, 0, 0))
    # screw axis for joint 1 expressed in link frame 1
    screw_axis_1 = utils.unit_twist(vector(0, 0, 1), vector(-1, 0, 0))
    # screw axis for joint 2 expressed in link frame 2
    screw_axis_2 = utils.unit_twist(vector(0, 0, 1), vector(-1, 0, 0))
    screw_axis = np.array([screw_axis_0, screw_axis_1, screw_axis_2])

    # inertial matrix of link i expressed in link frame i
    I0 = np.zeros((3, 3))
    I1 = np.diag([0, 1/6., 1/6.])
    I2 = np.diag([0, 1/6., 1/6.])
    I = np.array([I0, I1, I2])
    # mass of link i
    m0 = 0
    m1 = 1
    m2 = 1
    m = np.array([m0, m1, m2])

    # Setup factor graph
    # Gaussian Factor Graph
    gfg = GaussianFactorGraph()

    # factor graph keys
    key_twist_accel_i_minus_1 = 1
    key_twist_accel_i = 2
    key_joint_accel_i = 3
    key_wrench_i = 4
    key_wrench_i_plus_1 = 5

    for i in range(1, num+1):

        # configuration of link frame i-1 relative to link frame i for joint i angle 0
        i_T_i_minus_1 = utils.compose(
            link_config[i].inverse(), link_config[i-1])
        twist_i = helper_calculate_twist_i(
            i_T_i_minus_1, joint_vel[i], twist_i_mius_1, screw_axis[i])

        if i is not 1:
            key_twist_accel_i = key_wrench_i + 1
            key_joint_accel_i = key_wrench_i + 2
            key_wrench_i_plus_1 = key_wrench_i + 3

        print(key_twist_accel_i_minus_1, key_twist_accel_i,
              key_joint_accel_i, key_wrench_i, key_wrench_i_plus_1)

        # factor 1
        # LHS of acceleration equation
        J_twist_accel_i = np.identity(6)
        J_twist_accel_i_mius_1 = -i_T_i_minus_1.AdjointMap()
        J_joint_accel_i = -np.reshape(screw_axis[i], (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(utils.adtwist(twist_i), screw_axis[i]*joint_vel[i])

        sigmas = np.zeros(6)
        model = noiseModel_Diagonal.Sigmas(sigmas)
        gfg.add(key_twist_accel_i_minus_1, J_twist_accel_i_mius_1,
                key_twist_accel_i, J_twist_accel_i,
                key_joint_accel_i, J_joint_accel_i,
                b_accel, model)

        # configuration of link frame i relative to link frame i+1 for joint i+1 angle 0
        i_plus_1_T_i = utils.compose(
            link_config[i+1].inverse(), link_config[i])

        # factor 2
        # LHS of wrench equation
        J_wrench_i = np.identity(6)
        J_wrench_i_plus_1 = -i_plus_1_T_i.AdjointMap().transpose()
        J_twist_accel_i = -utils.genaral_mass_matrix(I[i], m[i])
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(utils.adtwist(twist_i).transpose(),
                                  utils.genaral_mass_matrix(I[i], m[i])), twist_i)

        sigmas = np.zeros(6)
        model = noiseModel_Diagonal.Sigmas(sigmas)
        gfg.add(key_wrench_i, J_wrench_i,
                key_wrench_i_plus_1, J_wrench_i_plus_1,
                key_twist_accel_i, J_twist_accel_i,
                b_wrench, model)

        # factor 3
        # LHS of torque equation
        J_wrench_i = np.array([screw_axis[i]])
        # RHS of torque equation
        b_torque = np.array([0])
        model = noiseModel_Diagonal.Sigmas(np.array([0.0]))
        gfg.add(key_wrench_i, J_wrench_i, b_torque, model)

        if i == 1:
            # prior factor, link 0 twist acceleration = 0
            # LHS
            J_twist_accel_i_mius_1 = np.identity(6)
            # RHS
            b_twist_accel = np.zeros(6)
            sigmas = np.zeros(6)
            model = noiseModel_Diagonal.Sigmas(sigmas)
            gfg.add(key_twist_accel_i_minus_1,
                    J_twist_accel_i_mius_1, b_twist_accel, model)

        if i == num:
            # prior factor, end effector wrench = 0
            # LHS
            J_wrench_i_plus_1 = np.identity(6)
            # RHS
            b_wrench = np.zeros(6)
            sigmas = np.zeros(6)
            model = noiseModel_Diagonal.Sigmas(sigmas)
            gfg.add(key_wrench_i_plus_1, J_wrench_i_plus_1, b_wrench, model)

        twist_i_mius_1 = twist_i
        key_twist_accel_i_minus_1 = key_twist_accel_i
        key_wrench_i = key_wrench_i_plus_1

    results = gfg.optimize()
    return results.at(4)


class TestForwardDynamics(GtsamTestCase):
    """Unit tests for R manipulator class."""

    def setUp(self):
        """setup."""
        pass

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        expected_joint_accels = forward_traditional_way_RR()
        # Call a function with appropriate arguments to co compute them
        actual_joint_accels = forward_factor_graph_way_RR()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)


if __name__ == "__main__":
    unittest.main()
