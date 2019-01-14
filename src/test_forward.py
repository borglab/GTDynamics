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

import link_parameters

def forward_traditional_way(paras):
    """
    Calculate joint accelerations for manipulator using traditional method 
    Return joint accelerations
    arguments: paras:
               Denavit-Hartenberg parameters and
               input values for PUMA manipulator
    """
    return paras.dh_tdd[1:-1] 

def get_link_configuration(num_of_links, paras):
    """
    return each link frame (origin at center of mass) 
    expressed in space frame
    arguments: paras:
               Denavit-Hartenberg parameters and
               input values for PUMA manipulator
    """
    link_config = np.array(Pose3(Rot3(), Point3(0, 0, 0)))
    # link i-1 joint frame expressed in space frame s
    frame_joint_i_minus_1 = Pose3(Rot3(), Point3(0, 0, 0))

    for i in range(1, num_of_links+1):
        # link i joint frame expressed in link i-1 joint frame
        joint_i_minus_1_frame_joint_i = utils.compose(Pose3(Rot3.Roll(paras.dh_f[i-1]), Point3(paras.dh_a[i-1], 0, 0)),
                                           Pose3(Rot3.Yaw(paras.dh_t[i]), Point3(0, 0, paras.dh_d[i])))
        # link i joint frame expressed in space frame s
        frame_joint_i = utils.compose(frame_joint_i_minus_1, joint_i_minus_1_frame_joint_i)
        # update link i-1 joint frame in space frame s for next iteration
        frame_joint_i_minus_1 = frame_joint_i
        # link i com frame expressed in space frame s 
        frame_i = utils.compose(frame_joint_i, Pose3(Rot3(), Point3(paras.dh_com[0, i],
                                                                    paras.dh_com[1, i], 
                                                                    paras.dh_com[2, i])))
        link_config = np.append(link_config, frame_i)

    # configuration of link eef frame expressed in space frame num_of_links
    # eef frame expressed in last link joint frame
    joint_last_frame_eef = utils.compose(Pose3(Rot3.Roll(paras.dh_f[num_of_links]), Point3(paras.dh_a[num_of_links], 0, 0)),
                                              Pose3(Rot3.Yaw(paras.dh_t[num_of_links+1]), Point3(0, 0, paras.dh_d[num_of_links+1])))
    # eef frame expressed in space frame s
    frame_eef = utils.compose(frame_joint_i_minus_1, joint_last_frame_eef)
    link_config = np.append(link_config, frame_eef)
    return link_config

def get_screw_axis(num_of_links, paras):
    """
    return screw axis of each joints expressed in its own link frame
    arguments: paras:
               Denavit-Hartenberg parameters and
               input values for PUMA manipulator    
    """
    screw_axis = np.array([utils.unit_twist(vector(0, 0, 0), vector(0, 0, 0))])
    for i in range(1, num_of_links+1):
        # screw axis for joint i expressed in link frame i
        screw_axis = np.append(screw_axis, [utils.unit_twist(vector(0, 0, 1),
                                                            vector(-paras.dh_com[0, i],
                                                                   -paras.dh_com[1, i], 
                                                                   -paras.dh_com[2, i]))], axis=0)    
    return screw_axis

def joint_accel_result(num_of_links, results):
    joint_accel_result = np.array([])
    for i in range(1, num_of_links+1):
        joint_accel_result = np.append(joint_accel_result, results.at(symbol(ord('j'), i)))
    return joint_accel_result

class forward_factor_graph_way():
    """
        Calculate forward dynamics for manipulator using factor graph method
    """
    def __init__(self, paras):
        """
        Constructor that initializes forward factor graph
        for dynamics of manipulator (Puma robot arm or RR)
        arguments: paras:
                   Denavit-Hartenberg parameters and
                   input values for PUMA manipulator
        """
        # number of revolute joint
        self.num_of_links = paras.dh_d.size - 2
        # configuration of base com frame in space frame s
        self.link_config = get_link_configuration(self.num_of_links, paras)
        # screw axis for joint 0 expressed in link frame 0
        self.screw_axis = get_screw_axis(self.num_of_links, paras)
        # joint velocities (1 rad/s) of joint between link i-1 and link i
        self.joint_vel = paras.dh_td[:-1]
        # mass of each link
        self.m = paras.dh_m[:-1]
        # inertial matrix of link i expressed in link frame i
        self.I = paras.dh_pI[:, :-1]
        # twist of link 0
        self.twist_i_mius_1 = vector(0, 0, 0, 0, 0, 0)
        # torque applied to each link
        self.torque = paras.dh_tq

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
        b_accel = np.dot(Pose3.adjointMap(self.twist_i), self.screw_axis[i]*self.joint_vel[i])

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
        J_twist_accel_i = -utils.inertia_matrix(self.I[:,i], self.m[i])
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(Pose3.adjointMap(self.twist_i).transpose(),
                                utils.inertia_matrix(self.I[:,i], self.m[i])), self.twist_i)

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
        b_torque = np.array([self.torque[i]])
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
        optimize factor graph for manipulator forward dynamics
        Return wrench on first link
        Return joint accelerations
        """
        gfg = self.forward_factor_graph()
        results = gfg.optimize()
        return joint_accel_result(self.num_of_links, results)


class TestForwardDynamics(GtsamTestCase):
    """Unit tests for different manipulators class."""

    def setUp(self):
        """setup."""
        pass

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        RR = link_parameters.LinkParameters_RR()
        expected_joint_accels = forward_traditional_way(RR)
        # Call a function with appropriate arguments to co compute them
        ffg_way = forward_factor_graph_way(RR)
        actual_joint_accels = ffg_way.factor_graph_optimization()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)

    def test_PUMA_forward_dynamics(self):
        """Try a PUMA robot."""
        PUMA = link_parameters.LinkParameters_PUMA()
        expected_joint_accels = forward_traditional_way(PUMA)
        # Call a function with appropriate arguments to co compute them
        ffg_way = forward_factor_graph_way(PUMA)
        actual_joint_accels = ffg_way.factor_graph_optimization()
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)

if __name__ == "__main__":
    unittest.main()
