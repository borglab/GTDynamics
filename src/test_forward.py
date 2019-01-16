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
    """
    returns joint accelerations for all joints,
    take number of links and results from factor graph as input
    """
    return [results.at(symbol(ord('j'), i+1)) for i in range(num_of_links)]

class forward_factor_graph_way():
    """
        Calculate forward dynamics for manipulator using factor graph method
    """
    def __init__(self, calibration):
        """
        Constructor that initializes factor graph
        for forward dynamics of manipulator
        Construct from arguments:
            calibration : manipulator calibration
            joint_vel   : joint velocities for each link
            torque      : torque applied to joints
        """
        # number of revolute joint
        self._calibration = calibration
        # configuration of link frame in base frame at home position
        self._link_config_home = calibration.link_configuration_home()


    def link_configuration(self, joint_angles):
        """
        calculate link configurations expessed in its previous link frame
        take joint angles as input
        """
        return [Pose3()] + [utils.compose(self._link_config_home[i].between(self._link_config_home[i-1]), 
                Pose3(Rot3.Yaw(utils.degrees_to_radians(joint_angles[i])), Point3())) 
                for i in range(1, self._calibration.num_of_links()+2)]


    def link_twist_i(self, i_T_i_minus_1, twist_i_mius_1, joint_vel_i, screw_axis_i):
        """
        Calculate twist on the ith link based on the dynamic equation
        Return twist on the ith link
        take link i configuration expressed in link i-1 frame, 
        link i-1 twist, link i joint velocity, ith screw axis
        """
        twist_i = np.dot(i_T_i_minus_1.AdjointMap(),
                        twist_i_mius_1) + screw_axis_i * utils.degrees_to_radians(joint_vel_i)
        return twist_i



    def twist_accel_factor(self, i_T_i_minus_1, twist_i, joint_vel_i, screw_axis_i, i):
        """
        Return factor based on twist acceleration equation of the ith link
        """
        # factor graph keys
        key_twist_accel_i_minus_1 = symbol(ord('t'), i - 1)
        key_twist_accel_i = symbol(ord('t'), i)
        key_joint_accel_i = symbol(ord('j'), i)

        # LHS of acceleration equation
        J_twist_accel_i = np.identity(6)
        J_twist_accel_i_minus_1 = -i_T_i_minus_1.AdjointMap()
        J_joint_accel_i = -np.reshape(screw_axis_i, (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(Pose3.adjointMap(twist_i), 
                    screw_axis_i*utils.degrees_to_radians(joint_vel_i))

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(key_twist_accel_i_minus_1, J_twist_accel_i_minus_1,
                                    key_twist_accel_i, J_twist_accel_i,
                                    key_joint_accel_i, J_joint_accel_i,
                                    b_accel, model)

    def joint_accel_factor(self, i_plus_1_T_i, twist_i, i):
        """
        Return factor based on joint acceleration equation of the ith link
        """
        # get inertia of link i
        inertia = self._calibration.link_inertia(i)
        # get mass of link i
        mass = self._calibration.link_mass(i)
        # factor graph keys
        key_twist_accel_i = symbol(ord('t'), i)
        key_wrench_i = symbol(ord('w'), i)
        key_wrench_i_plus_1 = symbol(ord('w'), i + 1)

        # LHS of wrench equation
        J_wrench_i = np.identity(6)
        J_wrench_i_plus_1 = -i_plus_1_T_i.AdjointMap().transpose()
        J_twist_accel_i = -utils.inertia_matrix(inertia, mass)
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(Pose3.adjointMap(twist_i).transpose(),
                        utils.inertia_matrix(inertia, mass)), twist_i)

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(key_wrench_i, J_wrench_i,
                                    key_wrench_i_plus_1, J_wrench_i_plus_1,
                                    key_twist_accel_i, J_twist_accel_i,
                                    b_wrench, model)

    def wrench_factor(self, torque_i, screw_axis_i, i):
        """
        Return factor based on wrench equation of the ith link
        """
        # factor graph keys
        key_wrench_i = symbol(ord('w'), i)

        # LHS of torque equation
        J_wrench_i = np.array([screw_axis_i])
        # RHS of torque equation
        b_torque = np.array([torque_i])
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.0]))
        return gtsam.JacobianFactor(key_wrench_i, J_wrench_i, b_torque, model)

    def prior_factor_base(self):
        """
        Return prior factor that twist acceleration 
        on base link is assumed to be zero for 
        angular and gravity accelration for linear 
        """
        # LHS
        J_twist_accel_i_minus_1 = np.identity(6)
        # RHS
        b_twist_accel = np.array([0, 0, 0, 0, 0, 9.8])
        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(symbol(ord('t'), 0),
                                    J_twist_accel_i_minus_1, b_twist_accel, model)

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
        return gtsam.JacobianFactor(symbol(ord('w'), self._calibration.num_of_links() + 1),
                                    J_wrench_i_plus_1, b_wrench, model)


    def forward_factor_graph(self, joint_angles, joint_velocities, joint_torques):
        """
        build factor graph for RR manipulator forward dynamics
        Return Gaussian factor graph
        take joint angles, joint velocities and torques as input
        """
        # Setup factor graph
        # Gaussian Factor Graph
        gfg = gtsam.GaussianFactorGraph()

        # prior factor, link 0 twist acceleration = 0
        gfg.add(self.prior_factor_base())

        # screw axis for each joints expressed in its link frame 
        screw_axes = self._calibration.screw_axes()

        # configuration of link frame i-1 relative to link frame i for arbitrary joint angle
        link_config = self.link_configuration(joint_angles)

        # link i-1 (base for the first iteration) twist
        twist_i_mius_1 = vector(0, 0, 0, 0, 0, 0)

        for i in range(1, self._calibration.num_of_links() + 1):
            # ith link twist
            twist_i = self.link_twist_i(link_config[i], twist_i_mius_1, joint_velocities[i], screw_axes[i])

            # factor 1
            gfg.add(self.twist_accel_factor(link_config[i], twist_i, joint_velocities[i], screw_axes[i], i))

            # factor 2
            gfg.add(self.joint_accel_factor(link_config[i+1], twist_i, i))

            # factor 3
            gfg.add(self.wrench_factor(joint_torques[i], screw_axes[i], i))

            # update link i-1 twist for the next iteration
            twist_i_mius_1 = twist_i

        # prior factor, end effector wrench = 0
        gfg.add(self.prior_factor_eef())
        return gfg

    def factor_graph_optimization(self, forward_factor_graph):
        """
        optimize factor graph for manipulator forward dynamics
        Return joint accelerations
        take factor graph for forward dynamics as input
        """
        results = forward_factor_graph.optimize()
        return joint_accel_result(self._calibration.num_of_links(), results)


class TestForwardDynamics(GtsamTestCase):
    """Unit tests for different manipulators class."""

    def setUp(self):
        """setup."""
        pass

    def test_RR_forward_dynamics(self):
        """Try a simple RR robot."""
        # Denavit-Hartenberg parameters for RR manipulator
        rr_link_parameters = [
            LinkParameters( 0, 0,  0, 0, 'B', 0, Point3(0, 0, 0), [0, 0, 0]),
            LinkParameters( 0, 0,  2, 0, 'R', 1, Point3(1, 0, 0), [0, 1/6., 1/6.]),
            LinkParameters( 0, 0,  2, 0, 'R', 1, Point3(1, 0, 0), [0, 1/6., 1/6.]),
            LinkParameters( 0, 90, 0, 0, 'G', 0, Point3(0, 0, 0), [0, 0, 0])
        ]

        RR_calibration = DenavitHartenberg(rr_link_parameters, 2)
        expected_joint_accels = vector(0, 0) # frome MATLAB
        # Call a function with appropriate arguments to co compute them
        joint_angles = [0, 0, 0, 0]
        joint_velocities = [0, 1, 1, 0]
        joint_torques = [0, 0, 0, 0]
        factor_graph_method = forward_factor_graph_way(RR_calibration)
        factor_graph = factor_graph_method.forward_factor_graph(joint_angles, joint_velocities, joint_torques)
        actual_joint_accels = factor_graph_method.factor_graph_optimization(factor_graph)
        np.testing.assert_array_almost_equal(
            actual_joint_accels, expected_joint_accels)

    def test_PUMA_forward_dynamics(self):
        """Try a PUMA robot."""
        # Denavit-Hartenberg parameters for PUMA manipulator
        puma_link_parameters = [
            LinkParameters( 0,      0,  0,       0,  'B', 0,     Point3(0, 0, 0),              [0, 0, 0]),
            LinkParameters( 1,      5,  0,      -90, 'R', 0,     Point3(0, 0, 0),              [0, 0, 0.35]),
            LinkParameters( 0.2435, 10, 0.4318,  0,  'R', 17.40, Point3(0.068, 0.006, -0.016), [0.130, 0.524, 0.539]),
            LinkParameters(-0.0934, 15, 0.0203, -90, 'R', 4.80,  Point3(0, -0.070, 0.014),     [0.066, 0.0125, 0.086]),
            LinkParameters( 0.4331, 20, 0,       90, 'R', 0.82,  Point3(0, 0, -0.019),         [0.0018, 0.0018, 0.00130]),
            LinkParameters( 0,      25, 0,      -90, 'R', 0.34,  Point3(0, 0, 0),              [0.00030, 0.00030, 0.00040]),
            LinkParameters( 0.2000, 30, 0,       90, 'R', 0.09,  Point3(0, 0, 0.032),          [0.00015, 0.00015, 0.00004]),
            LinkParameters( 0,      90, 0,       0,  'G', 0,     Point3(0, 0, 0),              [0, 0, 0]),
        ]

        PUMA_calibration = DenavitHartenberg(puma_link_parameters, 6)
        expected_joint_accels = vector(0.174533,  0.349066,  0.523599,  0.698132,  0.872665,  1.047198) # from MATLAB
        # Call a function with appropriate arguments to co compute them
        joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]
        joint_velocities = [0, -5, -10, -15, -20, -25, -30, 0]
        joint_torques = [0, 0.626950752326773, -34.8262338725151, 1.02920598714973, 
                        -0.0122426673731905, 0.166693973271978, 7.20736555357164e-05, 0]
        factor_graph_method = forward_factor_graph_way(PUMA_calibration)
        factor_graph = factor_graph_method.forward_factor_graph(joint_angles, joint_velocities, joint_torques)
        actual_joint_accels = factor_graph_method.factor_graph_optimization(factor_graph)
        np.testing.assert_array_almost_equal(
        actual_joint_accels, expected_joint_accels)

if __name__ == "__main__":
    unittest.main()
