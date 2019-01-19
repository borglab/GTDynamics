"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math

import gtsam
import numpy as np
import utils


def symbol(char, index):
    """Shorthand for gtsam symbol."""
    return gtsam.symbol(ord(char), index)


def J(index):
    """Shorthand for j_index."""
    return symbol('j', index)


def T(index):
    """Shorthand for j_index."""
    return symbol('t', index)


def W(index):
    """Shorthand for j_index."""
    return symbol('w', index)


def joint_accel_result(num_of_links, results):
    """
    returns joint accelerations for all joints,
    take number of links and results from factor graph as input
    """
    return [results.at(J(i+1)) for i in range(num_of_links)]


class Manipulator(object):
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
        """Calculate link configurations expessed in its previous link frame take joint angles as input."""
        def iTi_1(i):
            """Transform from i-1 to i."""
            return self._link_config_home[i].between(self._link_config_home[i-1])

        def yaw(i):
            """Rotate by joint_angles[i]."""
            return gtsam.Pose3(gtsam.Rot3.Yaw(math.radians(joint_angles[i])), gtsam.gtsam.Point3())

        return [utils.compose(iTi_1(i), yaw(i))
                for i in range(1, self._calibration.num_of_links())]

    def link_twist_i(self, i_T_i_minus_1, twist_i_mius_1, joint_vel_i, screw_axis_i):
        """
        Calculate twist on the ith link based on the dynamic equation
        Return twist on the ith link
        take link i configuration expressed in link i-1 frame,
        link i-1 twist, link i joint velocity, ith screw axis
        """
        twist_i = np.dot(i_T_i_minus_1.AdjointMap(),
                         twist_i_mius_1) + screw_axis_i * math.radians(joint_vel_i)
        return twist_i

    def twist_accel_factor(self, i_T_i_minus_1, twist_i, joint_vel_i, screw_axis_i, i):
        """
        Return factor based on twist acceleration equation of the ith link
        """
        # factor graph keys
        key_twist_accel_i_minus_1 = T(i - 1)
        key_twist_accel_i = T(i)
        key_joint_accel_i = J(i)

        # LHS of acceleration equation
        J_twist_accel_i = np.identity(6)
        J_twist_accel_i_minus_1 = -i_T_i_minus_1.AdjointMap()
        J_joint_accel_i = -np.reshape(screw_axis_i, (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(gtsam.Pose3.adjointMap(twist_i),
                         screw_axis_i*math.radians(joint_vel_i))

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(key_twist_accel_i_minus_1, J_twist_accel_i_minus_1,
                                    key_twist_accel_i, J_twist_accel_i,
                                    key_joint_accel_i, J_joint_accel_i,
                                    b_accel, model)

    def joint_accel_factor(self, i_plus_1_T_i, twist_i, i):
        """
        Return factor based on joint acceleration equation of the ith link
        """
        # get mass and inertia of link i
        (mass, inertia) = self._calibration.link_properties(i)
        # factor graph keys
        key_twist_accel_i = T(i)
        key_wrench_i = W(i)
        key_wrench_i_plus_1 = W(i + 1)

        # LHS of wrench equation
        J_wrench_i = np.identity(6)
        J_wrench_i_plus_1 = -i_plus_1_T_i.AdjointMap().transpose()
        J_twist_accel_i = -utils.inertia_matrix(inertia, mass)
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(gtsam.Pose3.adjointMap(twist_i).transpose(),
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
        key_wrench_i = W(i)

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
        return gtsam.JacobianFactor(T(0),
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
        return gtsam.JacobianFactor(W(self._calibration.num_of_links() + 1),
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
        twist_i_mius_1 = utils.vector(0, 0, 0, 0, 0, 0)

        for i in range(1, self._calibration.num_of_links() + 1):
            # ith link twist
            twist_i = self.link_twist_i(
                link_config[i], twist_i_mius_1, joint_velocities[i], screw_axes[i])

            # factor 1
            gfg.add(self.twist_accel_factor(
                link_config[i], twist_i, joint_velocities[i], screw_axes[i], i))

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
