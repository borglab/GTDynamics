"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie

We follow Lynch & Park 2017 conventions, but using j to index joints, as in Corke 2017:
    - j=0 is base
    - j \in 1...N index joints
    - Mj is COM pose of link j in base frame 0.
    - Tj is the link frame j, aligned with joint j+1, expressed in base frame 0.
    - we use i to denote the previous link/joint with index j-1
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
    """Shorthand for jjndex."""
    return symbol('j', index)


def T(index):
    """Shorthand for jjndex."""
    return symbol('t', index)


def W(index):
    """Shorthand for jjndex."""
    return symbol('w', index)


def joint_accel_result(num_links, results):
    """
    returns joint accelerations for all joints,
    take number of links and results from factor graph as input
    """
    return [results.at(J(i+1)) for i in range(num_links)]


class SerialLink(object):
    """
        Calculate forward dynamics for manipulator using factor graph method
    """

    def __init__(self, calibration, base=gtsam.Pose3(), tool=gtsam.Pose3()):
        """ Constructor serial link manipulator from list of Link instances.
            Keyword arguments:
                calibration -- Link list
                base        -- optional wT0 base frame in world frame
                tool        -- optional tool frame in link N frame
        """
        self._links = calibration
        self._base = base
        self._tool = tool
        self._link_config_home = self.com_frames()

    def num_links(self):
        """return number of *moving* links."""
        return len(self._links)

    def screw_axes(self):
        """
        return screw axis of each joints expressed in its own link frame
        """
        return [link.screw_axis() for link in self._links]

    def link_transforms(self, q):
        """ Calculate link transforms for all links.
            Keyword arguments:
                q (numpy array) -- optional joint_angles (default all zero).
        """
        return [link.A(0 if q is None else q[i]) for i, link in enumerate(self._links)]

    def fkine(self, q):
        """ Forward kinematics.
            Keyword arguments:
                q (numpy array) -- joint_angles.
            Returns tool frame in world frame.
        """
        t = self._base
        A = self.link_transforms(q)
        for A in self.link_transforms(q):
            t = t.compose(A)
        return t.compose(self._tool)

    def link_frames(self, q=None):
        """ Return each link frame for given joint angles.
            Note that frame Tj is aligned with the joint axis of joint j+1
            according to the Denavit-Hartenberg convention.
            Keyword arguments:
                q (numpy array) -- optional joint_angles (default all zero).
            Returns tool frame in world frame.
        """
        frames = []
        t = self._base
        A = self.link_transforms(q)
        for A in self.link_transforms(q):
            t = t.compose(A)
            frames.append(t)
        return frames

    def link_properties(self, j):
        """return link mass and inertia, take link index as input."""
        return self._links[j].properties()

    def com_frames(self):
        """ Return each link frame (origin at center of mass) at home position
            expressed in base frame 0.
        """
        return []  # TODO(Frank): fix

    def link_twist_j(self, jTi, twist_i, joint_vel_j, screw_axis_j):
        """
        Calculate twist on the ith link based on the dynamic equation
        Return twist on the ith link
        take link j configuration expressed in link j-1 frame,
        link j-1 twist, link j joint velocity, ith screw axis
        """
        twist_j = np.dot(jTi.AdjointMap(), twist_i) + \
            screw_axis_j * math.radians(joint_vel_j)
        return twist_j

    def twist_accel_factor(self, jTi, twist_j, joint_vel_j, screw_axis_j, j):
        """
        Return factor based on twist acceleration equation of the ith link
        """
        # LHS of acceleration equation
        J_twist_accel_j = np.identity(6)
        J_twist_accel_i = -jTi.AdjointMap()
        J_joint_accel_j = -np.reshape(screw_axis_j, (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(gtsam.Pose3.adjointMap(twist_j),
                         screw_axis_j*math.radians(joint_vel_j))

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(T(j - 1), J_twist_accel_i,
                                    T(j), J_twist_accel_j,
                                    J(j), J_joint_accel_j,
                                    b_accel, model)

    def joint_accel_factor(self, j_plus_1_T_j, twist_j, j):
        """
        Return factor based on joint acceleration equation of the ith link
        """
        # get mass and inertia of link j
        mass, inertia = self.link_properties(j)

        # LHS of wrench equation
        J_wrenchj = np.identity(6)
        J_wrenchj_plus_1 = -j_plus_1_T_j.AdjointMap().transpose()
        J_twist_accel_j = -utils.inertia_matrix(inertia, mass)
        # RHS of wrench equation
        b_wrench = -np.dot(np.dot(gtsam.Pose3.adjointMap(twist_j).transpose(),
                                  utils.inertia_matrix(inertia, mass)), twist_j)

        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(W(j), J_wrenchj,
                                    W(j + 1), J_wrenchj_plus_1,
                                    T(j), J_twist_accel_j,
                                    b_wrench, model)

    def wrench_factor(self, torquej, screw_axis_j, j):
        """
        Return factor based on wrench equation of the ith link
        """
        # LHS of torque equation
        J_wrenchj = np.array([screw_axis_j])
        # RHS of torque equation
        b_torque = np.array([torquej])
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.0]))
        return gtsam.JacobianFactor(W(j), J_wrenchj, b_torque, model)

    def prior_factor_base(self):
        """
        Return prior factor that twist acceleration
        on base link is assumed to be zero for
        angular and gravity accelration for linear
        """
        # LHS
        J_twist_accel_i = np.identity(6)
        # RHS
        b_twist_accel = np.array([0, 0, 0, 0, 0, 9.8])
        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(T(0),
                                    J_twist_accel_i, b_twist_accel, model)

    def prior_factor_eef(self):
        """
        Return prior factor that wrench
        on end effector is assumed to be zero
        """
        # LHS
        J_wrenchj_plus_1 = np.identity(6)
        # RHS
        b_wrench = np.zeros(6)
        model = gtsam.noiseModel_Constrained.All(6)
        return gtsam.JacobianFactor(W(self.num_links() + 1),
                                    J_wrenchj_plus_1, b_wrench, model)

    def forward_factor_graph(self, joint_angles, joint_velocities, joint_torques):
        """
        build factor graph for RR manipulator forward dynamics
        Return Gaussian factor graph
        take joint angles, joint velocities and torques as input
        """
        # Set up Gaussian Factor Graph
        gfg = gtsam.GaussianFactorGraph()

        # Add prior factor, link 0 twist acceleration = 0
        gfg.add(self.prior_factor_base())

        # Calculate screw axis for each joints expressed in its link frame
        screw_axes = self.screw_axes()

        # configuration of link frame j-1 relative to link frame j for arbitrary joint angle
        link_config = self.link_transforms(joint_angles)

        # link j-1 (base for the first iteration) twist
        twist_i = utils.vector(0, 0, 0, 0, 0, 0)

        for j in range(1, self.num_links() + 1):
            # ith link twist
            twist_j = self.link_twist_j(
                link_config[j], twist_i, joint_velocities[j], screw_axes[j])

            # factor 1
            gfg.add(self.twist_accel_factor(
                link_config[j], twist_j, joint_velocities[j], screw_axes[j], j))

            # factor 2
            gfg.add(self.joint_accel_factor(link_config[j+1], twist_j, j))

            # factor 3
            gfg.add(self.wrench_factor(joint_torques[j], screw_axes[j], j))

            # update link j-1 twist for the next iteration
            twist_i = twist_j

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
        return joint_accel_result(self.num_links(), results)
