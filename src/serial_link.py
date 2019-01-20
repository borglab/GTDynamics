"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie

We follow Lynch & Park 2017 conventions, but using j to index joints, as in Corke 2017:
    - j=0 is base
    - j in 1...N index joints
    - Mj is COM pose of link j in base frame 0.
    - Tj is the link frame j, aligned with joint j+1, expressed in base frame 0.
    - we use shorthand i==j-1 and k==j+1
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import gtsam
import numpy as np
import utils

I6 = np.identity(6)
ALL_6_CONSTRAINED = gtsam.noiseModel_Constrained.All(6)


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

    def num_links(self):
        """return number of *moving* links."""
        return len(self._links)

    def link_transforms(self, q=None):
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
        for A in self.link_transforms(q):
            t = t.compose(A)
            frames.append(t)
        return frames

    def com_frames(self, q=None):
        """ Return each link's center of mass frame at rest, in the world frame."""
        t = self._base
        frames = []
        for i, A in enumerate(self.link_transforms(q)):
            t = t.compose(A)
            iTcom = gtsam.Pose3(gtsam.Rot3(), self._links[i].center_of_mass)
            frames.append(utils.compose(t, iTcom))
        return frames

    def screw_axes(self):
        """ Return screw axes of each joint expressed in its own link frame."""
        return [link.screw_axis() for link in self._links]

    @staticmethod
    def link_twist_j(jTi, twist_i, joint_vel_j, screw_axis_j):
        """ Calculate twist fn the j^th link.
            Keyword arguments:
                jTi -- link j-1 configuration expressed in link j frame
                twist_i -- twist of link j-1
                joint_vel_j (rad/s) -- link j joint velocity
                screw_axis_j -- j^th screw axis
            Return twist of the j^th link.
        """
        # Equation 8.45 in MR, page 292
        return np.dot(jTi.AdjointMap(), twist_i) + screw_axis_j * joint_vel_j

    @staticmethod
    def twist_accel_factor(jTi, twist_j, joint_vel_j, screw_axis_j, j):
        """ Return factor based on twist acceleration equation of the j^th link.
            Keyword arguments:
                jTi -- link j-1 configuration expressed in link j frame
                twist_j -- twist of the j^th link
                joint_vel_j -- link j joint velocity
                screw_axis_j -- j^th screw axis
                j -- link index, in 1..N
        """
        # Equation 8.47 in MR, page 293

        # LHS of acceleration equation
        J_twist_accel_i = -jTi.AdjointMap()
        J_joint_accel_j = -np.reshape(screw_axis_j, (6, 1))

        # RHS of acceleration equation
        b_accel = np.dot(gtsam.Pose3.adjointMap(
            twist_j), screw_axis_j * joint_vel_j)

        return gtsam.JacobianFactor(T(j - 1), J_twist_accel_i,
                                    T(j), I6,
                                    J(j), J_joint_accel_j,
                                    b_accel, ALL_6_CONSTRAINED)

    def joint_accel_factor(self, kTj, twist_j, j):
        """ Return factor based on joint acceleration equation of the j^th link.
            Keyword arguments:
                kTj -- link j+1 configuration expressed in link j frame
                twist_j -- twist of the j^th link
                j -- link index, in 1..N
        """
        G_j = self._links[j-1].inertia_matrix()
        ad = gtsam.Pose3.adjointMap(twist_j).transpose()
        b = - np.dot(ad, np.dot(G_j, twist_j))
        jAk = kTj.AdjointMap().transpose()

        # Given the above, Equation 8.48 in MR, page 293 can be written as
        # G_j * T(j) + b == W(j) - jAk * W(j + 1)
        # OR
        # W(j) - jAk * W(j + 1) - G_j * T(j) == b

        return gtsam.JacobianFactor(W(j), I6,
                                    W(j + 1), -jAk,
                                    T(j),  -G_j,
                                    b, ALL_6_CONSTRAINED)

    @staticmethod
    def wrench_factor(torque_j, screw_axis_j, j):
        """ Return factor based on wrench equation of the j^th link
            Keyword arguments:
                torque_j -- torque at joint j
                screw_axis_j -- j^th screw axis
                j -- link index, in 1..N
        """
        # Equation 8.49 in MR, page 293

        # LHS of torque equation
        J_wrench_j = np.array([screw_axis_j])
        # RHS of torque equation
        b_torque = np.array([torque_j])
        model = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.0]))
        return gtsam.JacobianFactor(W(j), J_wrench_j, b_torque, model)

    @staticmethod
    def prior_factor_base():
        """
        Return prior factor that twist acceleration
        on base link is assumed to be zero for
        angular and gravity accelration for linear
        """
        # RHS
        b_twist_accel = np.array([0, 0, 0, 0, 0, 9.8])
        return gtsam.JacobianFactor(T(0), I6, b_twist_accel, ALL_6_CONSTRAINED)

    def prior_factor_eef(self):
        """
        Return prior factor that wrench
        on end effector is assumed to be zero
        """
        # RHS
        rhs = np.zeros(6)
        return gtsam.JacobianFactor(W(self.num_links() + 1),
                                    I6, rhs, ALL_6_CONSTRAINED)

    @staticmethod
    def jTi_list(Ms, screw_axes, joint_angles):
        """ Calculate list of transforms from COM frame j-1 relative to COM j.
            Keyword arguments:
                Ms (Pose3 list) -- COM frames at rest
                screw_axes (np.array list) -- screw axis in COM frames
                joint angles (np.array, in rad) - joint angles
            Returns list of Pose3.
        """
        # TODO(Frank): I don't like these inverses, wished we could do with forward.
        # configuration of link frame j-1 relative to link frame j at rest
        jMi_list = [M_j.between(M_i) for (M_i, M_j) in zip(Ms[:-1], Ms[1:])]

        return [gtsam.Pose3.Expmap(- A_j * q_j).compose(jMi)
                for A_j, q_j, jMi in zip(screw_axes, joint_angles, jMi_list)]

    def forward_factor_graph(self, joint_angles, joint_velocities, torques):
        """ Build factor graph for RR manipulator forward dynamics.
            Keyword arguments:
                joint angles (np.array, in rad) - joint angles
                joint velocities (np.array, in rad/s)
                torques (np.array, in Nm)
            Returns Gaussian factor graph
        """
        # TODO(Frank): take triples instead?
        N = self.num_links()
        assert len(joint_angles) == N
        assert len(joint_velocities) == N
        assert len(torques) == N

        # Set up Gaussian Factor Graph
        gfg = gtsam.GaussianFactorGraph()

        # Add prior factor, link 0 twist acceleration = 0
        gfg.add(self.prior_factor_base())

        # COM frame for all links, at rest.
        Ms = self.com_frames()

        # screw axis of each joint expressed in its own link frame.
        As = self.screw_axes()

        # configuration of link frame j-1 relative to link frame j for arbitrary joint angle
        jTi_list = self.jTi_list(Ms, As, joint_angles)

        # link j-1 (base for the first iteration) twist
        twist_i = utils.vector(0, 0, 0, 0, 0, 0)

        for i, (joint_velocity_j, torque_j, screw_axis_j) \
                in enumerate(zip(joint_velocities, torques, As)):

            j = i + 1

            # Calculate j^th link twist, Eq. 8.45
            # TODO(Frank): why is this not a factor?
            twist_j = self.link_twist_j(
                jTi_list[i], twist_i, joint_velocity_j, screw_axis_j)

            # Constrain twist acceleration, Eq. 8.47
            gfg.add(self.twist_accel_factor(
                jTi_list[i], twist_j, joint_velocity_j, screw_axis_j, j))

            # Constraint joint accelerations, Eq. 8.48
            if j < N:
                gfg.add(self.joint_accel_factor(jTi_list[i+1], twist_j, j))
            else:
                gfg.add(self.joint_accel_factor(jTi_list[i+1], twist_j, j))

            # Constraint wrench given torque, Eq. 8.49
            gfg.add(self.wrench_factor(torque_j, screw_axis_j, j))

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
