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
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


def symbol(char, index):
    """Shorthand for gtsam symbol."""
    return gtsam.symbol(ord(char), index)


def J(index):
    """Shorthand for j_index."""
    return symbol('j', index)


def T(index):
    """Shorthand for t_index."""
    return symbol('t', index)


def F(index):
    """Shorthand for F_index."""
    return symbol('F', index)


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

        # Calculate screw axes for all joints, expressed in their COM frame.
        self._screw_axes = [link.screw_axis() for link in self._links]

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
        """Return screw axes for all joints, expressed in their COM frame."""
        return self._screw_axes

    def twists(self, Ts, joint_velocities):
        """Return velocity twists for all joints, expressed in their COM frame."""
        # The first link's twist is just from the joint
        twists = [self._screw_axes[0] * joint_velocities[0]]

        # Loop over joints j>1
        for j in range(2, self.num_links()+1):
            # Equation 8.45 in MR, page 292
            twist_i = twists[-1]
            jTi = Ts[j-1].between(Ts[j-2])
            Aj = self._screw_axes[j-1]
            joint_vel_j = joint_velocities[j-1]
            twist_j = np.dot(jTi.AdjointMap(), twist_i) + Aj * joint_vel_j
            twists.append(twist_j)

        return twists

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
        G_j = self._links[j].inertia_matrix()
        ad = gtsam.Pose3.adjointMap(twist_j).transpose()
        b = - np.dot(ad, np.dot(G_j, twist_j))
        jAk = kTj.AdjointMap().transpose()

        # Given the above, Equation 8.48 in MR, page 293 can be written as
        # G_j * T(j) + b == F(j) - jAk * F(j + 1)
        # OR
        # F(j) - jAk * F(j + 1) - G_j * T(j) == b

        return gtsam.JacobianFactor(F(j), I6,
                                    F(j + 1), -jAk,
                                    T(j),  -G_j,
                                    b, ALL_6_CONSTRAINED)

    def tip_factor(self, twist_N, F_tip=ZERO6):
        """ Return factor based on joint acceleration equation of the N^th link.
            Keyword arguments:
                twist_N -- twist of the N^th link
            Note: special case of joint_accel_factor for last link.
        """
        N = self.num_links()
        G_N = self._links[N].inertia_matrix()
        ad = gtsam.Pose3.adjointMap(twist_N).transpose()
        b = - np.dot(ad, np.dot(G_N, twist_N))
        Ad = self._tool.AdjointMap().transpose()

        # Given the above, Equation 8.48 in MR, page 293 can be written as
        # G_N * T(N) + b == F(N) - Ad * F_tip
        # OR
        # F(N) - G_N * T(N) == b + Ad * F_tip

        return gtsam.JacobianFactor(F(N), I6,
                                    T(N),  -G_N,
                                    b + Ad * F_tip, ALL_6_CONSTRAINED)

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
        return gtsam.JacobianFactor(F(j), J_wrench_j, b_torque, model)

    @staticmethod
    def base_factor():
        """
        Return prior factor that twist acceleration
        on base link is assumed to be zero for
        angular and gravity accelration for linear
        """
        # RHS
        b_twist_accel = np.array([0, 0, 0, 0, 0, 9.8])
        return gtsam.JacobianFactor(T(0), I6, b_twist_accel, ALL_6_CONSTRAINED)

    def jTi_list(self, joint_angles):
        """ Calculate list of transforms from COM frame j-1 relative to COM j.
            Keyword arguments:
                joint angles (np.array, in rad) - joint angles
            Returns list of Pose3.
        """
        # TODO(Frank): I don't like these inverses, wished we could do with forward.
        Ts = self.com_frames(joint_angles)
        return [Tj.between(Ti) for Ti, Tj in zip(Ts[:-1], Ts[1:])]

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

        # # configuration of COM link frames
        Ts = self.com_frames(joint_angles)

        # Calculate all twists
        twists = self.twists(Ts, joint_velocities)

        # Set up Gaussian Factor Graph
        gfg = gtsam.GaussianFactorGraph()

        # Add factor to enforce base acceleration
        accel_0 = ZERO6
        gfg.add(self.base_factor())

        # configuration of link frame j-1 relative to link frame j for arbitrary joint angle
        jTi_list = self.jTi_list(joint_angles)

        for i, (joint_velocity_j, twist_j, torque_j, screw_axis_j) \
                in enumerate(zip(joint_velocities, twists, torques, self._screw_axes)):

            j = i + 1
            print(j)

            # Constrain twist acceleration, Eq. 8.47
            if j > 1:
                gfg.add(self.twist_accel_factor(
                    jTi_list[i], twist_j, joint_velocity_j, screw_axis_j, j))
            else:
                pass  # TODO: need 1T0

            if j < N:
                gfg.add(self.joint_accel_factor(jTi_list[i], twists[i-1], j))
            else:
                gfg.add(self.tip_factor(twist_j))

            # Constraint wrench given torque, Eq. 8.49
            gfg.add(self.wrench_factor(torque_j, screw_axis_j, j))

        return gfg

    def factor_graph_optimization(self, forward_factor_graph):
        """
        optimize factor graph for manipulator forward dynamics
        Return joint accelerations
        take factor graph for forward dynamics as input
        """
        results = forward_factor_graph.optimize()
        return joint_accel_result(self.num_links(), results)
