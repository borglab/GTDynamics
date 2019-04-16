"""
Link class taking Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401, C0412

import math

import gtsam
import numpy as np
import utils
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3

I1 = np.identity(1)
I6 = np.identity(6)
ALL_6_CONSTRAINED = gtsam.noiseModel_Constrained.All(6)
ONE_CONSTRAINED = gtsam.noiseModel_Constrained.All(1)
ZERO = utils.vector(0)
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


def symbol(char, j):
    """Shorthand for gtsam symbol."""
    return gtsam.symbol(ord(char), j)


def V(j):
    """Shorthand for V_j, for 6D link twist vectors."""
    return symbol('V', j)


def T(j):
    """Shorthand for T_j, for twist accelerations."""
    return symbol('T', j)


def t(j):
    """Shorthand for t_j, for torque."""
    return symbol('t', j)


def a(j):
    """Shorthand for a_j, for joint accelerations."""
    return symbol('a', j)


def F(j):
    """Shorthand for F_j, for wrenches."""
    return symbol('F', j)


class Link(object):
    """
    parameters for a single link
    """

    def __init__(self, joint_type, mass, center_of_mass, inertia, screw_axis):
        """ Constructor.
            Keyword arguments:
                joint_type (char)       -- 'R': revolute,  'P' prismatic
                mass (float)            -- mass of link
                center_of_mass (Pose3)  -- the center of mass frame w.r.t. link frame
                inertia (vector)        -- principal inertias
                screw_axis (vector)     -- screw axis expressed in center of mass frame
            Note: angles are given in degrees, but converted to radians internally.
        """
        self._joint_type = joint_type
        self._mass = mass
        self._center_of_mass = center_of_mass
        self._inertia = inertia
        self._screw_axis = screw_axis
        
    @property
    def screw_axis(self):
        """Return screw axis expressed in center of mass frame."""
        return self._screw_axis

    @property
    def mass(self):
        """Return link mass."""
        return self._mass

    @property
    def center_of_mass(self):
        """Return center of mass (Pose3)."""
        return self._center_of_mass

    @property
    def inertia(self):
        """Return link moments of inertia."""
        return self._inertia

    def inertia_matrix(self):
        """Return the general mass matrix"""
        gmm = np.zeros((6, 6), np.float)
        gmm[:3, :3] = np.diag(self.inertia)
        gmm[3:, 3:] = self.mass * np.identity(3)
        return gmm
    
    def A(self, q=0):
        """ overwrite this method """
        raise NotImplementedError

    @staticmethod
    def base_factor(base_twist_accel=ZERO6):
        """ Factor enforcing base acceleration.
            Keyword argument:
                base_twist_accel (np.array) -- optional acceleration for base
        """
        return gtsam.JacobianFactor(T(0), I6, base_twist_accel, ALL_6_CONSTRAINED)

    @staticmethod
    def tool_factor(N, external_wrench=ZERO6):
        """ Factor enforcing external wrench at tool frame.
            Keyword argument:
                N -- number of links, used to create wrench index
                external_wrench (np.array) -- optional external wrench 
        """
        # Note: F(N+1) is the negative of the external wrench applied to the tool.
        # The reason is the negative sign in Formula 8.48, which is correct when a
        # link provides a reaction wrench to the next link, but should be positive
        # for an external wrench applied to the same link.
        return gtsam.JacobianFactor(F(N+1), I6, -external_wrench, ALL_6_CONSTRAINED)

    def twist_factor(self, j, jTi, joint_vel_j):
        """ Create single factor relating this link's twist with previous one.
            Keyword arguments:
                j -- index for this joint
                jTi -- previous COM frame, expressed in this link's COM frame
                joint_vel_j -- joint velocity for this link
        """
        A_j = self._screw_axis
        joint_twist = A_j * joint_vel_j

        if j == 1:
            return gtsam.JacobianFactor(V(j), I6, joint_twist, ALL_6_CONSTRAINED)
        else:
            # Equation 8.45 in MR, page 292
            # V(j) - np.dot(jTi.AdjointMap(), V(j-1)) == joint_twist
            return gtsam.JacobianFactor(V(j), I6, V(j-1), -jTi.AdjointMap(), joint_twist, ALL_6_CONSTRAINED)

    def wrench_factor(self, j, twist_j, kTj, gravity=None):
        """ Create wrench balance factor, common between forward and inverse dynamics.
            Keyword arguments:
                j -- index for this joint
                twist_j -- velocity twist for this link, in COM frame
                kTj -- this COM frame, expressed in next link's COM frame
                gravity (np.array) -- if given, will create gravity force. In link COM frame.
        """
        # Wrench on this link is due to acceleration and reaction to next link.
        # We need inertia, coriolis forces, and an Adjoint map:
        ad_j = Pose3.adjointMap(twist_j)
        G_j = self.inertia_matrix()
        rhs = np.dot(ad_j.transpose(), np.dot(G_j, twist_j))  # coriolis
        if gravity is not None:
            rhs[3:] += gravity * self.mass
        jAk = kTj.AdjointMap().transpose()
        # Given the above Equation 8.48 can be written as
        # G_j * T(j) - F(j) + jAk * F(j + 1) == coriolis_j [+ gravity]
        return gtsam.JacobianFactor(T(j), G_j,
                                    F(j), -I6,
                                    F(j + 1), jAk,
                                    rhs, ALL_6_CONSTRAINED)

    def forward_factors(self, j, jTi, joint_vel_j, twist_j, torque_j, kTj, gravity=None):
        """ Create all factors linking this links dynamics with previous and next link.
            Keyword arguments:
                j -- index for this joint
                jTi -- previous COM frame, expressed in this link's COM frame
                joint_vel_j -- joint velocity for this link
                twist_j -- velocity twist for this link, in COM frame
                torque_j - torque at this link's joint
                kTj -- this COM frame, expressed in next link's COM frame
                gravity (np.array) -- if given, will create gravity force. In link COM frame.
            Will create several factors corresponding to Lynch & Park book:
                - twist acceleration, Equation 8.47, page 293
                - wrench balance, Equation 8.48, page 293
                - torque-wrench relationship, Equation 8.49, page 293
        """
        factors = GaussianFactorGraph()

        # Twist acceleration in this link as a function of previous and joint accel.
        # We need to know our screw axis, and an adjoint map:
        A_j = self._screw_axis
        ad_j = Pose3.adjointMap(twist_j)
        # Given the above Equation 8.47 can be written as
        # T(j) - A_j * a(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j
        rhs = np.dot(ad_j, A_j * joint_vel_j)
        factors.add(T(j), I6,
                    a(j), -np.reshape(A_j, (6, 1)),
                    T(j - 1), -jTi.AdjointMap(),
                    rhs, ALL_6_CONSTRAINED)

        # Wrench on this link is due to acceleration and reaction to next link.
        factors.push_back(self.wrench_factor(j, twist_j, kTj, gravity))

        # Torque is always wrench projected on screw axis.
        # Equation 8.49 can be written as
        # A_j.transpose() * F(j).transpose() == torque_j
        tau_j = utils.vector(torque_j)
        factors.add(F(j), np.reshape(A_j, (1, 6)), tau_j, ONE_CONSTRAINED)

        return factors

    def inverse_factors(self, j, jTi, joint_vel_j, twist_j, acceleration_j, kTj, gravity=None):
        """ Create all factors linking this links dynamics with previous and next link.
            Keyword arguments:
                j -- index for this joint
                jTi -- previous COM frame, expressed in this link's COM frame
                joint_vel_j -- joint velocity for this link
                twist_j -- velocity twist for this link, in COM frame
                acceleration_j - acceleration at this link's joint
                kTj -- this COM frame, expressed in next link's COM frame
                gravity (np.array) -- if given, will create gravity force. In link COM frame.
            Will create several factors corresponding to Lynch & Park book:
                - twist acceleration, Equation 8.47, page 293
                - wrench balance, Equation 8.48, page 293
                - torque-wrench relationship, Equation 8.49, page 293
        """
        factors = GaussianFactorGraph()

        # Twist acceleration in this link as a function of previous and joint accel.
        # We need to know our screw axis, and an adjoint map:
        A_j = self._screw_axis
        ad_j = Pose3.adjointMap(twist_j)
        # Given the above Equation 8.47 can be written as
        # T(j) - jTi.AdjointMap() * T(j-1) == ad_j * A_j * joint_vel_j  + A_j * acceleration_j
        rhs = np.dot(ad_j, A_j * joint_vel_j) + A_j * acceleration_j
        factors.add(T(j), I6,
                    T(j - 1), -jTi.AdjointMap(),
                    rhs, ALL_6_CONSTRAINED)

        # Wrench on this link is due to acceleration and reaction to next link.
        factors.push_back(self.wrench_factor(j, twist_j, kTj, gravity))

        # Torque is always wrench projected on screw axis.
        # Equation 8.49 can be written as
        # A_j.transpose() * F(j).transpose() - t(j) == 0
        factors.add(F(j), np.reshape(A_j, (1, 6)),
                    t(j), -I1, ZERO, ONE_CONSTRAINED)

        return factors
