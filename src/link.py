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

I6 = np.identity(6)
ALL_6_CONSTRAINED = gtsam.noiseModel_Constrained.All(6)
ONE_CONSTRAINED = gtsam.noiseModel_Constrained.All(1)
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


def symbol(char, j):
    """Shorthand for gtsam symbol."""
    return gtsam.symbol(ord(char), j)


def T(j):
    """Shorthand for T_j, for twist accelerations."""
    return symbol('T', j)


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

    def __init__(self, theta, d, a, alpha, joint_type, mass, center_of_mass, inertia):
        """ Constructor.
            Keyword arguments:
                d (m)                   -- link offset, i.e., distance between two joints
                theta (degrees)         -- angle between two joint frame x-axes (theta)
                a (m)                   -- link length. i.e., distance between two joints
                alpha (degrees)         -- link twist, i.e., angle between joint axes
                joint_type (char)       -- 'R': revolute,  'P' prismatic
                mass (float)            -- mass of link
                center_of_mass (Point3) -- center of mass location expressed in link frame
                inertia (vector)        -- principal inertias
            Note: angles are given in degrees, but converted to radians internally.
        """
        self._d = d
        self._theta = math.radians(theta)
        self._a = a
        self._alpha = math.radians(alpha)
        self._joint_type = joint_type
        self._mass = mass
        self._center_of_mass = center_of_mass
        self._inertia = inertia

        # Calculate screw axis expressed in center of mass frame.
        # COM is expressed in the link frame, which is aligned with the *next* joint in
        # the DH convention. Hence, we need to translate back to *our* joint:
        com = utils.vector_of_point3(self._center_of_mass)
        joint = utils.vector(-self._a, 0, 0)
        self._screw_axis = utils.unit_twist(utils.vector(0, 0, 1), joint - com)

    def A(self, q=0):
        """ Return Link transform.
            Keyword arguments:
                q -- optional generalized joint angle (default 0)
        """
        theta = q if self._joint_type == 'R' else self._theta
        d = q if self._joint_type == 'P' else self._d
        return utils.compose(
            Pose3(Rot3.Yaw(theta), Point3(0, 0, d)),
            Pose3(Rot3.Roll(self._alpha),
                  Point3(self._a, 0, 0))
        )

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
        """Return center of mass (Point3)."""
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

    @staticmethod
    def base_factor(base_twist_accel=ZERO6):
        """ Factor enforcing base acceleration.
            Keyword argument:
                base_twist_accel (np.array) -- optional acceleration for base
            Example: if you wish to model gravity forces, use
                base_twist_accel = vector(0, 0, 0, 0, 0, -9.8)
            which imparts upwards acceleration on the base, which then will be
            propagated to all links, forcing wrenches and torques to generate
            upward forces consistent with gravity compensation.
        """
        return gtsam.JacobianFactor(T(0), I6, base_twist_accel, ALL_6_CONSTRAINED)

    def forward_factors(self, j, jTi, joint_vel_j, twist_j, torque_j, kTj):
        """ Create all factors linking this links dynamics with previous and next link.
            Keyword arguments:
                j -- index for this joint
                jTi -- previous COM frame, expressed in this link's COM frame
                joint_vel_j -- joint velocity for this link
                twist_j -- velocity twist for this link, in COM frame
                torque_j - torque at this link's joint
                kTj -- this COM frame, expressed in next link's COM frame
            Will create several factors corresponding to Lynch & Park book:
                - twist acceleration, Equation 8.47, page 293
                - wrench balance, Equation 8.48, page 293
                - torque-wrench relationship, Equation 8.49, page 293
        """
        factors = GaussianFactorGraph()

        A_j = self._screw_axis  # joint axis expressed in COM frame
        ad_j = Pose3.adjointMap(twist_j)

        # Given the above Equation 8.47 can be written as
        # T(j) == A_j * a(j) + jTi.AdjointMap() * T(j-1) + ad_j * A_j * joint_vel_j
        rhs = np.dot(ad_j, A_j * joint_vel_j)
        factors.add(T(j), I6,
                    a(j), -np.reshape(A_j, (6, 1)),
                    T(j - 1), -jTi.AdjointMap(),
                    rhs, ALL_6_CONSTRAINED)

        G_j = self.inertia_matrix()
        coriolis_j = np.dot(ad_j.transpose(), np.dot(G_j, twist_j))
        jAk = kTj.AdjointMap().transpose()

        # Given the above Equation 8.48 can be written as
        # G_j * T(j) - coriolis_j == F(j) - jAk * F(j + 1)
        factors.add(T(j), G_j,
                    F(j), -I6,
                    F(j + 1), jAk,
                    coriolis_j, ALL_6_CONSTRAINED)

        # Equation 8.49 can be written as
        # A_j.transpose() * F(j).transpose() == torque_j
        tau_j = utils.vector(torque_j)
        factors.add(F(j), np.reshape(A_j, (1, 6)), tau_j, ONE_CONSTRAINED)

        return factors
