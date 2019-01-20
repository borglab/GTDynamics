"""
Denavit Hartenberg parameters.
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
ZERO6 = utils.vector(0, 0, 0, 0, 0, 0)


def symbol(char, j):
    """Shorthand for gtsam symbol."""
    return gtsam.symbol(ord(char), j)


def a(j):
    """Shorthand for a_j, for joint accelerations."""
    return symbol('a', j)


def V(j):
    """Shorthand for V_j, for velocity twists."""
    return symbol('V', j)


def A(j):
    """Shorthand for A_j, for twist accelerations."""
    return symbol('A', j)


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

    def forward_factors(self, j, jTi, joint_vel_j):
        """ Create all factors linking this links dynamics with previous and next link.
            Keyword arguments:
                j -- index for this joint
                jTi -- previous COM frame, expresse din this joint
                joint_vel_j -- joint velocity for this link
            Will create several factors corresponding to Lynch & Park book:
                - twist factor, Equation 8.45, page 292
        """
        factors = GaussianFactorGraph()

        A_j = self._screw_axis  # joint axis expressed in COM frame

        # twist_j - jTi.AdjointMap() * twist_i = A_j * joint_vel_j
        factors.add(V(j), I6, V(j-1), - jTi.AdjointMap(),
                    A_j * joint_vel_j, ALL_6_CONSTRAINED)

        return factors
