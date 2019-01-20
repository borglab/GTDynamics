"""
Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

import math

import gtsam
import numpy as np
import utils


class Link(object):
    """
    parameters for a single link
    """

    def __init__(self, theta, d, a, alpha, joint_type, mass, center_of_mass, inertia):
        """ Constructor.
            Keyword arguments:
                d (m)                   -- link offset, i.e., distance between two joints along joint axis
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

    def A(self, q=0):
        """ Return Link transform.
            Keyword arguments:
                q -- optional generalized joint angle (default 0)
        """
        theta = q if self._joint_type == 'R' else self._theta
        d = q if self._joint_type == 'P' else self._d
        return utils.compose(
            gtsam.Pose3(gtsam.Rot3.Yaw(theta), gtsam.Point3(0, 0, d)),
            gtsam.Pose3(gtsam.Rot3.Roll(self._alpha),
                        gtsam.Point3(self._a, 0, 0))
        )

    def screw_axis(self):
        """Return screw axis expressed in link frame."""
        com = utils.vector_of_point3(self._center_of_mass)
        return utils.unit_twist(utils.vector(0, 0, 1), - com)

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
