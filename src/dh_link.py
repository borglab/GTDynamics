"""
dh parameter link class, inheritate from Link class
taking Denavit-Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""
import math

import numpy as np
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3

import utils
from link import Link

Z33 = np.diag([0, 0, 0])  # zero inertia matrix


class DH_Link(Link):
    """Link taking Denavit-Hartenberg parameters"""

    def __init__(self, theta, d, a, alpha, joint_type: str,
                 mass=0, com=Point3(0, 0, 0), inertia=Z33):
        """ Constructor.
        Arguments:
            theta (degrees)         -- angle between two joint frame x-axes (theta)
            d (m)                   -- link offset, i.e., distance between two joints
            a (m)                   -- link length. i.e., distance between two joints
            alpha (degrees)         -- link twist, i.e., angle between joint axes
            joint_type (char)       -- 'R': revolute,  'P' prismatic
            mass (float)            -- mass of link
            com (Point3)            -- center of mass location expressed in link frame
            inertia (matrix)        -- general inertias
        Note: angles are given in degrees, but converted to radians internally.
        A DH link implements A[d,theta,a,alpha] = Z[theta,d]X[a,alpha], see Wikipedia.
        """
        self._d = d
        self._theta = math.radians(theta)
        self._a = a
        self._alpha = math.radians(alpha)

        # Calculate screw axis expressed in center of mass frame.
        # COM is expressed in the link frame, which is aligned with the *next* joint in
        # the DH convention. Hence, we need to translate back to *our* joint:
        com_vector = utils.vector_of_point3(com)
        joint = utils.vector(-self._a, 0, 0)
        screw_axis = utils.unit_twist(utils.vector(0, math.sin(self._alpha),
                                                   math.cos(self._alpha)), joint - com_vector)

        center_of_mass_pose3 = Pose3(Rot3(), com)
        Link.__init__(self, joint_type, mass,
                      center_of_mass_pose3, inertia, screw_axis)

    @classmethod
    def revolute(self, d, a, alpha, **kwargs):
        """ Create a link attached to a revolute joint.

        Arguments:
            d (m)               -- link offset, i.e., distance between two joints
            a (m)               -- link length. i.e., distance between two joints
            alpha (degrees)     -- link twist, i.e., angle between joint axes
        Keyword arguments:
            **kwargs            -- optional mass, COM, inertia matrix
        """
        return DH_Link(0, d, a, alpha, "R", **kwargs)

    def A(self, q=0):
        """ Return Link transform.
            Keyword arguments:
                q -- optional generalized joint angle (default 0)
        """
        theta = q if self._joint_type == 'R' else self._theta
        d = q if self._joint_type == 'P' else self._d
        return utils.compose(
            Pose3(Rot3.Rz(theta), Point3(0, 0, d)),
            Pose3(Rot3.Rx(self._alpha), Point3(self._a, 0, 0))
        )
