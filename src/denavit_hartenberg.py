"""
Denavit Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie

We follow Lynch & Park 2017 conventions, but using j to index joints, as in Corke 2017: 
    - j=0 is base
    - j \in 1...N index joints
    - Mj is COM pose of link j in base frame 0.
    - Tj is the link frame j, aligned with joint j+1, expressed in base frame 0.
    - we use i to denote the previous link/joint with index j-1
"""

# pylint: disable=C0103, E1101, E0401

import math

import utils
from gtsam import Point3, Pose3, Rot3


class Link(object):
    """
    parameters for a single link
    """

    def __init__(self, theta, d, a, alpha, joint_type, mass, center_of_mass, inertia):
        """ Constructor.
            Keyword arguments:
                d (float)               -- link offset, i.e., distance between two joints along joint axis
                theta (float)           -- angle between two joint frame x-axes (theta)
                a (float)               -- link length. i.e., distance between two joints
                alpha (float)           -- link twist, i.e., angle between joint axes
                joint_type (char)       -- 'R': revolute,  'P' prismatic
                mass (float)            -- mass of link
                center_of_mass (Point3) -- center of mass location expressed in link frame
                inertia (vector)        -- principal inertias
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
            Pose3(Rot3.Yaw(theta), Point3(0, 0, d)),
            Pose3(Rot3.Roll(self._alpha), Point3(self._a, 0, 0))
        )

    def screw_axis(self):
        """Return screw axis expressed in link frame."""
        return utils.unit_twist(utils.vector(0, 0, 1),
                                -utils.vector_of_point3(self.center_of_mass))

    def properties(self):
        """Return link mass and inertia."""
        return self._mass, self._inertia


class DenavitHartenberg(object):
    """
    Denavit-Hartenberg labeling parameters for manipulators.
    """

    def __init__(self, link_parameters, base=Pose3(), tool=Pose3()):
        """Construct from list of Link, 2 more than # joints"""
        self._links = link_parameters
        self._base = base
        self._tool = tool

    def _link_frames_from(self, j, Ti):
        """
        Return all joint frames from joint j onwards.
        Keyword arguments:
            j -- joint index in [1..N]
            Ti -- joint frame j-1.
        """
        N = self.num_of_links()
        assert j > 0 and j <= N

        iTj = self._links[j-1].A()
        Tj = Ti.compose(iTj)
        if j == N:
            return [Tj]

        return [Tj] + self._link_frames_from(j+1, Tj)

    def link_frames(self):
        """ Return each link frame at home position expressed in base frame 0.
            Note that frame Tj is aligned with the joint axis of joint j+1 
            according to the Denavit-Hartenberg convention.
        """
        return self._link_frames_from(1, Pose3())

    def com_frames(self):
        """ Return each link frame (origin at center of mass) at home position
            expressed in base frame 0.
        """
        return []  # TODO(Frank): fix

    def screw_axes(self):
        """
        return screw axis of each joints expressed in its own link frame
        """
        return [link.screw_axis() for link in self._links]

    def num_of_links(self):
        """return number of *moving* links."""
        return len(self._links)

    def link_properties(self, j):
        """return link mass and inertia, take link index as input."""
        return self._links[j].properties()
