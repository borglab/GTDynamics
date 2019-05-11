"""
dh parameter link class, inheritate from Link class
taking Denavit-Hartenberg parameters.
Author: Frank Dellaert and Mandy Xie
"""
import math
import utils
from link import Link
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3

class DH_Link(Link):
    """Link taking Denavit-Hartenberg parameters"""
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
                inertia (matrix)        -- general inertias
            Note: angles are given in degrees, but converted to radians internally.
        """
        self._d = d
        self._theta = math.radians(theta)
        self._a = a
        self._alpha = math.radians(alpha)

        # Calculate screw axis expressed in center of mass frame.
        # COM is expressed in the link frame, which is aligned with the *next* joint in
        # the DH convention. Hence, we need to translate back to *our* joint:
        com = utils.vector_of_point3(center_of_mass)
        joint = utils.vector(-self._a, 0, 0)
        screw_axis = utils.unit_twist(utils.vector(0, math.sin(self._alpha), 
                            math.cos(self._alpha)), joint - com)

        center_of_mass_pose3 = Pose3(Rot3(), center_of_mass)
        Link.__init__(self, joint_type, mass, center_of_mass_pose3, inertia, screw_axis)

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