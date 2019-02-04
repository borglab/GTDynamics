"""
URDF parameter link class, inheritate from Link class
taking universal robot discription format parameters.
Author: Frank Dellaert and Mandy Xie
"""
import math
import utils
from link import Link
from gtsam import GaussianFactorGraph, Point3, Pose3, Rot3
import numpy as np


class URDF_Link(Link):
    """Link taking universal robotics discription format parameters"""

    def __init__(self, origin, axis, joint_type, mass, center_of_mass, inertia):
        """ Constructor.
            Keyword arguments:
                origin (Pose3)          -- the x-y-z and roll-pitch-yaw coords of link frame w.r.t.
                                           the former link frame 
                axis (vecotr)           -- the x-y-z unit vector along the rotation axis in the link frame
                joint_type (char)       -- 'R': revolute,  'P' prismatic
                mass (float)            -- mass of link
                center_of_mass (Pose3)  -- the position and orientation of the center of mass frame w.r.t.
                                           joint frame
                inertia (vector)        -- principal inertias
            Note: angles are given in degrees, but converted to radians internally.
        """
        self._origin = origin
        self._axis = axis

        # Calculate screw axis expressed in center of mass frame.
        center_of_mass_com = center_of_mass.inverse()
        # joint axis expressed in com frame
        # TODO: need to add operator*(const Unit3& p) const for cython
        joint_axis_com = utils.vector_of_point3(
            center_of_mass_com.rotation().rotate(utils.point3_of_vector(axis)))
        # point on joint axis expressed in com frame
        com = utils.vector_of_point3(center_of_mass_com.translation())
        screw_axis = utils.unit_twist(joint_axis_com, com)

        Link.__init__(self, joint_type, mass, center_of_mass, inertia, screw_axis)

    def A(self, q=0):
        """ Return Link transform.
            Keyword arguments:
                q -- optional generalized joint angle (default 0)
        """
        if self._joint_type == 'R':
            # TODO: need to add Rot3.AxisAngle(self._axis, q) for cython
            if np.array_equal(self._axis, utils.vector(1, 0, 0)):
                rotation = Rot3.Rx(q)
            else:
                if np.array_equal(self._axis, utils.vector(0, 1, 0)):
                    rotation = Rot3.Ry(q)
                else:
                    rotation = Rot3.Rz(q)
            return utils.compose(
                self._origin,
                Pose3(rotation, Point3()))
        else:
            return utils.compose(
                self._origin,
                Pose3(Rot3(), utils.point3_of_vector(self._axis*q))
            )
