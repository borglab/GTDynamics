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
from urdf_parser_py.urdf import URDF
from serial_link import SerialLink


class URDF_Link(Link):
    """Link taking universal robotics discription format parameters"""

    def __init__(self, origin, axis, joint_type, mass, center_of_mass, inertia):
        """ Constructor.
            Keyword arguments:
                origin (Pose3)          -- the x-y-z and roll-pitch-yaw coords of link frame w.r.t.
                                           the former link frame
                axis (vector)           -- the x-y-z unit vector along the rotation axis in the link frame
                joint_type (char)       -- 'R': revolute,  'P' prismatic
                mass (float)            -- mass of link
                center_of_mass (Pose3)  -- the center of mass frame w.r.t. link frame
                inertia (matrix)        -- general inertias
            Note: angles are given in degrees, but converted to radians internally.
        """
        self._origin = origin
        self._axis = axis

        # Calculate screw axis expressed in center of mass frame.
        # link frame w.r.t. com frame
        link_com = center_of_mass.inverse()
        # joint axis expressed in com frame
        # TODO: need to add operator*(const Unit3& p) const for cython
        joint_axis_com = utils.vector_of_point3(
            link_com.rotation().rotate(utils.point3_of_vector(axis)))
        # point on joint axis expressed in com frame
        point_on_axis = utils.vector_of_point3(link_com.translation())
        screw_axis = utils.unit_twist(joint_axis_com, point_on_axis)

        Link.__init__(self, joint_type, mass,
                      center_of_mass, inertia, screw_axis)

    @staticmethod
    def FromLinkSpec(link, parent_map, joints):
        """ Create link from urdf specs.
            Arguments:
                link: URDF spec for the link
                parent_map: robot topology from URDF file
                joints (dict): named joint specs from URDF file
            Returns
                URDF_Link
                parent (str)
        """
        I = link.inertial.inertia
        inertia_matrix = np.array(
            [[I.ixx, I.ixy, I.ixz], [I.ixy, I.iyy, I.iyz], [I.ixz, I.iyz, I.izz]])

        mass = link.inertial.mass
        center_of_mass = compose_pose(
            link.inertial.origin.rpy, link.inertial.origin.xyz)

        # find the joint attached to the link
        if link.name in parent_map:
            joint_name, parent_name = parent_map[link.name]
            joint = joints[joint_name]
            joint_type = 'R' if joint.joint_type in [
                'revolute', 'continuous'] else 'P'
            # TODO(yetong): special case for fixed joint, combine both links as the same link, or set the joint angle to be fixed
            axis = utils.vector(0, 0, 1) if joint.axis is None else utils.vector(
                joint.axis[0], joint.axis[1], joint.axis[2])
            origin = compose_pose(joint.origin.rpy, joint.origin.xyz)
        else:  # base link
            parent_name = None
            joint_type = 'R'
            axis = utils.vector(0, 0, 1)
            origin = compose_pose([0, 0, 0], [0, 0, 0])

        # create the link
        urdf_link = URDF_Link(origin, axis, joint_type,
                              mass, center_of_mass, inertia_matrix)
        return [urdf_link, parent_name]

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
                Pose3(Rot3(), utils.point3_of_vector(self._axis * q))
            )


def compose_pose(rpy, xyz):
    """ Return Pose.
        Keyword arguments:
            rpy -- 3 dimensional vector representing rotation
            xyz -- 3 dimensional vector representing position
    """

    # TODO(yetong): construct the rotation using rpy values
    # rotation = Rot3.rpy(rpy[0], rpy[1], rpy[2])
    rotation = Rot3()
    point = Point3(xyz[0], xyz[1], xyz[2])
    return Pose3(rotation, point)


def read_urdf(file_name):
    """ Return a dictionary of {link_name: [URDFLink, parent_name]}
        Keyword arguments:
            file_name -- file path for the urdf file to read
    """

    with open(file_name, "r") as f:
        urdf_contents = f.read()
        robot = URDF.from_xml_string(urdf_contents)

    joints = {joint.name: joint for joint in robot.joints}
    return {link.name: URDF_Link.FromLinkSpec(link, robot.parent_map, joints)
            for link in robot.links if link.inertial is not None}
