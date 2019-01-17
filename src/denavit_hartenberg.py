#!/usr/bin/env python
import numpy as np

import utils
from gtsam import Point3, Pose3, Rot3, symbol
from utils import vector


class LinkParameters(object):
    """
    parameters for a single link
    """

    def __init__(self, joint_offset, joint_angle, joint_normal, twist_angle,
                 joint_type,  mass, center_of_mass, inertia):
        """
        Construct from arguments:
            joint_offset (float)   : distance between two joints along joint axis
            joint_angle (float)    : initial angle of joint
            joint_normal (float)   : distance between two joints along common 
                                     normal of two joint axises
            twist_angle (float)    : angle between joint axises
            joint_type (char)      : R:revolute
                                     P:prismatic
            mass (float)           : mass of link
            center_of_mass (Point3): center of mass location expressed 
                                     in link frame
            inertia (vector)       : principal inertias 
        """
        self.joint_offset = joint_offset
        self.joint_angle = utils.degrees_to_radians(joint_angle)
        self.joint_normal = joint_normal
        self.twist_angle = utils.degrees_to_radians(twist_angle)
        self.joint_type = joint_type
        self.mass = mass
        self.center_of_mass = center_of_mass
        self.inertia = inertia

    def screw_axis(self):
        """
        return screw axis expressed in link frame
        """
        return utils.unit_twist(vector(0, 0, 1), -utils.vector_of_point3(self.center_of_mass))


class DenavitHartenberg(object):
    """
    Denavit-Hartenberg labeling parameters for manipulators
    """

    def __init__(self, link_parameters):
        """Construct from list of LinkParameters, 2 more than # joints"""
        self._links = link_parameters

    # TODO: modify this function
    def _link_configuration_from(self, i, frame_joint_i_minus_1):
        """
        recursively call this function to get all link configurations
        return all link configurations starting from joint i>0, 
        takes previous frame as input.
        """
        if i > self.num_of_links():
            return []
        else:
            # link i joint frame expressed in space frame s and
            # link i com frame expressed in space frame s
            frame_joint_i, frame_i = calculate_frame_i(frame_joint_i_minus_1,
                                                       self._links[i -
                                                                   1].twist_angle,
                                                       self._links[i -
                                                                   1].joint_normal,
                                                       self._links[i].joint_angle,
                                                       self._links[i].joint_offset,
                                                       self._links[i].center_of_mass)
            return [frame_i] + self._link_configuration_from(i+1, frame_joint_i)

    def link_configuration_home(self):
        """
        return each link frame (origin at center of mass) at home position
        expressed in base
        """
        # link 0 (base) frame expressed in base frame
        frame_0 = Pose3()
        # link 0 (base) joint frame expressed in base frame
        frame_joint_0 = Pose3()
        return [frame_0] + self._link_configuration_from(1, frame_joint_0)

    def screw_axes(self):
        """
        return screw axis of each joints expressed in its own link frame
        """
        return [link.screw_axis() for link in self._links]

    def num_of_links(self):
        """return number of *moving* links."""
        return len(self._links) - 2

    def link_properties(self, i):
        """return link mass and inertia, take link index as input"""
        return (self._links[i].mass, self._links[i].inertia)


def calculate_frame_i(frame_joint_i_minus_1, twist_angle,
                      joint_normal, joint_angle, joint_offset, center_of_mass):
    """
    Return link i joint frame expressed base frame and 
    link i com frame expressed in space frame s 
    Takes previous joint frame and 
    denavit_hartenberg parameters as inputs 
    """
    # link i joint frame expressed in link i-1 joint frame
    joint_i_minus_1_frame_joint_i = utils.compose(Pose3(Rot3.Roll(twist_angle), Point3(joint_normal, 0, 0)),
                                                  Pose3(Rot3.Yaw(joint_angle), Point3(0, 0, joint_offset)))
    # link i joint frame expressed in space frame s
    frame_joint_i = utils.compose(
        frame_joint_i_minus_1, joint_i_minus_1_frame_joint_i)
    # link i com frame expressed in space frame s
    frame_i = utils.compose(frame_joint_i, Pose3(Rot3(), center_of_mass))
    return (frame_joint_i, frame_i)
