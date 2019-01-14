#!/usr/bin/env python
from math import pi
import numpy as np
from gtsam import Point3, Pose3, Rot3, symbol
import utils
from utils import vector

def degrees_to_radians(degrees):
    """convert degrees to radians"""
    return degrees * pi / 180. 

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
    frame_joint_i = utils.compose(frame_joint_i_minus_1, joint_i_minus_1_frame_joint_i)
    # link i com frame expressed in space frame s 
    frame_i = utils.compose(frame_joint_i, Pose3(Rot3(), Point3(center_of_mass[0],
                                                                center_of_mass[1], 
                                                                center_of_mass[2])))
    return (frame_joint_i, frame_i)

def screw_axis_for_one_link(link):
    """
    return screw axis expressed in link frame
    take link parameter as input
    """
    return utils.unit_twist(vector(0, 0, 1), vector(-link.center_of_mass[0],
                                                    -link.center_of_mass[1], 
                                                    -link.center_of_mass[2]))    

class DenavitHartenberg(object):
    """
    Denavit-Hartenberg labeling parameters
    and input values for manipulators
    """
    def __init__(self, link_parameters, num_of_links):
        self._link_parameters = link_parameters
        self._num_of_links = num_of_links
        
    def _link_configuration_from(self, i, frame_joint_i_minus_1):
        """
        return all frames starting from joint i>0, 
        takes previous frame as input.
        """
        if i > self._num_of_links + 1:
            return []
        else: 
            # link i joint frame expressed in space frame s and 
            # link i com frame expressed in space frame s 
            frame_joint_i, frame_i = calculate_frame_i(frame_joint_i_minus_1, 
                                                self._link_parameters[i-1].twist_angle,
                                                self._link_parameters[i-1].joint_normal,
                                                self._link_parameters[i].joint_angle,
                                                self._link_parameters[i].joint_offset,
                                                self._link_parameters[i].center_of_mass)
            return [frame_i] + self._link_configuration_from(i+1, frame_joint_i)
        

    def link_configuration(self):
        """
        return each link frame (origin at center of mass) 
        expressed in base frame
        """ 
        # link 0 com frame expressed in base frame
        frame_0 = Pose3()
        # link 0 joint frame expressed in base frame
        frame_joint_0 = Pose3()
        return [frame_0] + self._link_configuration_from(1, frame_joint_0)

    def screw_axis(self):
        """
        return screw axis of each joints expressed in its own link frame
        """
        return [screw_axis_for_one_link(link) for link in self._link_parameters]

class LinkParameters(object):
    """
    parameters for a single link
    """
    def __init__(self, joint_offset, joint_angle, joint_normal, twist_angle,
                joint_type, joint_vel, joint_accel, mass, center_of_mass,
                inertia, torque):
        """
        Construct from arguments:
            joint_offset (float)  : distance between two joints along joint axis
            joint_angle (float)   : initial angle of joint
            joint_normal (float)  : distance between two joints along common 
                                    normal of two joint axises
            twist_angle (float)   : angle between joint axises
            joint_type (char)     : R:revolute
                                    P:prismatic
            joint_vel (float)     : joint velocity
            joint_accel (float)   : joint acceleration
            mass (float)          : mass of link
            center_of_mass (float): center of mass location expressed 
                                    in link frame
            inertia (float)       : principal inertias 
            torque (float)        : torque applied to joint
        """
        self.joint_offset = joint_offset
        self.joint_angle = degrees_to_radians(joint_angle) 
        self.joint_normal = joint_normal
        self.twist_angle = degrees_to_radians(twist_angle)
        self.joint_type = joint_type
        self.joint_vel = degrees_to_radians(joint_vel)
        self.joint_accel = degrees_to_radians(joint_accel)
        self.mass = mass
        self.center_of_mass = center_of_mass
        self.inertia = inertia
        self.torque = torque
