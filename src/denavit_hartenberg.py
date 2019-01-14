#!/usr/bin/env python
import numpy as np
from gtsam import Point3, Pose3, Rot3, symbol
import utils
from utils import vector

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
    parameters for a signle link
    """
    def __init__(self, joint_offset, joint_angle, joint_normal, twist_angle,
                joint_type, joint_vel, joint_accel, mass, center_of_mass,
                inertia, torque):
        """
        arguments:
        joint_offset: distance between two joints along joint axis
        joint_angle: initial angle of joint
        joint_normal: distance between two joints along common 
                      normal of two joint axises
        twist_angle: angle between joint axises
        joint_type: R:revolute
                    P:prismatic
        joint_vel: joint velocity
        joint_accel: joint acceleration
        mass: mass of link
        center_of_mass: center of mass location expressed 
                        in link frame
        inertia: principal inertias 
        torque: torque applied to joint
        """
        self.joint_offset = joint_offset
        self.joint_angle = joint_angle
        self.joint_normal = joint_normal
        self.twist_angle = twist_angle
        self.joint_type = joint_type
        self.joint_vel = joint_vel
        self.joint_accel = joint_accel
        self.mass = mass
        self.center_of_mass = center_of_mass
        self.inertia = inertia
        self.torque = torque

# #========================================================================================
# #Puma Kinematic Parameters and Inputs
# #joint offsets       0...n+1 (first element ignored)
# dh_d = np.array([0, 1, 0.2435, -0.0934, 0.4331, 0, 0.2000, 0])        
# #joint angles        0...n+1 (first element ignored) 
# dh_t = np.array([0, 1, 2, 3, 4, 5, 6, 90/5.])*( 5)*pi/180.
# #link common normals 0...n+1 (last  element ignored)
# dh_a = np.array([0, 0, 0.4318, 0.0203, 0, 0, 0, 0])           
# #link twist angles   0...n+1 (last  element ignored)
# dh_f = np.array([0, -90, 0, -90, 90, -90, 90, 0])*pi/180.  
# #{R/P/G/B/N} = {Revolute/Prismatic/Gripper/Base/None} joint type
# dh_j = np.array(['B', 'R', 'R', 'R', 'R', 'R', 'R', 'G'])      
# #joint velocities    0...n+1 (first element ignored)
# dh_td = np.array([0, 1, 2, 3, 4, 5, 6, 0])*(-5)*pi/180.
# #joint accelerations 0...n+1 (first element ignored)
# dh_tdd = np.array([0, 1, 2, 3, 4, 5, 6, 0])*(10)*pi/180.

# #Puma Dynamics Parameters
# #link masses (first and last element are ignored)
# dh_m = np.array([0, 0, 17.40, 4.80, 0.82, 0.34, 0.09, 0])
# #ith center-of-mass location in frame i 0...n+1 (first and last elements ignored)
# dh_com = np.array([[0, 0, 0.068, 0, 0, 0, 0, 0],         
#                    [0, 0, 0.006, -0.070, 0, 0, 0, 0], 
#                    [0, 0, -0.016, 0.014, -0.019, 0, 0.032, 0]])
# #ith principal inertias i 0...n+1 (first and last elements ignored)
# dh_pI = np.array([[0, 0, 0.130, 0.066, 0.0018, 0.00030, 0.00015, 0],              
#                   [0, 0, 0.524, 0.0125, 0.00180, 0.00030, 0.00015, 0],
#                   [0, 0.35, 0.539, 0.086, 0.00130, 0.00040, 0.00004, 0]])
# # torque applied on each joints (frist and last elements ignored)
# dh_tq = np.array([0, 0.626950752326773,	-34.8262338725151,	1.02920598714973,
#                     -0.0122426673731905,	0.166693973271978,	7.20736555357164e-05, 0])
# #========================================================================================

# #========================================================================================
# #RR manipulator Kinematic Parameters and Inputs
# #joint offsets       0...n+1 (first element ignored)
# dh_d = np.array([0, 0, 0, 0])          
# #joint angles        0...n+1 (first element ignored) 
# dh_t = np.array([0, 0, 0, 0])
# #link common normals 0...n+1 (last  element ignored)
# dh_a = np.array([0, 2, 2, 0])         
# #link twist angles   0...n+1 (last  element ignored)
# dh_f = np.array([0, 0, 0, 0])
# #{R/P/G/B/N} = {Revolute/Prismatic/Gripper/Base/None} joint type
# dh_j = np.array(['B', 'R', 'R', 'G'])      

# #joint velocities    0...n+1 (first element ignored)
# dh_td = np.array([0, 1, 1, 0])
# #joint accelerations 0...n+1 (first element ignored)
# dh_tdd = np.array([0, 0, 0, 0])

# #Puma Dynamics Parameters
# #link masses (first and last element are ignored)
# dh_m = np.array([0, 1, 1, 1, 1, 1, 1, 0])
# #ith center-of-mass location in frame i 0...n+1 (first and last elements ignored)
# dh_com = np.array([[0, 1, 1, 0],         
#                    [0, 0, 0, 0], 
#                    [0, 0, 0, 0]])
# #ith principal inertias i 0...n+1 (first and last elements ignored)
# dh_pI = np.array([[0, 0, 0, 0],              
#                   [0, 1/6., 1/6., 0],
#                   [0, 1/6., 1/6., 0]])
# # torque applied on each joints (frist and last elements ignored)
# dh_tq = np.array([0, 0, 0, 0])    
# #========================================================================================