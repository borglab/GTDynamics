"""
@file   jr_visualizer.py
@brief  visualize the jumping trajectory of the jumping robot
@author Yetong Zhang
"""

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,currentdir) 

import gtdynamics as gtd
import gtsam

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from jumping_robot import JumpingRobot

def visualize_jr(values: gtsam.Values, jr: JumpingRobot, k: int):
    """ visualize jumping robot

    Args:
        values (gtsam.Values): all values of the time step
        jr (JumpingRobot): jumping robot
        k (int): time step to visualize
    """
    link_names = ["shank_r", "thigh_r", "torso", "thigh_l", "shank_l"]
    colors = ["red", "orange", "green", "blue", "purple"]
    # for k in range(k_start, k_end):

    fig = plt.figure(figsize=(10, 10), dpi=80)

    for name, color in zip(link_names, colors):
        link = jr.robot.link(name)
        i = link.id()
        pose = gtd.Pose(values, i, k)

        y = pose.y()
        z = pose.z()
        theta = pose.rotation().roll()
        l = 0.55
        start_y = y - l/2 * np.cos(theta)
        start_z = z - l/2 * np.sin(theta)
        end_y = y + l/2 * np.cos(theta)
        end_z = z + l/2 * np.sin(theta)

        plt.plot([start_y, end_y], [start_z, end_z], color=color)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(-1, 1)
    plt.ylim(-1, 2)
    
    plt.show()

if __name__ == "__main__":
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
    jr = JumpingRobot(yaml_file_path, JumpingRobot.create_init_config())

    from jr_simulator import JRSimulator

    init_config = JumpingRobot.create_init_config()
    controls = JumpingRobot.create_controls()
    jr_simulator = JRSimulator(yaml_file_path, init_config)

    # specify joint angles, joint vels, torques
    k = 0
    phase = 0
    values = gtsam.Values()
    for acutator in jr_simulator.jr.actuators:
        j = acutator.j
        gtd.InsertTorqueDouble(values, j, k, 0.0)
        gtd.InsertJointAngleDouble(values, j, k, 0.0)
        gtd.InsertJointVelDouble(values, j, k ,0.0)
    jr_simulator.step_robot_dynamics(k, values, phase)

    visualize_jr(values, jr, k)

