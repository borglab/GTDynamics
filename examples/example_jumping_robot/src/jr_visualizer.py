"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  jr_visualizer.py
 * @brief Visualize the jumping trajectory of the jumping robot.
 * @author Yetong Zhang
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
# import matplotlib.animation as animation

from jumping_robot import JumpingRobot

def update_jr_frame(ax, values, jr, k):
    link_names = ["shank_r", "thigh_r", "torso", "thigh_l", "shank_l"]
    colors = ["red", "orange", "green", "blue", "purple"]

    ax.clear()

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

        ax.plot([start_y, end_y], [start_z, end_z], color=color)

    for link in jr.robot.links():
        i = link.id()
        pose = link.wTcom()

        y = pose.y()
        z = pose.z()
        theta = pose.rotation().roll()
        l = 0.55
        start_y = y - l/2 * np.cos(theta)
        start_z = z - l/2 * np.sin(theta)
        end_y = y + l/2 * np.cos(theta)
        end_z = z + l/2 * np.sin(theta)

        ax.plot([start_y, end_y], [start_z, end_z], color='k', alpha=0.2)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 2)
    # return fig

def visualize_jr(values: gtsam.Values, jr: JumpingRobot, k: int):
    """ Visualize the jumping robot.

    Args:
        values (gtsam.Values): all values of the time step
        jr (JumpingRobot): jumping robot
        k (int): time step to visualize
    """

    fig = plt.figure(figsize=(10, 10), dpi=80)
    ax = fig.add_subplot(1,1,1)
    update_jr_frame(ax, values, jr, k)
    plt.show()



def visualize_jr_trajectory(values, jr, num_steps):
    fig = plt.figure(figsize=(10, 10), dpi=80)
    ax = fig.add_subplot(1,1,1)

    def animate(i):
        update_jr_frame(ax, values, jr, i)
        
    ani = FuncAnimation(fig, animate, frames=np.arange(num_steps), interval=10) 
    plt.show()

if __name__ == "__main__":
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
    jr = JumpingRobot(yaml_file_path, JumpingRobot.create_init_config())

    values = gtsam.Values()
    k = 0
    for link in jr.robot.links():
        i = link.id()
        pose = link.wTcom()
        gtd.InsertPose(values, i, k, pose)
    visualize_jr(values, jr, k)

    # visualize_jr_trajectory(0, 0, 0)
