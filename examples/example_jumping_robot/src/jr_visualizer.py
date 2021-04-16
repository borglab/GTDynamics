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

from jumping_robot import JumpingRobot, Actuator

def update_jr_frame(ax, values, jr, k):
    link_names = ["shank_r", "thigh_r", "torso", "thigh_l", "shank_l"]
    colors = ["red", "orange", "black", "green", "blue"]

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

    # for link in jr.robot.links():
    #     i = link.id()
    #     pose = link.wTcom()

    #     y = pose.y()
    #     z = pose.z()
    #     theta = pose.rotation().roll()
    #     l = 0.55
    #     start_y = y - l/2 * np.cos(theta)
    #     start_z = z - l/2 * np.sin(theta)
    #     end_y = y + l/2 * np.cos(theta)
    #     end_z = z + l/2 * np.sin(theta)

    #     ax.plot([start_y, end_y], [start_z, end_z], color='k', alpha=0.2)

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



def visualize_jr_trajectory(values, jr, num_steps, step=1):
    fig = plt.figure(figsize=(10, 10), dpi=80)
    ax = fig.add_subplot(1,1,1)

    def animate(i):
        update_jr_frame(ax, values, jr, i)
        
    ani = FuncAnimation(fig, animate, frames=np.arange(0, num_steps, step), interval=10) 
    plt.show()


def make_plot(values, jr, num_steps):
    """ Draw plots of all quantities with time. """
    joint_names = ["knee_r", "hip_r", "hip_l", "knee_l"]
    colors = {"knee_r": "red",
              "hip_r": "orange",
              "hip_l": "green",
              "knee_l": "blue",
              "source": "black"}

    qs_dict = {name:[] for name in joint_names}
    vs_dict = {name:[] for name in joint_names}
    torques_dict = {name:[] for name in joint_names}
    pressures_dict = {name:[] for name in joint_names}
    masses_dict = {name:[] for name in joint_names}
    mdots_dict = {name:[] for name in joint_names}
    contractions_dict = {name:[] for name in joint_names}
    forces_dict = {name:[] for name in joint_names}
    pressures_dict["source"] = []
    masses_dict["source"] = []
    time_list = []

    for k in range(num_steps):
        for name in joint_names:
            j = jr.robot.joint(name).id()
            qs_dict[name].append(gtd.JointAngleDouble(values, j, k))
            vs_dict[name].append(gtd.JointVelDouble(values, j, k))
            torques_dict[name].append(gtd.TorqueDouble(values, j, k))
            pressures_dict[name].append(values.atDouble(Actuator.PressureKey(j, k)))
            masses_dict[name].append(values.atDouble(Actuator.MassKey(j, k)))
            mdots_dict[name].append(values.atDouble(Actuator.MassRateActualKey(j, k)))
            contractions_dict[name].append(values.atDouble(Actuator.ContractionKey(j, k)))
            forces_dict[name].append(values.atDouble(Actuator.ForceKey(j, k)))
        masses_dict["source"].append(values.atDouble(Actuator.SourceMassKey(k)))
        pressures_dict["source"].append(values.atDouble(Actuator.SourcePressureKey(k)))
        time_list.append(values.atDouble(gtd.TimeKey(k).key()))

    fig, axs = plt.subplots(2, 3, sharex=True, figsize=(10, 6.7), dpi=80)

    for name in qs_dict.keys():
        axs[0, 0].plot(time_list, qs_dict[name], label=name, color=colors[name])
    axs[0, 0].set_title("joint angle")

    for name in torques_dict.keys():
        axs[0, 1].plot(time_list, torques_dict[name], label=name, color=colors[name])
    axs[0, 1].set_title("torque")

    for name in forces_dict.keys():
        axs[0, 2].plot(time_list, forces_dict[name], label=name, color=colors[name])
    axs[0, 2].set_title("force")

    for name in pressures_dict.keys():
        axs[1, 0].plot(time_list, pressures_dict[name], label=name, color=colors[name])
    axs[1, 0].set_title("pressure")

    for name in masses_dict.keys():
        axs[1, 1].plot(time_list, masses_dict[name], label=name, color=colors[name])
    axs[1, 1].set_title("mass")

    for name in contractions_dict.keys():
        axs[1, 2].plot(time_list, contractions_dict[name], label=name, color=colors[name])
    axs[1, 2].set_title("contraction")
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

