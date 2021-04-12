"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  draw_cdpr.py
@brief Utility functions for plotting a cable robot trajectory
@author Frank Dellaert
@author Gerry Chen
"""

import gtdynamics as gtd
import gtsam
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from gtsam import Pose3, Rot3

from cdpr_planar import Cdpr

BOXINDS = np.array([0,1,2,3,0]).reshape(5,1)

def pose32xy(x):
    """Extracts just the planar translational component of a Pose3"""
    return x.translation()[[0, 2]]
def a_coords(cdpr):
    """Returns a 2x5 array of the xy coordinates of the frame corners"""
    return cdpr.params.a_locs[BOXINDS, [0,2]].T
def b_coords(cdpr, x):
    """Returns a 2x5 array of the xy coordinates of the end-effector mounting points"""
    return (pose32xy(x) + cdpr.params.b_locs[BOXINDS, [0, 2]]).T
def ab_coords(cdpr, x, ji):
    """Returns a 2x2 arrays of the xy coordinates of the specified cable's start/end locations."""
    b = b_coords(cdpr, x)
    return np.array([[cdpr.params.a_locs[ji][0], b[0, ji]],
                     [cdpr.params.a_locs[ji][2], b[1, ji]]])
def draw_cdpr(ax, cdpr, x):
    """Draws the CDPR in the specified axis and returns the line objects:
    l_a - the frame
    l_b - the end effector
    l_abs - a 4-list containing the 4 cable lines
    """
    l_a, = ax.plot(*a_coords(cdpr))
    l_b, = ax.plot(*b_coords(cdpr, x))
    l_abs = [ax.plot(*ab_coords(cdpr, x, ji))[0] for ji in range(4)]
    ax.axis('equal')
    ax.set_xlabel('x(m)');ax.set_ylabel('y(m)');ax.set_title('Trajectory')
    return l_a, l_b, l_abs
def redraw_cdpr(l_a, l_b, l_abs, cdpr, x):
    """Updates the lines for the frame, end effector, and cables"""
    l_a.set_data(*a_coords(cdpr))
    l_b.set_data(*b_coords(cdpr, x))
    [l_abs[ci].set_data(*ab_coords(cdpr, x, ci)) for ci in range(4)]
    return l_a, l_b, *l_abs

def plot_all(cdpr, result, Tf, dt, N, x_des, step=1):
    """Animates the cdpr and controls in side-by-side subplots.

    Args:
        cdpr (Cdpr): cable robot object
        result (gtsam.Values): the data from the simulation including poses and torques
        Tf (float): final time
        dt (float): time step interval
        N (int): number of discrete samples
        x_des (List[gtsam.Pose3]): the desired trajectory as a list of gtsam Pose3 objects
        step (int, optional): number of time steps to update the animation by each time. 1 doesn't skip any frames, and e.g. 10 would skip 9 frames at a time and update at a period of 10*dt. Defaults to 1.

    Returns:
        matplotlib.animation.FuncAnimation: a matplotlib animation object
    """    
    # extract useful variables as lists
    act_T = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N+1)]
    act_xy = np.array([pose32xy(pose) for pose in act_T]).T
    des_xy = np.array([pose32xy(pose) for pose in x_des]).T
    torques = np.array([[gtd.TorqueDouble(result, ji, k) for ji in range(4)] for k in range(N)])

    # plot
    fig = plt.figure(1, figsize=(12,4))
    # xy plot
    ax1 = plt.subplot(1,2,1)
    cdpr_lines = draw_cdpr(ax1, cdpr, gtd.Pose(result, cdpr.ee_id(), 0))
    plt.plot(*des_xy, 'r-') # desired trajectory
    ltraj, = plt.plot(*act_xy, 'k-') # actual trajectory
    # controls
    ax2 = plt.subplot(1,2,2)
    lsctrl = plt.plot(np.arange(0,Tf,dt), torques)
    plt.xlabel('time (s)');plt.ylabel('Cable tension (N)');plt.title('Control Inputs')

    # animate
    plt.rcParams["savefig.dpi"] = 80

    def update_line(num):
        lines_to_update = redraw_cdpr(*cdpr_lines, cdpr, gtd.Pose(result, cdpr.ee_id(), num))
        for ji in range(4):
            lsctrl[ji].set_data(np.arange(0,Tf,dt)[:num], torques[:num, ji])
        return [*lines_to_update, *lsctrl]

    return animation.FuncAnimation(fig, update_line, frames=range(0, N, step),
                                   interval=dt*step*1e3, blit=True)
