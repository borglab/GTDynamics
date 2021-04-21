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
    """Draws the CDPR in the specified axis.
    Args:
        ax (plt.Axes): matplotlib axis object
        cdpr (Cdpr): cable robot object
        x (gtsam.Pose3): current pose of the end effector
    
    Returns:
        tuple(): matplotlib line objects:
            l_a - the frame
            l_b - the end effector
            ls_ab - a 4-list containing the 4 cable lines
    """
    l_a, = ax.plot(*a_coords(cdpr), 'k-')
    l_b, = ax.plot(*b_coords(cdpr, x), color='#caa472')
    ls_ab = [ax.plot(*ab_coords(cdpr, x, ji))[0] for ji in range(4)]
    ax.axis('equal')
    ax.set_xlabel('x(m)');ax.set_ylabel('y(m)');ax.set_title('Trajectory');ax.grid()
    return l_a, l_b, ls_ab
def redraw_cdpr(l_a, l_b, ls_ab, cdpr, x):
    """Updates the lines for the frame, end effector, and cables"""
    l_a.set_data(*a_coords(cdpr))
    l_b.set_data(*b_coords(cdpr, x))
    [ls_ab[ci].set_data(*ab_coords(cdpr, x, ci)) for ci in range(4)]
    return l_a, l_b, *ls_ab
def draw_traj(ax, cdpr, des_xy, act_xy):
    """Draws the desired and actual x/y trajectories"""
    l_des, = plt.plot(*des_xy, 'r-')  # desired trajectory
    l_act, = plt.plot(*act_xy, 'k-')  # actual trajectory
    ax.axis('equal')
    ax.set_xlabel('x(m)');ax.set_ylabel('y(m)');ax.set_title('Trajectory')
    return l_des, l_act
def redraw_traj(l_des, l_act, des_xy, act_xy, N=None):
    """Updates the lines for the trajectory drawing"""
    if N is None:
        N = des_xy.shape[1]
    l_des.set_data(*des_xy[:, :N])
    l_act.set_data(*act_xy[:, :N])
    return l_des, l_act
def draw_ctrl(ax, cdpr, tensions, Tf, dt):
    """Draws the control tension signals"""
    ls_ctrl = ax.plot(np.arange(0,Tf,dt), tensions)
    ax.plot([0, Tf], [cdpr.params.tmin,]*2, 'r--')
    ax.plot([0, Tf], [cdpr.params.tmax,]*2, 'r--')
    ax.set_xlabel('time (s)');ax.set_ylabel('Cable tension (N)');ax.set_title('Control Inputs')
    ax.grid()
    return ls_ctrl,
def redraw_ctrl(ls_ctrl, tensions, Tf, dt, N=None):
    """Updates the lines for the tensions plot"""
    if N is None:
        N = tensions.shape[0]
    else:
        ls_ctrl[0].axes.set_xlim(max(0, dt*N-10), max(10, dt*N))
    for ji in range(4):
        ls_ctrl[ji].set_data(np.arange(0,Tf,dt)[:N], tensions[:N, ji])
    return *ls_ctrl,

def plot_all(cdpr, result, Tf, dt, N, x_des, step=1):
    """Animates the cdpr and controls in side-by-side subplots.

    Args:
        cdpr (Cdpr): cable robot object
        result (gtsam.Values): the data from the simulation including poses and tensions
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
    tensions = np.array([[gtd.TorqueDouble(result, ji, k) for ji in range(4)] for k in range(N)])

    # plot
    fig = plt.figure(1, figsize=(12,4))
    # xy plot
    ax1 = plt.subplot(1,2,1)
    cdpr_lines = draw_cdpr(ax1, cdpr, gtd.Pose(result, cdpr.ee_id(), 0))
    traj_lines = draw_traj(ax1, cdpr, des_xy, act_xy)
    # controls
    ax2 = plt.subplot(1,2,2)
    ctrl_lines = draw_ctrl(ax2, cdpr, tensions, Tf, dt)

    # animate
    plt.rcParams["savefig.dpi"] = 80
    def update_line(num):
        cdpr_to_update = redraw_cdpr(*cdpr_lines, cdpr, gtd.Pose(result, cdpr.ee_id(), num))
        traj_to_update = redraw_traj(*traj_lines, des_xy, act_xy, num if num > 0 else None)
        ctrl_to_update = redraw_ctrl(*ctrl_lines, tensions, Tf, dt, num if num > 0 else None)
        return [*cdpr_to_update, *traj_to_update, *ctrl_to_update]
    return animation.FuncAnimation(fig, update_line, frames=range(0, N, step),
                                   interval=dt*step*1e3, blit=True)
