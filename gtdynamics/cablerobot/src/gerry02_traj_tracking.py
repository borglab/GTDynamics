"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  gerry02_traj_tracking.py
@brief Computes the linearized feedforward controllers for tracking a trajectory.
@author Frank Dellaert
@author Gerry Chen

Given a .h file containing the trajectory, we compute the time-varying linearized iLQR gains &
feedforward terms and write them out to a new .h file.
"""

import gtdynamics as gtd
import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam import Pose3, Rot3

from cdpr_planar import Cdpr, CdprParams
from cdpr_controller_ilqr import CdprControllerIlqr
from cdpr_planar_sim import CdprSimulator
from paint_parse import ParseFile, writeControls
from draw_cdpr import plot_trajectory
from draw_controller import draw_controller_anim

import cProfile
from pstats import SortKey

FRAME_WIDTH, FRAME_HEIGHT = 2.92, 2.32
EE_WIDTH, EE_HEIGHT = 0.15, 0.30

def create_cdpr():
    """Creates a cdpr with parameters matching the real CDPR in lab.
    """
    # cdpr object
    aw, ah = FRAME_WIDTH, FRAME_HEIGHT
    bw, bh = EE_WIDTH, EE_HEIGHT
    params = CdprParams()
    params.mass = 0.5
    params.gravity = np.array([0, 0, -9.8]).reshape((3, 1))
    params.a_locs = np.array([[aw, 0, 0], [aw, 0, ah], [0, 0, ah], [0, 0, 0]])
    params.b_locs = np.array([[bw, 0., -bh], [bw, 0., bh], [-bw, 0., bh], [-bw, 0, -bh]]) / 2
    params.b_locs = params.b_locs - [0, 0, bh * 0.4]
    params.winch_params.inertia_ = 9.26e-5 * 890 / 420 # https://bit.ly/3sOF2Wt
    params.winch_params.radius_ = 0.0127
    return Cdpr(params)

def print_data(isPaints, colorinds, colorpalette, traj, N=100):
    """Prints out the paint trajectory for sanity check"""
    def paintString(isPaint, colori):
        return '{:d}, {:d}, {:d}'.format(*colorpalette[colori]) if isPaint else 'paint off'
    i = 0
    print(traj.shape)
    for isPaint, colori, point in zip(isPaints, colorinds, traj):
        print('{:5.2f}, {:5.2f}\t-\t{}'.format(*point, paintString(isPaint, colori)))
        if i > N:
            return
        i += 1

def xy2Pose3(traj):
    """Converts an Nx2 numpy array into a list of gtsam.Pose3 objects"""
    des_T = []
    for xy in traj:
        des_T.append(gtsam.Pose3(gtsam.Rot3(), (xy[0], 0, xy[1])))
    return des_T

def main(fname='data/ATL.h',
         debug=False,
         Q=np.ones(6) * 1e2,
         R=np.ones(1) * 1e-2,
         N0=0,
         N=None,
         dN=1,
         speed_multiplier=1):
    """Runs a simulation of the iLQR controller trying to execute a predefined trajectory.

    Args:
        fname (str, optional): The trajectory filename. Defaults to 'data/ATL.h'.
        debug (bool, optional): Whether to print debug information. Defaults to False.
        Q (np.ndarray, optional): Vector of weights to apply to the state objectives.  The real
        weight matrix will be diag(Q). Defaults to np.ones(6)*1e2.
        R (np.ndarray, optional): Vector of weights to apply to the control costs.  The real weight
        matrix will be diag(R). Defaults to np.ones(1)*1e-2.
        N0 (int, optional): The initial timestep of the trajectory to start at, since running the
        full trajectory is very time consuming. Defaults to 0.
        N (int, optional): The number of timesteps of the trajectory to run, since running the full
        trajectory is very time consuming. Defaults to 500.
        dN (int, optional): Skips some timesteps from the input trajectory, to make things faster.
        The controller and simulation both will only use each dN'th time step.  Defaults to 1.

    Returns:
        tuple(Cdpr, CdprControllerIlqr, gtsam.Values, int, float, list[gtsam.Pose3]): The relevant
        output data including:
            - cdpr: the cable robot object
            - controller: the controller
            - result: the Values object containing the full state and controls of the robot in
            open-loop
            - N: the number of time steps (equal to N passed in)
            - dt: the time step size
            - des_T: the desired poses
    """
    # cdpr object
    cdpr = create_cdpr()

    # import data
    isPaints, colorinds, colorpalette, traj = ParseFile(fname)
    N = len(traj) - N0 - 1 if N is None else N
    dt = (0.01 / speed_multiplier
          ) * dN  # this is a hardcoded constant.  TODO(gerry): include this in the .h file.
    N = int(N/dN)  # scale time by dN
    N0 = int(N0/dN)
    width, _, height = cdpr.params.a_locs[1] - cdpr.params.a_locs[3]  # rescale trajectory to be
    traj = (traj - [width/2, height/2]) * 0.85 + [width/2, height/2]  # smaller
    traj = traj[::dN, :]
    if debug:
        print_data(isPaints, colorinds, colorpalette, traj, N=100)
    # only simulate a subset of the trajectory, since the trajectory is very large
    des_T = xy2Pose3(traj[N0:N0+N, :])


    # initial configuration
    x0 = gtsam.Values()
    # gtd.InsertPose(x0, cdpr.ee_id(), 0, gtsam.Pose3(gtsam.Rot3(), (1.5, 0, 1.5)))
    gtd.InsertPose(x0, cdpr.ee_id(), 0, des_T[0])
    gtd.InsertTwist(x0, cdpr.ee_id(), 0, (0, 0, 0, 0, 0, 0))

    # controller
    controller = CdprControllerIlqr(cdpr, x0, des_T, dt, Q, R)
    # feedforward control
    xff = np.zeros((N, 2))
    uff = np.zeros((N, 4))
    for t in range(N):
        xff[t, :] = gtd.Pose(controller.result, cdpr.ee_id(), t).translation()[[0, 2]]
        uff[t, :] = [gtd.TorqueDouble(controller.result, ji, t) for ji in range(4)]
    if debug:
        print(xff)
        print(uff)

    # simulate
    sim = CdprSimulator(cdpr, x0, controller, dt=dt)
    result = sim.run(N=N)
    if debug:
        print(result)

    return cdpr, controller, result, N, dt, des_T

def plot(cdpr, controller, result, N, dt, des_T):
    """Plots the results"""
    plot_trajectory(cdpr, result, dt*N, dt, N, des_T, step=1)

def save_controller(fname, controller):
    writeControls(fname, controller.gains_ff)

if __name__ == '__main__':
    # cProfile.run('results = main(N=100)', sort=SortKey.TIME)
    results = main(fname='data/ATL_filled.h', Q=np.ones(6)*1e1, R=np.ones(1)*1e-3, dN=1, debug=False, speed_multiplier=1)
    plot(*results)
    plt.show()
    save_controller('data/tmp.h', results[1])
