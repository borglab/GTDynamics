"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  gerry03_stroke_tracking.py
@brief Computes the controllers stroke-by-stroke for tracking a trajectory.
@author Frank Dellaert
@author Gerry Chen
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
import utils
import gerry02_traj_tracking

import cProfile
from pstats import SortKey

DT = 0.01  # hardcoded constant.  TODO(gerry): include in .h file.

def preprocessTraj(cdpr, is_paint_on, traj, dN):
    """Preprocesses the trajectory to (1) rescale the trajectory, and (2) take every dN'th index.

    Args:
        cdpr (Cdpr): CDPR object
        traj ([type]): [description]
        dN ([type]): [description]
    """
    # rescale trajectory to be smaller
    width, _, height = cdpr.params.a_locs[1] - cdpr.params.a_locs[3]  # rescale trajectory to be
    traj = (traj - [width/2, height/2]) * 0.85 + [width/2, height/2]  # smaller
    # extract the part of the trajectory we care about
    return np.array(is_paint_on)[::dN], traj[::dN, :]

def splitTrajByStroke(is_paint_on, all_des_poses):
    prev_index = 0
    for transition_index in np.argwhere(np.diff(is_paint_on)) + 1:
        yield all_des_poses[prev_index:transition_index[0]]
        prev_index = transition_index[0]
    yield all_des_poses[prev_index:]

def main(fname='data/ATL_filled.h',
         debug=False,
         Q=np.ones(6) * 1e2,
         R=np.ones(1) * 1e-2,
         dN=1,
         speed_multiplier=1):
    """Runs a simulation of the iLQR controller trying to execute a predefined trajectory.

    Args:
        fname (str, optional): The trajectory filename. Defaults to 'data/iros_logo_2.h'.
        debug (bool, optional): Whether to print debug information. Defaults to False.
        Q (np.ndarray, optional): Vector of weights to apply to the state objectives.  The real
        weight matrix will be diag(Q). Defaults to np.ones(6)*1e2.
        R (np.ndarray, optional): Vector of weights to apply to the control costs.  The real weight
        matrix will be diag(R). Defaults to np.ones(1)*1e-2.
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
    cdpr = gerry02_traj_tracking.create_cdpr()
    dt = (DT / speed_multiplier) * dN

    # import data
    is_paint_on, _, _, all_trajs = ParseFile(fname)
    is_paint_on, all_trajs = preprocessTraj(cdpr, is_paint_on, all_trajs, dN)
    all_des_poses = gerry02_traj_tracking.xy2Pose3(all_trajs)

    all_controllers = []
    all_results = []
    cur_x, cur_v = all_des_poses[0], (0, 0, 0, 0, 0, 0)

    strokes = list(splitTrajByStroke(is_paint_on, all_des_poses))
    k = 0
    for i, des_poses in enumerate(strokes):
        print("Starting stroke {:d} of {:d}".format(i, len(strokes)))
        if i > 5:
            break
        N = len(des_poses)

        # initial configuration
        X_init = gtsam.Values()
        gtd.InsertPose(X_init, cdpr.ee_id(), 0, cur_x)
        gtd.InsertTwist(X_init, cdpr.ee_id(), 0, cur_v)

        # controller
        controller = CdprControllerIlqr(cdpr, X_init, des_poses, dt, Q, R)
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
        sim = CdprSimulator(cdpr, X_init, controller, dt=dt)
        result = sim.run(N=N)
        if debug:
            print(result)
        cur_x = gtd.Pose(result, cdpr.ee_id(), N)
        cur_v = gtd.Twist(result, cdpr.ee_id(), N)

        # shift timesteps
        new = gtsam.Values()
        for key in result.keys():
            utils.UpdateFromValues(result, new, gtd.DynamicsSymbol(key), shift_time_by=k)
        result = new

        # store
        all_controllers.append(all_controllers)
        all_results.append(result)
        k += N

    return cdpr, all_controllers, all_results, strokes, dt

def plot(cdpr, all_results, strokes, dt):
    """Plots the results"""
    N = sum(len(s) for s in strokes[:len(all_results)])
    combined_results = gtsam.Values()
    time_shifts = np.cumsum([len(des_T) for des_T in strokes])
    time_shifts = [0, *time_shifts]
    for i, (results, time_shift) in enumerate(zip(all_results, time_shifts)):
        # remove duplicates
        overlap = set(combined_results.keys()) & set(results.keys())
        for key in overlap:
            results.erase(key)
        # insert
        combined_results.insert(results)
    plot_trajectory(cdpr, combined_results, dt*N, dt, N, sum(strokes, []), step=1)

def save_controller(fname, controller):
    writeControls(fname, controller.gains_ff)

if __name__ == '__main__':
    cProfile.run('cdpr, all_controllers, all_results, strokes, dt = main()',
                 sort=SortKey.TIME)
    plot(cdpr, all_results, strokes, dt)
