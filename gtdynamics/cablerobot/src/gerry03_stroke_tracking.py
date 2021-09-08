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
import tqdm

DT = 0.01  # hardcoded constant.  TODO(gerry): include in .h file.


def preprocessTraj(cdpr: Cdpr, is_paint_on: list[bool], traj: np.ndarray, dN: int):
    """Preprocesses the trajectory to (1) rescale the trajectory, and (2) take every dN'th index.
    `traj` is an Nx2 array.
    """
    # rescale trajectory to be smaller
    width, _, height = cdpr.params.a_locs[1] - cdpr.params.a_locs[3]  # rescale trajectory to be
    traj = (traj - [width / 2, height / 2]) * 0.85 + [width / 2, height / 2]  # smaller
    # extract the part of the trajectory we care about
    return np.array(is_paint_on)[::dN], traj[::dN, :]


def splitTrajByStroke(is_paint_on: list[bool], all_des_poses: list[gtsam.Pose3]):
    """Splits up a trajectory according to paint changes
    """
    prev_index = 0
    for transition_index in np.argwhere(np.diff(is_paint_on)) + 1:
        yield all_des_poses[prev_index:transition_index[0]]
        prev_index = transition_index[0]
    yield all_des_poses[prev_index:]


def combineResults(all_results: list[gtsam.Values]) -> gtsam.Values:
    """Combines values objects, taking care to remove duplicates
    """
    combined_results = gtsam.Values()
    combined_keys = set()
    for results in all_results:
        # remove duplicates
        new_keys = set(results.keys())
        for key in (combined_keys & new_keys):
            results.erase(key)
        combined_keys |= new_keys
        # insert
        combined_results.insert(results)
    return combined_results


def optimizeStroke(cdpr: Cdpr,
                   stroke: list[gtsam.Pose3],
                   x0: gtsam.Pose3,
                   v0,
                   Q,
                   R,
                   dt: float,
                   x_guess: gtsam.Values = None):
    """Computes the optimal controller for a single stroke
    """
    N = len(stroke)

    # initial configuration
    X_init = gtsam.Values()
    gtd.InsertPose(X_init, cdpr.ee_id(), 0, x0)
    gtd.InsertTwist(X_init, cdpr.ee_id(), 0, v0)

    # controller
    controller = CdprControllerIlqr(cdpr, X_init, stroke, dt, Q, R, x_guess=x_guess)
    # feedforward control
    xff = np.zeros((N, 2))
    uff = np.zeros((N, 4))
    for t in range(N):
        xff[t, :] = gtd.Pose(controller.result, cdpr.ee_id(), t).translation()[[0, 2]]
        uff[t, :] = [gtd.TorqueDouble(controller.result, ji, t) for ji in range(4)]

    # simulate
    sim = CdprSimulator(cdpr, X_init, controller, dt=dt)
    result = sim.run(N=N)
    xf = gtd.Pose(result, cdpr.ee_id(), N)
    vf = gtd.Twist(result, cdpr.ee_id(), N)

    return xf, vf, controller, result


def optimizeStrokes(cdpr: Cdpr, strokes: list[list[gtsam.Pose3]], Q, R, dt: float):
    """Finds the optimal controllers for a set of strokes
    """
    all_controllers = []
    all_results = []
    cur_x, cur_v = strokes[0][0], (0, 0, 0, 0, 0, 0)
    k = 0

    # TODO(Gerry): figure out multithreading.  Tried concurrent.futures.ThreadPoolExecutor but it
    # didn't help at all due to GIL.  Couldn't get ProcessPoolExecutor to work - it wouldn't
    # actually execute the thread.
    for stroke in tqdm.tqdm(strokes):  # tqdm is iteration timer
        cur_x, cur_v, controller, result = optimizeStroke(cdpr, stroke, cur_x, cur_v, Q, R, dt)

        # shift timesteps
        new = gtsam.Values()
        for key in result.keys():
            utils.UpdateFromValues(result, new, gtd.DynamicsSymbol(key), shift_time_by=k)
        result = new

        # save and update
        all_controllers.append(controller)
        all_results.append(result)
        k += len(stroke)

    return cdpr, all_controllers, all_results, strokes, dt


def main(fname: str = 'data/ATL_filled.h',
         Q=np.ones(6) * 1e2,
         R=np.ones(1) * 1e-2,
         dN: int = 1,
         speed_multiplier: float = 1):
    """Runs a simulation of the iLQR controller trying to execute a predefined trajectory.

    Args:
        fname (str, optional): The trajectory filename. Defaults to 'data/iros_logo_2.h'.
        Q (np.ndarray, optional): Vector of weights to apply to the state objectives.  The real
        weight matrix will be diag(Q). Defaults to np.ones(6)*1e2.
        R (np.ndarray, optional): Vector of weights to apply to the control costs.  The real weight
        matrix will be diag(R). Defaults to np.ones(1)*1e-2.
        dN (int, optional): Skips some timesteps from the input trajectory, to make things faster.
        The controller and simulation both will only use each dN'th time step.  Defaults to 1.
        speed_multiplier (float, optional): Makes the entire trajectory faster or slower

    Returns:
        tuple(Cdpr, CdprControllerIlqr, gtsam.Values, list[gtsam.Pose3], float): The relevant
        output data including:
            - cdpr: the cable robot object
            - controller: the controller
            - result: the Values object containing the full state and controls of the robot in
            open-loop
            - des_T: the desired poses
            - dt: the time step size
    """
    cdpr = gerry02_traj_tracking.create_cdpr()
    dt = (DT / speed_multiplier) * dN

    # import data
    is_paint_on, _, _, all_trajs = ParseFile(fname)
    is_paint_on, all_trajs = preprocessTraj(cdpr, is_paint_on, all_trajs, dN)
    all_des_poses = gerry02_traj_tracking.xy2Pose3(all_trajs)

    # run per-stroke
    print("Running per-stroke optimization...")
    strokes = list(splitTrajByStroke(is_paint_on, all_des_poses))
    cdpr, all_controllers, all_results, strokes, dt = optimizeStrokes(cdpr, strokes, Q, R, dt)

    # combine and run together to smooth out
    print("Combining per-stroke results and running one final optimization...")
    x_guess = combineResults(all_results)
    _, _, controller, result = optimizeStroke(cdpr,
                                              all_des_poses,
                                              all_des_poses[0], (0, 0, 0, 0, 0, 0),
                                              Q,
                                              R,
                                              dt,
                                              x_guess=x_guess)

    print("Done.")
    return cdpr, controller, result, all_des_poses, dt


def plot(cdpr, all_results, strokes, dt):
    """Plots the results"""
    N = sum(len(s) for s in strokes[:len(all_results)])
    plot_trajectory(cdpr, combineResults(all_results), dt * N, dt, N, sum(strokes, []), step=1)


def save_controller(fname, all_controllers):
    gains_ff = sum((controller.gains_ff for controller in all_controllers), [])
    writeControls(fname, gains_ff)


if __name__ == '__main__':
    # cProfile.run('cdpr, controller, result, des_poses, dt = main(dN=10)', sort=SortKey.TIME)
    cdpr, controller, result, des_poses, dt = main(dN=1)
    gerry02_traj_tracking.plot(cdpr, controller, result, len(des_poses), dt, des_poses)
    plt.show()
    gerry02_traj_tracking.save_controller('data/tmp.h', controller)
