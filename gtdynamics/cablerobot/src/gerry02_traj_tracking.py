"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  gerry01_planar_tracking.py
@brief quick test of open-loop trajectory tracking for planar cdpr.
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
from paint_parse import ParseFile
from draw_cdpr import plot_all

import cProfile
from pstats import SortKey

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

def main(fname='data/iros_logo_2.h', debug=False):
    # cdpr object
    aw, ah = 2.32, 1.92
    bw, bh = 0.15, 0.30
    params = CdprParams()
    params.a_locs = np.array([[aw, 0, 0], [aw, 0, ah], [0, 0, ah], [0, 0, 0]])
    params.b_locs = np.array([[bw, 0., -bh], [bw, 0., bh], [-bw, 0., bh], [-bw, 0, -bh]]) / 2
    params.b_locs = params.b_locs - [0, 0, bh * 0.4]
    cdpr = Cdpr(params)

    # import data
    isPaints, colorinds, colorpalette, traj = ParseFile(fname)
    dt = 0.01  # this is a hardcoded constant.  TODO(gerry): include this in the .h file.
    N = 500  # only simulate a subset of the trajectory, since the trajectory is very large
    if debug:
        print_data(isPaints, colorinds, colorpalette, traj, N=100)
    des_T = xy2Pose3(traj[:N, :])


    # initial configuration
    x0 = gtsam.Values()
    # gtd.InsertPose(x0, cdpr.ee_id(), 0, gtsam.Pose3(gtsam.Rot3(), (1.5, 0, 1.5)))
    gtd.InsertPose(x0, cdpr.ee_id(), 0, des_T[0])
    gtd.InsertTwist(x0, cdpr.ee_id(), 0, (0, 0, 0, 0, 0, 0))

    # controller
    controller = CdprControllerIlqr(cdpr, x0, des_T, dt, np.ones(6)*1e2, np.ones(1)*1e-2)
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
    sim = CdprSimulator(cdpr, x0, controller, dt=0.01)
    result = sim.run(N=N)
    if debug:
        print(result)

    return cdpr, controller, result, N, dt, des_T

def plot(cdpr, controller, result, N, dt, des_T):
    """Plots the results"""
    plot_all(cdpr, result, dt*N, dt, N, des_T, step=1)

if __name__ == '__main__':
    cProfile.run('results = main()', sort=SortKey.TIME)
    plot(*results)
