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

from cdpr_planar import Cdpr
from cdpr_planar_controller import CdprController
from cdpr_planar_sim import CdprSimulator

import cProfile

def main():
    Tf = 1
    dt = 0.01
    N = int(Tf / dt)
    cdpr = Cdpr()
    # set up controller
    x_des = [
        gtsam.Pose3(gtsam.Rot3(),
                    (1.5 + np.cos(2 * np.pi * i / N), 0, 1.5 + np.sin(2 * np.pi * i / N)))
        for i in range(N)
    ]
    x_des[0] = x_des[1]
    # initial state
    x0 = gtsam.Values()
    gtd.InsertPose(x0, cdpr.ee_id(), 0, x_des[0])
    gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))
    # controller
    controller = CdprController(cdpr,
                                x0,
                                x_des,
                                dt=dt,
                                Q=np.array([0, 1, 0, 1e3, 0, 1e3]),
                                R=np.array([1e-3]))
    # run simulation
    sim = CdprSimulator(cdpr, x0, controller, dt=dt)
    result = sim.run(N=N, verbose=False)

    # extract useful variables as lists
    poses = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N+1)]
    posesxy = np.array([[pose.x() for pose in poses], [pose.z() for pose in poses]])
    desposesxy = np.array([[pose.x() for pose in x_des], [pose.z() for pose in x_des]])
    torques = np.array([[gtd.TorqueDouble(result, ji, k) for ji in range(4)] for k in range(N)])

    # plot utils
    boxinds = np.array([0,1,2,3,0]).reshape(5,1)
    def ee_coords(k):
        return (posesxy[:, k]+cdpr.params.b_locs[boxinds, [0,2]]).T
    frame_coords = cdpr.params.a_locs[boxinds, [0,2]].T
    def cable_coords(k, ji):
        return np.array([[cdpr.params.a_locs[ji][0], posesxy[0][k]+cdpr.params.b_locs[ji][0]],
                        [cdpr.params.a_locs[ji][2], posesxy[1][k]+cdpr.params.b_locs[ji][2]]])
    # plot
    fig = plt.figure(1, figsize=(12,4))
    # xy plot
    plt.subplot(1,2,1)
    plt.plot(*frame_coords, 'k-')
    plt.plot(*desposesxy, 'r*') # desired trajectory
    ltraj, = plt.plot(*posesxy, 'k-') # actual trajectory
    lscables = plt.plot(np.zeros((2,4)), np.zeros((2,4)))
    lee, = plt.plot(*ee_coords(0))
    plt.axis('equal')
    plt.xlabel('x(m)');plt.ylabel('y(m)');plt.title('Trajectory')
    # controls
    plt.subplot(1,2,2)
    lsctrl = plt.plot(np.arange(0,Tf,dt), torques)
    plt.xlabel('time (s)');plt.ylabel('Cable tension (N)');plt.title('Control Inputs');


if __name__ == '__main__':
    cProfile.run('main()')
