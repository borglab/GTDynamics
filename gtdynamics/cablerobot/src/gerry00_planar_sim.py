"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  gerry00_planar_sim.py
@brief Quick visualization of a planar cdpr simulation.
@author Frank Dellaert
@author Gerry Chen
"""

import gtdynamics as gtd
import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam import Pose3, Rot3

from cdpr_planar import Cdpr
from cdpr_planar_controller import CdprControllerBase
from cdpr_planar_sim import cdpr_sim


class DummyController(CdprControllerBase):
    def __init__(self, N):
        self.N = N
    def update(self, values, t):
        tau = gtsam.Values()
        gtd.InsertTorque(tau, 0, t, 3 * np.cos(2. * t / self.N * np.pi))
        gtd.InsertTorque(tau, 1, t, 3 * np.cos(2. * t / self.N * np.pi + np.pi/2))
        gtd.InsertTorque(tau, 2, t, 3 * np.cos(2. * t / self.N * np.pi + np.pi))
        gtd.InsertTorque(tau, 3, t, 3 * np.cos(2. * t / self.N * np.pi + 3*np.pi/2))
        return tau

def main():
    Tf = 1
    dt = 0.05
    N = int(Tf / dt)
    cdpr = Cdpr()
    controller = DummyController(N)
    # initial state
    x0 = gtsam.Values()
    gtd.InsertPose(x0, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
    gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))
    # run simulation
    result = cdpr_sim(cdpr, x0, controller, dt=dt, N=N)
    poses = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N)]

    plt.figure(1)
    plt.plot([pose.x() for pose in poses], [pose.z() for pose in poses], 'k--')
    plt.plot([*cdpr.params.a_locs[:, 0], cdpr.params.a_locs[0, 0]],
             [*cdpr.params.a_locs[:, 2], cdpr.params.a_locs[0, 2]], 'k-')
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    main()
