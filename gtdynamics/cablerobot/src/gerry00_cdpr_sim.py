"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_cdpr_planar.py
@brief Unit tests for CDPR.
@author Frank Dellaert
@author Gerry Chen
"""

import gtdynamics as gtd
import gtsam
import matplotlib.pyplot as plt
import numpy as np
from gtsam import Pose3, Rot3

from cdpr_planar import Cdpr
from cdpr_planar_controller import CdprController, CdprControllerBase
from cdpr_planar_sim import cdpr_sim


class DummyController(CdprControllerBase):
    def __init__(self, N):
        self.N = N
    def update(self, values, t):
        tau = gtsam.Values()
        gtd.InsertTorqueDouble(tau, 0, t, 3 * np.cos(2. * t / self.N * np.pi))
        gtd.InsertTorqueDouble(tau, 1, t, 3 * np.cos(2. * t / self.N * np.pi + np.pi/2))
        gtd.InsertTorqueDouble(tau, 2, t, 3 * np.cos(2. * t / self.N * np.pi + np.pi))
        gtd.InsertTorqueDouble(tau, 3, t, 3 * np.cos(2. * t / self.N * np.pi + 3*np.pi/2))
        return tau

def main():
    Tf = 1
    dt = 0.05
    N = int(Tf / dt)
    cdpr = Cdpr()
    controller = DummyController(N)
    # initial state
    xInit = gtsam.Values()
    gtd.InsertPose(xInit, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
    gtd.InsertTwist(xInit, cdpr.ee_id(), 0, np.zeros(6))
    # run simulation
    result = cdpr_sim(cdpr, xInit, controller, dt=dt, N=N)
    poses = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N)]

    plt.figure(1)
    plt.plot([pose.x() for pose in poses], [pose.z() for pose in poses], 'k--')
    plt.plot([*cdpr.params.frameLocs[:, 0], cdpr.params.frameLocs[0, 0]],
             [*cdpr.params.frameLocs[:, 2], cdpr.params.frameLocs[0, 2]], 'k-')
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    main()
