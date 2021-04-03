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
from cdpr_planar_sim import cdpr_sim

def main():
    Tf = 1
    dt = 0.05
    N = int(Tf / dt)
    cdpr = Cdpr()
    # set up controller
    pdes = [
        gtsam.Pose3(gtsam.Rot3(),
                    (1.5 + np.cos(2 * np.pi * i / N), 0, 1.5 + np.sin(2 * np.pi * i / N)))
        for i in range(N)
    ]
    pdes[0] = pdes[1]
    x0 = gtsam.Values()
    gtd.InsertPose(x0, cdpr.ee_id(), 0, pdes[0])
    gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))
    controller = CdprController(cdpr,
                                x0,
                                pdes,
                                dt=dt,
                                Q=np.array([0, 1, 0, 1e3, 0, 1e3]),
                                R=np.array([1e-3]))
    # run simulation
    result = cdpr_sim(cdpr, x0, controller, dt=dt, N=N, verbose=True)
    poses = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(N)]

    # print(poses)

    plt.figure(1)
    plt.plot([pose.x() for pose in pdes], [pose.z() for pose in pdes], 'r-')
    plt.plot([pose.x() for pose in poses], [pose.z() for pose in poses], 'k--')
    plt.plot([*cdpr.params.frameLocs[:, 0], cdpr.params.frameLocs[0, 0]],
             [*cdpr.params.frameLocs[:, 2], cdpr.params.frameLocs[0, 2]], 'k-')
    plt.axis('equal')
    plt.show()


if __name__ == '__main__':
    main()
