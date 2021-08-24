"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  utils.py
@brief Useful python functions specific to cable robots
@author Frank Dellaert
@author Gerry Chen
"""

import gtsam
import gtdynamics as gtd
import numpy as np

def zerovalues(lid, ts=[], dt=0.01):
    """Creates a values object for initialization, populated with zeros.

    Args:
        lid (int): The id of the (end-effector) link
        ts (list, optional): Time step indices. Defaults to [].
        dt (float, optional): Time step duration. Defaults to 0.01.

    Returns:
        gtsam.Values: initialized values with zeros
    """
    zero = gtsam.Values()
    zero.insert(0, dt)
    for t in ts:
        for j in range(4):
            gtd.InsertJointAngleDouble(zero, j, t, 0)
            gtd.InsertJointVelDouble(zero, j, t, 0)
            gtd.InsertTorqueDouble(zero, j, t, 0)
            gtd.InsertWrench(zero, lid, j, t, np.zeros(6))
        gtd.InsertPose(zero, lid, t, gtsam.Pose3(gtsam.Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(zero, lid, t, np.zeros(6))
        gtd.InsertTwistAccel(zero, lid, t, np.zeros(6))
    return zero
