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
            gtd.InsertJointAccelDouble(zero, j, t, 0)
            gtd.InsertTensionDouble(zero, j, t, 0)
            gtd.InsertTorqueDouble(zero, j, t, 0)
            gtd.InsertWrench(zero, lid, j, t, np.zeros(6))
        gtd.InsertPose(zero, lid, t, gtsam.Pose3(gtsam.Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(zero, lid, t, np.zeros(6))
        gtd.InsertTwistAccel(zero, lid, t, np.zeros(6))
    return zero

def InsertPose(dest, link_id, k, source):
    gtd.InsertPose(dest, link_id, k, gtd.Pose(source, link_id, k))
def InsertTwist(dest, link_id, k, source):
    gtd.InsertTwist(dest, link_id, k, gtd.Twist(source, link_id, k))
def InsertTwistAccel(dest, link_id, k, source):
    gtd.InsertTwistAccel(dest, link_id, k, gtd.TwistAccel(source, link_id, k))
def InsertJointAngles(dest, k, source):
    for ji in range(4):
        gtd.InsertJointAngleDouble(dest, ji, k, gtd.JointAngleDouble(source, ji, k))
def InsertJointVels(dest, k, source):
    for ji in range(4):
        gtd.InsertJointVelDouble(dest, ji, k, gtd.JointVelDouble(source, ji, k))
def InsertTorques(dest, k, source):
    for ji in range(4):
        gtd.InsertTorqueDouble(dest, ji, k, gtd.TorqueDouble(source, ji, k))
def InsertWrenches(dest, link_id, k, source):
    for ji in range(4):
        gtd.InsertWrench(dest, link_id, ji, k, gtd.Wrench(source, link_id, ji, k))

def MyLMParams(abs_err_tol=1e-15):
    params = gtsam.LevenbergMarquardtParams()
    params.setRelativeErrorTol(0)
    params.setAbsoluteErrorTol(0)
    params.setErrorTol(abs_err_tol)
    params.setLinearSolverType("MULTIFRONTAL_QR")
    return params
