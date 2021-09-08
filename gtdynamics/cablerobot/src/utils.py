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

def InsertZeroByLabel(values, key, dt=None):
    label, l, j, t = key.label(), key.linkIdx(), key.jointIdx(), key.time()
    if label == "q":  # JointAngleKey
        gtd.InsertJointAngleDouble(values, j, t, 0.0)
    elif label == "v":  # JointVelKey
        gtd.InsertJointVelDouble(values, j, t, 0.0)
    elif label == "a":  # JointAccelKey
        gtd.InsertJointAccelDouble(values, j, t, 0.0)
    elif label == "t":  # TensionKey
        gtd.InsertTensionDouble(values, j, t, 0.0)
    elif label == "T":  # TorqueKey
        gtd.InsertTorqueDouble(values, j, t, 0.0)
    elif label == "p":  # PoseKey
        gtd.InsertPose(values, l, t, gtsam.Pose3())
    elif label == "V":  # TwistKey
        gtd.InsertTwist(values, l, t, np.zeros(6))
    elif label == "A":  # TwistAccelKey
        gtd.InsertTwistAccel(values, l, t, np.zeros(6))
    elif label == "F":  # WrenchKey
        gtd.InsertWrench(values, l, j, t, np.zeros(6))
    elif (dt is not None) and (key.key() == 0):
        values.insert(key.key(), dt)
    else:
        print(key)
        raise RuntimeError("Unknown key label type: ", label)

def ZeroValues(graph, dt=None):
    z = gtsam.Values()
    for key in graph.keyVector():
        key = gtd.DynamicsSymbol(key)
        InsertZeroByLabel(z, key, dt)
    return z


def UpdateFromValues(src, dst, key, shift_time_by=0):
    label = key.label()
    if shift_time_by:
        new_key = gtd.DynamicsSymbol.LinkJointSymbol(key.label(), key.linkIdx(), key.jointIdx(),
                                                     key.time() + shift_time_by)
    else:
        new_key = key

    if label in "qvatT":  # joint angle, vel, acc, tension, torque
        dst.insert(new_key.key(), src.atDouble(key.key()))
    elif label == "p":  # PoseKey
        dst.insert(new_key.key(), src.atPose3(key.key()))
    elif label == "V":  # TwistKey
        gtd.InsertTwist(dst, new_key.linkIdx(), new_key.time(),
                        gtd.Twist(src, key.linkIdx(), key.time()))
    elif label == "A":  # TwistAccelKey
        gtd.InsertTwistAccel(dst, new_key.linkIdx(), new_key.time(),
                             gtd.TwistAccel(src, key.linkIdx(), key.time()))
    elif label == "F":  # WrenchKey
        gtd.InsertWrench(dst, new_key.linkIdx(), new_key.time(),
                         gtd.Wrench(src, key.linkIdx(), key.time()))
    elif key.key() == 0:
        dst.insert(new_key.key(), src.atDouble(key.key()))
    else:
        print(key)
        raise RuntimeError("Unknown key label type: ", label)


def InitValues(graph, values=None, dt=None):
    init = gtsam.Values()
    existing_keys = set(values.keys()) if values is not None else set()
    graphkeys = set(graph.keyVector())
    for key in graphkeys & existing_keys:
        UpdateFromValues(values, init, gtd.DynamicsSymbol(key))
    for key in graphkeys - existing_keys:
        InsertZeroByLabel(init, gtd.DynamicsSymbol(key), dt)
    return init

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
