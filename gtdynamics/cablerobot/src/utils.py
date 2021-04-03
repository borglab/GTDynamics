import gtsam
import gtdynamics as gtd
import numpy as np

def zerovalues(lid, ts=[], dt=0.01):
    zero = gtsam.Values()
    zero.insertDouble(0, dt)
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
