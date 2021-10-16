import gtsam

from gtdynamics.gtdynamics import *

from . import sim


class _GtdKeyFormatter(object):
    """Private class to format Values and NonlinearFactorGraph keys correctly."""
    def __repr__(self):
        return GtdFormat(self)


class Values(_GtdKeyFormatter, gtsam.Values):
    pass


class NonlinearFactorGraph(_GtdKeyFormatter, gtsam.NonlinearFactorGraph):
    pass


# Deprecate templated JointXXXDouble and InsertJointXXXDouble functions
import warnings


def InsertJointAngleDouble(*args, **kwargs):
    warnings.warn("InsertJointAngleDouble is deprecated.  Please use InsertJointAngle instead",
                  category=DeprecationWarning)
    InsertJointAngle(*args, **kwargs)


def JointAngleDouble(*args, **kwargs):
    warnings.warn("JointAngleDouble is deprecated.  Please use JointAngle instead",
                  category=DeprecationWarning)
    JointAngle(*args, **kwargs)


def InsertJointVelDouble(*args, **kwargs):
    warnings.warn("InsertJointVelDouble is deprecated.  Please use InsertJointVel instead",
                  category=DeprecationWarning)
    InsertJointVel(*args, **kwargs)


def JointVelDouble(*args, **kwargs):
    warnings.warn("JointVelDouble is deprecated.  Please use JointVel instead",
                  category=DeprecationWarning)
    JointVel(*args, **kwargs)


def InsertJointAccelDouble(*args, **kwargs):
    warnings.warn("InsertJointAccelDouble is deprecated.  Please use InsertJointAccel instead",
                  category=DeprecationWarning)
    InsertJointAccel(*args, **kwargs)


def JointAccelDouble(*args, **kwargs):
    warnings.warn("JointAccelDouble is deprecated.  Please use JointAccel instead",
                  category=DeprecationWarning)
    JointAccel(*args, **kwargs)


def InsertTorqueDouble(*args, **kwargs):
    warnings.warn("InsertTorqueDouble is deprecated.  Please use InsertTorque instead",
                  category=DeprecationWarning)
    InsertTorque(*args, **kwargs)


def TorqueDouble(*args, **kwargs):
    warnings.warn("TorqueDouble is deprecated.  Please use Torque instead",
                  category=DeprecationWarning)
    Torque(*args, **kwargs)
