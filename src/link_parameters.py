#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math

import numpy as np

import utils
from dh_link import DH_Link, Z33
from gtsam import Point3, Pose3, Rot3
from urdf_link import URDF_Link

# Denavit-Hartenberg parameters for RR manipulator
RR_calibration_dh = 2 * \
    [DH_Link.revolute(0, 2, 0, mass=1, com=Point3(-1, 0, 0))]

# Denavit-Hartenberg parameters for RRR manipulator
# Note that the rest configuration has the arm perpendicular to shoulder axis.
RRR_calibration_dh = [
    DH_Link.revolute(1, 0, 90),  # 12
    DH_Link.revolute(0, 1, 0),  # 23
    DH_Link.revolute(0, 1, -90)  # 34
]

# Denavit-Hartenberg parameters for R6 manipulator: RRR + wrist
R6_calibration_dh = RRR_calibration_dh + [
    DH_Link.revolute(0, 0, 90),  # 45
    DH_Link.revolute(0, 0, -90),  # 56
    DH_Link.revolute(1, 0, 0)  # 6e
]

# Denavit-Hartenberg parameters for PUMA manipulator theta, d, a, alpha
PUMA_calibration_dh = [
    DH_Link(0, 0, 0, +90, 'R', 0,
            Point3(0, 0, 0), np.diag([0, 0.35, 0])),
    DH_Link(0, 0.4318, 0, 0, 'R', 17.40, Point3(
        -0.3638, 0.006, 0.2275), np.diag([0.130, 0.524, 0.539])),
    DH_Link(0, 0.0203, 0.15005, -90, 'R', 4.80,
            Point3(-0.0203, -0.0141, 0.0700), np.diag([0.066, 0.086, 0.0125])),
    DH_Link(0, 0, 0.4318, +90, 'R', 0.82,
            Point3(0, 0.19, 0), np.diag([0.0018, 0.0013, 0.0018])),
    DH_Link(0, 0, 0, -90, 'R', 0.34,
            Point3(0, 0, 0), np.diag([0.0003, 0.0004, 0.0003])),
    DH_Link(0, 0, 0, 0, 'R', 0.09, Point3(
        0, 0, 0.032), np.diag([0.00015, 0.00015, 0.00004]))
]

# URDF parameters for RR manipulator
RR_calibration_urdf = [
    URDF_Link(Pose3(Rot3(), Point3(2, 0, 0)), utils.vector(0, 0, 1),
              'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), Z33),
    URDF_Link(Pose3(Rot3(), Point3(2, 0, 0)), utils.vector(0, 0, 1),
              'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), Z33)
]
