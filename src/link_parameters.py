#!/usr/bin/env python
"""
Test forward dynamics using factor graphs.
Author: Frank Dellaert and Mandy Xie
"""

# pylint: disable=C0103, E1101, E0401

from __future__ import print_function

import math
import utils
from dh_link import DH_Link
from urdf_link import URDF_Link
from gtsam import Point3, Pose3, Rot3

# Denavit-Hartenberg parameters for RR manipulator
RR_calibration_dh = 2 * \
    [DH_Link(0, 0, 2, 0, 'R', 1, Point3(-1, 0, 0), [0, 0, 0])]

# Denavit-Hartenberg parameters for PUMA manipulator theta, d, a, alpha
PUMA_calibration_dh = [
    DH_Link(0, 0.0000, 0.0000, +90, 'R', 0, Point3(0, 0, 0), [0, 0.35, 0]),
    DH_Link(0, 0.4318, 0, 0.0, 'R', 17.40, Point3(
        -0.3638, 0.006, 0.2275), [0.130, 0.524, 0.539]),
    DH_Link(0, 0.0203, 0.15005, -90, 'R', 4.80,
            Point3(-0.0203, -0.0141, 0.0700), [0.066, 0.086, 0.0125]),
    DH_Link(0, 0, 0.4318, +90, 'R', 0.82,
            Point3(0, 0.19, 0), [0.0018, 0.0013, 0.0018]),
    DH_Link(0, 0.0000, 0.0000, -90, 'R', 0.34,
            Point3(0, 0, 0), [0.0003, 0.0004, 0.0003]),
    DH_Link(0, 0.0000, 0.0000, 0.0, 'R', 0.09, Point3(
        0, 0, 0.032), [0.00015, 0.00015, 0.00004])
]

# URDF parameters for RR manipulator
RR_calibration_urdf = [
    URDF_Link(Pose3(Rot3(), Point3(2, 0, 0)), utils.vector(0, 0, 1), 'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), [0, 0, 0]),
    URDF_Link(Pose3(Rot3(), Point3(2, 0, 0)), utils.vector(0, 0, 1), 'R', 1, Pose3(Rot3(), Point3(1, 0, 0)), [0, 0, 0])
]
