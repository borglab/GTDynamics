"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_utils.py
@brief Unit tests for python utility functions.
@author Frank Dellaert
@author Gerry Chen
"""

import unittest

import gtdynamics as gtd
import gtsam
from gtsam import Pose3, Rot3
import numpy as np
from utils import gc_map2gains
from gtsam.utils.test_case import GtsamTestCase

class TestUtils(GtsamTestCase):
    """Unit tests for planar CDPR"""
    def test_gc_map2gains(self):
        expected_K = np.array([
            [1, 2, 3, 4],  #
            [5, 6, 7, 8],  #
            [9, 10, 11, 12]
        ])
        expected_k = np.array([13, 14, 15]).reshape((-1, 1))

        gc_map = { i : gtsam.GaussianConditional(i, expected_k[i], )}

if __name__ == "__main__":
    unittest.main()
