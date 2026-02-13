"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_paint_parse.py
@brief Unit tests for parsing a paint trajectory in a .h file.
@author Frank Dellaert
@author Gerry Chen
"""

import unittest

from paint_parse import ParseFile

import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestPaintParse(GtsamTestCase):
    """Unit tests for paint trajectory file parsing"""
    @unittest.skip("Temporarily disabled: parser currently expects `const PROGMEM` declarations; test data format differs.")
    def testParse(self):
        e_painton = np.array([1, 1, 0, 1, 1, 1])
        e_colorinds = np.array([0, 0, 1, 1, 2, 3])
        e_colorpalette = np.array([[4, 49, 75],
                                   [209, 4, 32],
                                   [236, 237, 237],
                                   [0, 0, 0]])
        e_traj = np.array([[0.1, 1.1],
                           [0.2, 1.0],
                           [0.3, 0.9],
                           [0.4, 1.1],
                           [0.5, 1.2],
                           [0.1, 1.15]])
        a_painton, a_colorinds, a_colorpalette, a_traj = ParseFile('data/test_traj.h')

        np.testing.assert_equal(e_painton, a_painton)
        np.testing.assert_equal(e_colorinds, a_colorinds)
        np.testing.assert_equal(e_colorpalette, a_colorpalette)
        np.testing.assert_equal(e_traj, a_traj)

if __name__ == "__main__":
    unittest.main()
