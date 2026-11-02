# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_sole_support.py
# @brief Test planar hulls and canonical G1 support rectangles.
# @author GTDynamics contributors

"""Check conservative G1 sole patches independently of browser serialization."""

import unittest

import numpy as np

from g1_model import load_g1_model
from sole_support import planar_hull, support_rectangle


class TestSupportFootprint(unittest.TestCase):
    def test_hull_discards_interior_duplicate_and_collinear_points(self) -> None:
        points = np.array(
            [[1, 1], [0, 0], [0, 1], [1, 0], [0, 0], [0.5, 0], [0.5, 0.5]]
        )
        np.testing.assert_array_equal(
            planar_hull(points), [[0, 0], [1, 0], [1, 1], [0, 1]]
        )
        with self.assertRaises(ValueError):
            planar_hull(np.array([[0, 0], [1, 1], [2, 2]]))

    def test_canonical_support_matches_the_published_capsule_hull(self) -> None:
        model = load_g1_model()
        for side in ("left", "right"):
            np.testing.assert_allclose(
                support_rectangle(model, side),
                [0.07630448211559485, 0.026930993687857015],
                rtol=0,
                atol=1e-14,
            )


if __name__ == "__main__":
    unittest.main()
