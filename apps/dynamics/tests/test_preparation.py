"""Check the planar support patch and independent GTSAM feasibility programs."""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import gtsam
import gtdynamics as gtd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from export_recording import planar_hull, support_rectangle
from g1_model import load_g1_model
from validate_recording import linear_feasible


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


class TestContactFeasibility(unittest.TestCase):
    def test_resultant_cone_accepts_support_and_rejects_tension_and_excess_cop(
        self,
    ) -> None:
        cone = gtd.RectangularContactWrench(0.1, 0.05, 0.8).matrix()
        for wrench, feasible in (
            ([0, 0, 0, 0, 0, 100], True),
            ([0, 0, 0, 0, 0, -100], False),
            ([6, 0, 0, 0, 0, 100], False),
            ([0, 0, 0, 60, 0, 100], False),
        ):
            with self.subTest(wrench=wrench):
                self.assertEqual(
                    linear_feasible(np.eye(6), np.asarray(wrench, dtype=float), cone),
                    feasible,
                )

    def test_nonnegative_generators_and_unattainable_equalities(self) -> None:
        for target in (1.0, -1.0):
            self.assertEqual(
                linear_feasible(np.ones((1, 2)), np.array([target]), -np.eye(2)),
                target > 0,
            )
        self.assertFalse(linear_feasible(np.zeros((1, 2)), np.ones(1), -np.eye(2)))

    def test_no_contact_supports_only_zero_load(self) -> None:
        for load in (0.0, 1.0):
            self.assertEqual(
                linear_feasible(np.zeros((1, 0)), np.array([load]), np.zeros((0, 0))),
                load == 0,
            )

    def test_solver_failure_is_reported_separately_from_infeasibility(self) -> None:
        with patch.object(
            gtsam.LpProblem, "optimize", side_effect=RuntimeError("failed")
        ):
            with self.assertRaisesRegex(RuntimeError, "failed"):
                linear_feasible(np.ones((1, 2)), np.ones(1), -np.eye(2))


if __name__ == "__main__":
    unittest.main()
