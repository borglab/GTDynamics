# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_preparation.py
# @brief Test feasible and infeasible contact loads with native GTSAM LPs.
# @author GTDynamics contributors

"""Check independent GTSAM contact-feasibility programs."""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import gtsam
import gtdynamics as gtd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from validate_recording import linear_feasible


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
