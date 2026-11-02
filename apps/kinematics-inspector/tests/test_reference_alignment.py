"""Rigid reference fitting must recover SE(3), without reflection or scale."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from pack_recordings import reference_alignment


class ReferenceAlignmentTests(unittest.TestCase):
    def test_recovers_transform_and_is_invariant_to_common_origin(self) -> None:
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [0.0, 0.0, 3.0],
                [1.0, 2.0, 3.0],
            ]
        )
        rotation = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
        translation = np.array([2.0, -4.0, 1.0])
        estimate = points @ rotation.T + translation
        fit = reference_alignment(
            {"pelvisWorld": {"reference": points, "estimate": estimate}}
        )
        np.testing.assert_allclose(fit["rotation"], rotation, atol=1e-12)
        np.testing.assert_allclose(fit["translation"], translation, atol=1e-12)
        self.assertLess(fit["rmsAfterMeters"], 1e-12)
        self.assertEqual(fit["scale"], 1.0)

    def test_does_not_fit_scale_or_reflection(self) -> None:
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]
        )
        for estimate in (points * 2, points * np.array([-1.0, 1.0, 1.0])):
            fit = reference_alignment(
                {"pelvisWorld": {"reference": points, "estimate": estimate}}
            )
            rotation = np.array(fit["rotation"])
            self.assertAlmostEqual(np.linalg.det(rotation), 1.0)
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
            self.assertGreater(fit["rmsAfterMeters"], 0.1)
            self.assertLessEqual(fit["rmsAfterMeters"], fit["rmsBeforeMeters"] + 1e-12)


if __name__ == "__main__":
    unittest.main()
