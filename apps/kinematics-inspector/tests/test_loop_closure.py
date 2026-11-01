"""Check world-axis weighting and orientation freedom of the closure observation."""

from pathlib import Path
import sys
from types import SimpleNamespace
from typing import cast
import unittest

import gtsam
import gtdynamics as gtd
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from export_recording import KinematicRecording, loop_closure_factor
from import_openio import stationary_imu_calibration


class LoopClosureTests(unittest.TestCase):
    def test_stationary_calibration_uses_raw_pelvis_and_excludes_motion(self) -> None:
        times = np.array([0.0, 0.4, 3.5, 24.9, 25.0, 204.0])
        gyro_bias = np.array([-0.002, -0.005, -0.002])
        imus = np.zeros((2, len(times), 9))
        imus[0, :4, 3:6] = gyro_bias
        imus[0, :4, 2] = 9.74
        imus[0, 4:, 3:6] = 100.0  # Motion is not a calibration sample.
        imus[1, :, 3:6] = 200.0  # Other body parts are derived streams.
        calibration = stationary_imu_calibration(times, imus)
        self.assertEqual(calibration["sampleCount"], 4)
        np.testing.assert_allclose(calibration["gyroMeanRadiansPerSecond"], gyro_bias)
        np.testing.assert_allclose(
            calibration["meanAccelerationMetersPerSecondSquared"], [0, 0, 9.74]
        )

    def test_world_position_only(self) -> None:
        root = gtd.Link(1, "root", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        position = np.array([1.0, 2.0, 3.0])
        last = gtd.PoseKey(root.id(), 19)
        for initial_rotation, final_rotation in (
            (gtsam.Rot3(), gtsam.Rot3()),
            (gtsam.Rot3.RzRyRx(0.4, 0.3, 1), gtsam.Rot3.RzRyRx(-0.8, 0.7, -2)),
        ):
            walk = cast(
                KinematicRecording,
                SimpleNamespace(
                    root=root, count=20, anchor=gtsam.Pose3(initial_rotation, position)
                ),
            )
            factor = loop_closure_factor(walk)
            self.assertIsInstance(factor, gtsam.PoseTranslationPrior3D)
            self.assertEqual(list(factor.keys()), [last])
            for axis, sigma in enumerate([0.30, 0.30, 0.05]):
                with self.subTest(axis=axis, rotated=initial_rotation.rpy()):
                    displacement = np.zeros(3)
                    displacement[axis] = sigma
                    values = gtsam.Values()
                    values.insert(
                        last, gtsam.Pose3(final_rotation, position + displacement)
                    )
                    self.assertAlmostEqual(factor.error(values), 0.5)
                    # Only the final pose is connected, with zero rotation columns.
                    jacobian, _ = factor.linearize(values).jacobian()
                    self.assertEqual(jacobian.shape, (3, 6))
                    np.testing.assert_allclose(jacobian[:, :3], 0, atol=1e-12)
                    values.update(last, gtsam.Pose3(final_rotation, position))
                    self.assertAlmostEqual(factor.error(values), 0)


if __name__ == "__main__":
    unittest.main()
