"""Checks for native-timestamp Gal3 preintegration in the example."""

import unittest

import gtsam
import numpy as np

from imu_aiding import preintegrate_intervals


class ImuAidingTests(unittest.TestCase):
    def test_stationary_calibrated_gyro_does_not_rotate(self) -> None:
        gyro_bias = np.array([-0.00215, -0.00467, -0.00206])
        bias = gtsam.imuBias.ConstantBias(np.zeros(3), gyro_bias)
        params = gtsam.PreintegrationParams.MakeSharedU(9.81)
        pims = preintegrate_intervals(
            np.array([0.0, 0.13, 0.71, 1.0]),
            np.tile([0.0, 0.0, 9.81], (4, 1)),
            np.tile(gyro_bias, (4, 1)),
            np.array([0.0, 0.5, 1.0]),
            params,
            bias,
        )
        state = gtsam.NavState(gtsam.Pose3(), np.zeros(3))
        for pim in pims:
            state = pim.predict(state, bias)
        np.testing.assert_allclose(state.pose().matrix(), np.eye(4), atol=1e-12)
        np.testing.assert_allclose(state.velocity(), np.zeros(3), atol=1e-12)

    def test_nonuniform_samples_and_fractional_boundaries(self) -> None:
        times = np.array([-0.013, 0.007, 0.021, 0.038, 0.09, 0.12, 0.21])
        boundaries = np.array([0.0, 0.05, 0.2])
        params = gtsam.PreintegrationParams.MakeSharedU(9.81)
        bias = gtsam.imuBias.ConstantBias()
        pims = preintegrate_intervals(
            times,
            np.tile([0.0, 0.0, 9.81], (len(times), 1)),
            np.zeros((len(times), 3)),
            boundaries,
            params,
            bias,
        )
        self.assertEqual(len(pims), 2)
        state = gtsam.NavState(gtsam.Pose3(), np.array([0.2, -0.4, 0.1]))
        for pim, duration in zip(pims, [0.05, 0.15]):
            self.assertAlmostEqual(pim.deltaTij(), duration)
            state = pim.predict(state, bias)
        np.testing.assert_allclose(state.position(), [0.04, -0.08, 0.02], atol=1e-12)
        np.testing.assert_allclose(state.velocity(), [0.2, -0.4, 0.1], atol=1e-12)

    def test_no_extrapolation_or_duplicate_timestamps(self) -> None:
        params = gtsam.PreintegrationParams.MakeSharedU(9.81)
        bias = gtsam.imuBias.ConstantBias()
        for times in (
            np.array([0.001, 0.1]),
            np.array([0.0, 0.0]),
            np.array([0.0, 0.09]),
        ):
            with self.assertRaises(ValueError):
                preintegrate_intervals(
                    times,
                    np.zeros((2, 3)),
                    np.zeros((2, 3)),
                    np.array([0.0, 0.1]),
                    params,
                    bias,
                )


if __name__ == "__main__":
    unittest.main()
