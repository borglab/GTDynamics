# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file test_world_velocity.py
# @brief Exercise the reusable velocity-frame Python binding.
# @author GTDynamics contributors
"""Body/world velocity frames through the Python binding."""

import unittest

import gtdynamics as gtd
import gtsam
import numpy as np


class TestWorldVelocity(unittest.TestCase):
    """Check the Python boundary independently of the C++ fixture."""

    def test_world_velocity_frame(self) -> None:
        """The wrapped factor rotates COM linear velocity into world axes."""
        pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.2, -0.3, 0.5), np.ones(3))
        twist = np.array([0.2, 0.3, -0.1, 0.8, -0.4, 0.5])
        velocity = pose.rotation().matrix() @ twist[3:]
        factor = gtd.WorldVelocityFactor(1, 2, 3, gtsam.noiseModel.Unit.Create(3))
        values = gtsam.Values()
        values.insert(1, pose)
        values.insert(2, twist)
        values.insert(3, velocity)
        self.assertAlmostEqual(factor.error(values), 0.0)
        values.update(3, velocity + np.array([1.0, 0.0, 0.0]))
        self.assertAlmostEqual(factor.error(values), 0.5)


if __name__ == "__main__":
    unittest.main()
