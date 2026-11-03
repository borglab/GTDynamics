# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file test_kinodynamics.py
# @brief Exercise direct coupled-solver and velocity-frame Python bindings.
# @author GTDynamics contributors
"""Native kinodynamics bindings on a small, dataset-independent support graph."""

import unittest

import gtdynamics as gtd
import gtsam
import numpy as np


class TestKinodynamics(unittest.TestCase):
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

    def test_coupled_support_from_zero(self) -> None:
        """Bindings preserve the selected gauge and expose zero initial dynamics."""
        body = gtd.Link(0, "body", 10.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"body": body}, {})
        sole = gtd.PoseOnLink(
            body, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -0.5])), True
        )
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 2, gtd.PoseContactConstraintSpec([sole]))]),
            1,
        )
        pose = gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.5]))
        initial = gtsam.Values()
        measurements = gtsam.NonlinearFactorGraph()
        pose_rotation_sigma_degrees = 0.05
        pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([np.deg2rad(pose_rotation_sigma_degrees)] * 3 + [0.001] * 3)
        )
        for k in range(3):
            initial.insert(gtd.PoseKey(0, k), pose)
            measurements.add(
                gtsam.PriorFactorPose3(gtd.PoseKey(0, k), pose, pose_noise)
            )
            measurements.add(
                gtsam.PriorFactorVector(
                    gtd.LinearVelocityKey(0, k),
                    np.zeros(3),
                    gtsam.noiseModel.Isotropic.Sigma(3, 0.001),
                )
            )
        before = gtsam.Values(initial)
        options: gtd.kinodynamics.Options = gtd.kinodynamics.Options()
        self.assertEqual(options.maxIterations, 50)
        self.assertEqual(options.gaugeSlice, 0)
        self.assertEqual(options.retractionThreads, 8)
        options.maxIterations = 5
        options.gaugeSlice = 1
        result: gtd.kinodynamics.Result = gtd.solveKinodynamics(
            robot,
            trajectory,
            [gtd.Slice(k) for k in range(3)],
            [0.0, 0.02, 0.04],
            initial,
            [measurements],
            ["Root observations"],
            [gtd.fixed_dynamics.Contact(sole, 0.1, 0.04, 0.6)],
            np.empty(0),
            options,
        )
        self.assertTrue(initial.equals(before, 1e-12))
        self.assertTrue(result.feasible)
        self.assertLess(result.equalityL2, 1e-7)
        self.assertLess(result.inequalityL2, 1e-7)
        self.assertLess(result.objectiveFinal, result.objectiveInitial)
        self.assertEqual(len(result.contactKeys), 3)
        self.assertEqual(len(result.errorHistory), result.iterations + 1)
        self.assertTrue(result.values.atPose3(gtd.PoseKey(0, 1)).equals(pose, 1e-12))
        for k in range(3):
            np.testing.assert_array_equal(
                gtd.Twist(result.initialValues, 0, k), np.zeros(6)
            )
            np.testing.assert_array_equal(
                gtd.TwistAccel(result.initialValues, 0, k), np.zeros(6)
            )
            np.testing.assert_array_equal(
                result.initialValues.atVector(result.contactKeys[k]), np.zeros(6)
            )
            np.testing.assert_allclose(
                result.values.atVector(result.contactKeys[k]),
                [0, 0, 0, 0, 0, 98.1],
                atol=0.5,
            )


if __name__ == "__main__":
    unittest.main()
