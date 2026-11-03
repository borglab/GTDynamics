"""Dataset-independent tests for the private G1 native helper."""

import unittest
import gtdynamics as gtd
import gtsam
import numpy as np
from _g1_kinodynamics import solve
from kinodynamic_settings import solver_settings


class TestKinodynamics(unittest.TestCase):
    def test_coupled_support_from_zero(self) -> None:
        """Upright measurements correct an initial tilt; only translation/yaw anchor."""
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
        for k in range(3):
            initial.insert(
                gtd.PoseKey(0, k), gtsam.Pose3(gtsam.Rot3.Ry(0.1), pose.translation())
            )
            measurements.add(
                gtsam.AttitudeFactorPose3(
                    gtd.PoseKey(0, k),
                    gtsam.Unit3(0, 0, 1),
                    gtsam.noiseModel.Isotropic.Sigma(2, 0.001),
                )
            )
            measurements.add(
                gtsam.PriorFactorVector(
                    gtd.LinearVelocityKey(0, k),
                    np.zeros(3),
                    gtsam.noiseModel.Isotropic.Sigma(3, 0.001),
                )
            )
        before = gtsam.Values(initial)
        options = solver_settings()
        self.assertEqual(options["maxIterations"], 5)
        self.assertEqual(options["gaugeSlice"], 0)
        self.assertEqual(options["retractionThreads"], 8)
        options["maxIterations"] = 20
        options["gaugeSlice"] = 1
        result = solve(
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
        options["retractionThreads"] = 1
        options["historyWindow"] = 0
        serial = solve(
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
        self.assertTrue(result["values"].equals(serial["values"], 1e-8))
        np.testing.assert_allclose(
            result["errorHistory"], serial["errorHistory"], atol=1e-8
        )
        # The complete example objective has the same value after a global
        # translation/yaw change, with the gauge anchor transformed alongside it.
        change = gtsam.Pose3(gtsam.Rot3.Rz(0.7), np.array([2.0, -3.0, 1.0]))
        shifted = gtsam.Values(initial)
        for k in range(3):
            key = gtd.PoseKey(0, k)
            shifted.update(key, change.compose(initial.atPose3(key)))
        transformed = solve(
            robot,
            trajectory,
            [gtd.Slice(k) for k in range(3)],
            [0.0, 0.02, 0.04],
            shifted,
            [measurements],
            ["Root observations"],
            [gtd.fixed_dynamics.Contact(sole, 0.1, 0.04, 0.6)],
            np.empty(0),
            options,
        )
        np.testing.assert_allclose(
            result["errorHistory"], transformed["errorHistory"], rtol=1e-7, atol=1e-7
        )
        self.assertFalse(hasattr(gtd, "solveKinodynamics"))
        self.assertTrue(initial.equals(before, 1e-5))
        self.assertTrue(result["feasible"])
        self.assertLess(result["equalityL2"], 1e-7)
        self.assertLess(result["inequalityL2"], 1e-7)
        self.assertLess(result["objectiveFinal"], result["objectiveInitial"])
        self.assertEqual(len(result["contactKeys"]), 3)
        self.assertEqual(len(result["errorHistory"]), result["iterations"] + 1)
        self.assertTrue(result["values"].atPose3(gtd.PoseKey(0, 1)).equals(pose, 1e-5))
        for k in range(3):
            np.testing.assert_array_equal(
                gtd.Twist(result["initialValues"], 0, k), np.zeros(6)
            )
            np.testing.assert_array_equal(
                gtd.TwistAccel(result["initialValues"], 0, k), np.zeros(6)
            )
            np.testing.assert_array_equal(
                result["initialValues"].atVector(result["contactKeys"][k]), np.zeros(6)
            )
            np.testing.assert_allclose(
                result["values"].atVector(result["contactKeys"][k]),
                [0, 0, 0, 0, 0, 98.1],
                atol=0.5,
            )


if __name__ == "__main__":
    unittest.main()
