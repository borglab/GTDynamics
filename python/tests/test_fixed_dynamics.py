"""Direct bindings for material sole wrenches and fixed-kinematics dynamics."""

import unittest

import gtdynamics as gtd
import gtsam
import numpy as np


class TestFixedDynamics(unittest.TestCase):
    def test_static_support_and_determined_trajectory(self) -> None:
        """An entirely determined trajectory has no LM variables to optimize."""
        body = gtd.Link(0, "body", 10.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"body": body}, {})
        sole = gtd.PoseOnLink(
            body, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -0.5])), True
        )
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 2, gtd.PoseContactConstraintSpec([sole]))]),
            1,
        )
        fixed = gtsam.Values()
        for k in range(3):
            fixed.insert(
                gtd.PoseKey(0, k),
                gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.5])),
            )
            fixed.insert(gtd.TwistKey(0, k), np.zeros(6))
            fixed.insert(gtd.TwistAccelKey(0, k), np.zeros(6))
        result = gtd.solveFixedKinematicsDynamics(
            robot,
            trajectory,
            [gtd.Slice(k) for k in range(3)],
            [0.0, 0.08, 0.3],
            fixed,
            [gtd.fixed_dynamics.Contact(sole, 0.1, 0.04, 0.6)],
            np.empty(0),
            gtd.fixed_dynamics.Options(),
        )
        self.assertTrue(result.feasible)
        self.assertTrue(result.stationary)
        self.assertFalse(result.softDynamics)
        self.assertEqual(result.reducedDimension, 0)
        self.assertEqual(result.iterations, 0)
        self.assertEqual(len(result.contactKeys), 3)
        for key in result.contactKeys:
            np.testing.assert_allclose(
                result.values.atVector(key), [0, 0, 0, 0, 0, 98.1], atol=1e-10
            )

    def test_contact_wrench_frames_and_cone(self) -> None:
        """The public factor and utility use the same torque-first convention."""
        transform = gtsam.Pose3(gtsam.Rot3(), np.array([0.1, 0.0, -0.5]))
        sole = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 100.0])
        com = gtd.contactWrenchToCom(transform, sole)
        np.testing.assert_allclose(com, [0, -10, 0, 0, 0, 100], atol=1e-10)
        factor = gtd.ContactWrenchFactor(
            1, 2, transform, gtsam.noiseModel.Unit.Create(6)
        )
        values = gtsam.Values()
        values.insert(1, sole)
        values.insert(2, com)
        self.assertAlmostEqual(factor.error(values), 0.0)
        cone = gtd.RectangularContactWrench(0.1, 0.04, 0.6)
        self.assertEqual(cone.matrix().shape, (17, 6))
        self.assertLessEqual(np.max(cone.matrix() @ sole), 0)


if __name__ == "__main__":
    unittest.main()
