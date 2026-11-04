# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_fixed_dynamics.py
# @brief Test direct Python bindings for sole-wrench trajectory dynamics.
# @author GTDynamics contributors

"""Direct bindings for material sole wrenches and fixed-kinematics dynamics."""

import unittest

import gtdynamics as gtd
import gtsam
import numpy as np


class TestFixedDynamics(unittest.TestCase):
    def test_static_support_trajectory(self) -> None:
        """Acceleration priors recover static support with fixed poses and twists."""
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
        acceleration_priors = gtsam.Values()
        for k in range(3):
            fixed.insert(
                gtd.PoseKey(0, k),
                gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.5])),
            )
            fixed.insert(gtd.TwistKey(0, k), np.zeros(6))
            acceleration_priors.insert(gtd.TwistAccelKey(0, k), np.zeros(6))
        fixed_before = gtsam.Values(fixed)
        priors_before = gtsam.Values(acceleration_priors)
        options = gtd.fixed_dynamics.Options()
        options.baseAngularAccelerationSigmaDegrees = 0.01
        options.baseLinearAccelerationSigma = 0.001
        result = gtd.solveFixedKinematicsDynamics(
            robot,
            trajectory,
            [gtd.Slice(k) for k in range(3)],
            [0.0, 0.08, 0.3],
            fixed,
            acceleration_priors,
            [gtd.fixed_dynamics.Contact(sole, 0.1, 0.04, 0.6)],
            np.empty(0),
            options,
        )
        self.assertTrue(result.feasible)
        self.assertTrue(result.stationary)
        self.assertFalse(result.softDynamics)
        # Six root-acceleration coordinates per slice remain free. Exact
        # Newton-Euler balance determines the single supporting sole wrench.
        self.assertEqual(result.reducedDimension, 18)
        self.assertTrue(fixed.equals(fixed_before, 1e-12))
        self.assertTrue(acceleration_priors.equals(priors_before, 1e-12))
        self.assertEqual(len(result.contactKeys), 3)
        for key in result.contactKeys:
            np.testing.assert_allclose(
                result.values.atVector(key), [0, 0, 0, 0, 0, 98.1], atol=1e-6
            )
        for k in range(3):
            np.testing.assert_allclose(
                result.values.atVector(gtd.TwistAccelKey(0, k)), 0, atol=1e-8
            )
        self.assertLess(result.maxForceResidual, 1e-8)
        self.assertLess(result.maxMomentResidual, 1e-8)

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
