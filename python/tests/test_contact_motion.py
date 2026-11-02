"""Public contact-motion bindings accept real GTSAM poses and custom thresholds."""

import unittest

import gtsam
import gtdynamics as gtd
import numpy as np


class ContactMotionTests(unittest.TestCase):
    def test_discrete_support_graph(self) -> None:
        params = gtd.ContactSupportParams()
        self.assertEqual(params.transition.shape, (4, 4))
        graph: gtsam.DiscreteFactorGraph = gtd.contactSupportGraph(
            [0.001] * 20, [0.02] * 20
        )
        self.assertEqual(graph.size(), 114)
        result = graph.optimize(gtsam.Ordering.OrderingType.NATURAL)
        self.assertTrue(
            all(result[2 * k] == 1 and result[2 * k + 1] == 0 for k in range(20))
        )
        params.transition = np.full((4, 4), 1 / 4)
        self.assertEqual(gtd.contactSupportGraph([0, 0], [0, 0], params).size(), 6)
        with self.assertRaises(ValueError):
            gtd.contactSupportGraph([-1], [0])

    def test_duration_prior_binding(self) -> None:
        params = gtd.ContactSupportParams()
        self.assertEqual(params.singleSliceWeight, 0.1)
        params.singleSliceWeight = 0.2
        graph = gtd.contactSupportGraph([0.001] * 3, [0.001] * 3, params)
        self.assertEqual(graph.size(), 12)
        values = gtsam.DiscreteValues()
        values[0], values[2], values[4] = 0, 1, 0
        self.assertAlmostEqual(graph.at(10)(values), 0.2)
        params.singleSliceWeight = 1
        graph = gtd.contactSupportGraph([0.001] * 3, [0.001] * 3, params)
        self.assertEqual(graph.at(10)(values), 1)
        params.singleSliceWeight = 0
        with self.assertRaises(ValueError):
            gtd.contactSupportGraph([0.001] * 3, [0.001] * 3, params)

    def test_hard_walking_constraints(self) -> None:
        constraints = gtd.contactWalkingConstraints(3)
        self.assertEqual(constraints.size(), 5)
        graph = gtd.contactSupportGraph([0.001, 0.02, 0.02], [0.02, 0.02, 0.001])
        graph.push_back(constraints)
        result = graph.optimize(gtsam.Ordering.OrderingType.NATURAL)
        self.assertEqual(constraints(result), 1)
        result[0], result[1] = 0, 0
        self.assertEqual(constraints(result), 0)
        self.assertEqual(gtd.contactWalkingConstraints(1).size(), 1)
        with self.assertRaises(ValueError):
            gtd.contactWalkingConstraints(0)

    def test_velocity_factor_bindings(self) -> None:
        graph: gtsam.DiscreteFactorGraph = gtd.contactVelocityFactors([-0.3], [0.3])
        self.assertEqual(graph.size(), 2)
        state = gtsam.DiscreteValues()
        state[0] = 1
        state[1] = 1
        self.assertAlmostEqual(graph.at(0)(state), 1 / (1 + np.exp(-3)))
        self.assertAlmostEqual(graph.at(1)(state), 1 / (1 + np.exp(3)))
        graph = gtd.contactVelocityFactors([1000], [-1000], 0.2, 0.05)
        self.assertAlmostEqual(graph.at(0)(state), 0.05)
        self.assertAlmostEqual(graph.at(1)(state), 0.95)
        with self.assertRaises(ValueError):
            gtd.contactVelocityFactors([0], [])

    def test_walking_support_prior(self) -> None:
        times = [0.0, 0.05, 0.1, 0.15, 0.2]
        rocking = gtd.contactPoseMotionLabels(
            times, [gtsam.Pose3(gtsam.Rot3.Ry(2 * t), np.zeros(3)) for t in times]
        )
        swing = gtd.contactPoseMotionLabels(
            times, [gtsam.Pose3(gtsam.Rot3(), np.array([t, 0, 0])) for t in times]
        )
        self.assertEqual(gtd.walkingContactLabels(rocking, swing), [-1, -1, 2, -1, -1])
        self.assertEqual(gtd.walkingContactLabels(swing, rocking)[2], 0)
        self.assertEqual(rocking[2].label, 0)
        params = gtd.ContactPoseMotionParams()
        params.movingSpeed = 2
        params.movingExcursion = 0.2
        self.assertEqual(gtd.walkingContactLabels(rocking, swing, params)[2], 0)
        with self.assertRaises(ValueError):
            gtd.walkingContactLabels(rocking, [])

    def test_contact_pose_rotation_and_defaults(self) -> None:
        times = [0.0, 0.05, 0.1, 0.15, 0.2]
        poses = [gtsam.Pose3(gtsam.Rot3.Rz(2 * t), np.zeros(3)) for t in times]
        sample = gtd.contactPoseMotionLabels(times, poses)[2]
        self.assertEqual(sample.label, 0)
        self.assertAlmostEqual(sample.maxSpeed, 0.0)
        self.assertAlmostEqual(sample.angularSpeed, 2.0)
        params = gtd.ContactPoseMotionParams()
        params.stationaryAngularSpeed = 3.0
        params.movingAngularSpeed = 4.0
        params.stationaryAngularExcursion = 0.3
        params.movingAngularExcursion = 0.4
        self.assertEqual(gtd.contactPoseMotionLabels(times, poses, params)[2].label, 1)

    def test_default_and_custom_params(self) -> None:
        times = [0.0, 0.05, 0.1, 0.15, 0.2]
        poses = [gtsam.Pose3(gtsam.Rot3.Rz(2 * time), np.zeros(3)) for time in times]
        points = [np.zeros(3), np.array([0.2, 0.0, 0.0])]
        samples = gtd.contactMotionLabels(times, poses, points)
        self.assertEqual(len(samples), len(times))
        self.assertFalse(samples[0].completeWindow)
        self.assertEqual(samples[0].label, -1)
        self.assertTrue(samples[2].completeWindow)
        self.assertEqual(samples[2].label, 0)
        self.assertAlmostEqual(samples[2].pointSpeeds[0], 0.0)
        self.assertGreater(samples[2].pointSpeeds[1], 0.3)
        params = gtd.ContactMotionParams()
        params.stationarySpeed = 0.5
        params.movingSpeed = 1.0
        params.stationaryExcursion = 0.1
        params.movingExcursion = 0.2
        self.assertEqual(
            gtd.contactMotionLabels(times, poses, points, params)[2].label, 1
        )
        with self.assertRaises(ValueError):
            gtd.contactMotionLabels([0.0, 0.0], poses[:2], points)


if __name__ == "__main__":
    unittest.main()
