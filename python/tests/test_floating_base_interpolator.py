"""Public bindings: dynamic joint dimension, frames, queries, and defaults."""

import unittest

import gtsam
import gtdynamics as gtd
import numpy as np


class TestFloatingBaseInterpolator(unittest.TestCase):
    def make_interpolator(self) -> gtd.FloatingBaseInterpolator:
        """Use a two-joint robot with noncontiguous IDs and mixed joint units."""
        links = [
            gtd.Link(i, name, 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
            for i, name in [(3, "root"), (9, "middle"), (12, "tool")]
        ]
        joints = [
            gtd.RevoluteJoint(
                7, "hinge", gtsam.Pose3(), links[0], links[1], np.array([0.0, 0.0, 1.0])
            ),
            gtd.PrismaticJoint(
                18,
                "slider",
                gtsam.Pose3(),
                links[1],
                links[2],
                np.array([1.0, 0.0, 0.0]),
            ),
        ]
        for joint, parent, child in zip(joints, links, links[1:]):
            parent.addJoint(joint)
            child.addJoint(joint)
        robot = gtd.Robot(
            {link.name(): link for link in links},
            {joint.name(): joint for joint in joints},
        )
        self.assertEqual([7, 18], [joint.id() for joint in robot.orderedJoints()])
        return gtd.FloatingBaseInterpolator(
            robot=robot,
            baseLink="root",
            baseProcessSpectralDensity=np.ones(6),
            jointProcessSpectralDensity=np.array([np.deg2rad(30.0) ** 2, 1.0]),
            jointMeasurementSigmas=np.array([np.deg2rad(0.5), 0.001]),
        )

    def test_dynamic_vector_and_fk(self) -> None:
        """A Vector GP covers both joints; default query indices expose native FK."""
        gp = self.make_interpolator()
        for time in (0.0, 0.3, 1.0):
            gp.addKnot(
                time=time,
                basePose=gtsam.Pose3(),
                jointAngles=np.array([0.2 * time, 0.4 * time]),
            )
        gp.addJointMeasurement(time=0.5, jointAngles=np.array([0.1, 0.2]))
        gp.addBaseVelocityMeasurement(
            time=0.0, worldLinearVelocity=np.zeros(3), sigma=0.1
        )
        result: gtd.FloatingBaseInterpolation = gp.solve()
        state: gtd.FloatingBaseState = result.interpolate(time=0.6)
        np.testing.assert_allclose(state.jointAngles, [0.12, 0.24], atol=1e-9)
        np.testing.assert_allclose(state.jointVelocities, [0.2, 0.4], atol=1e-9)
        self.assertEqual([7, 18], result.jointIds())
        self.assertEqual(["hinge", "slider"], result.jointNames())
        values: gtsam.Values = result.values(0.6)
        self.assertTrue(values.exists(gtd.PoseKey(12, 0)))
        np.testing.assert_allclose(gtd.Twist(values, 3, 0), np.zeros(6), atol=1e-9)
        self.assertLess(result.jointMeasurementRms(), 1e-9)
        self.assertGreaterEqual(result.solveSeconds(), 0)
        self.assertGreater(result.graphConstructionSeconds(), 0)

    def test_validation_and_independent_results(self) -> None:
        """Input mutation and later solves preserve previous result snapshots."""
        gp = self.make_interpolator()
        with self.assertRaises(ValueError):
            gp.addKnot(0.0, gtsam.Pose3(), np.zeros(1))
        for time in (0.0, 1.0):
            gp.addKnot(time, gtsam.Pose3(), np.zeros(2))
        result: gtd.FloatingBaseInterpolation = gp.solve()
        with self.assertRaises(IndexError):
            result.interpolate(2.0)
        gp.addJointMeasurement(0.5, np.array([0.2, 0.1]))
        newer: gtd.FloatingBaseInterpolation = gp.solve()
        self.assertGreater(newer.interpolate(0.5).jointAngles[0], 0.0)
        np.testing.assert_array_equal(result.interpolate(0.5).jointAngles, [0.0, 0.0])
        gp.addKnot(2.0, gtsam.Pose3(), np.zeros(2))
        self.assertEqual([0.0, 1.0], result.knotTimes())
        self.assertEqual([0.0, 1.0], newer.knotTimes())
        self.assertEqual([0.0, 1.0, 2.0], gp.solve().knotTimes())
        del gp
        self.assertTrue(result.values(time=0.5, k=3).exists(gtd.PoseKey(12, 3)))
        self.assertEqual(0.0, result.jointMeasurementRms())

    def test_result_outlives_problem(self) -> None:
        """The returned object has no Python lifetime dependency on the builder."""

        def fit() -> gtd.FloatingBaseInterpolation:
            problem = self.make_interpolator()
            problem.addKnot(0.0, gtsam.Pose3(), np.zeros(2))
            problem.addKnot(1.0, gtsam.Pose3(), np.array([0.2, 0.4]))
            return problem.solve()

        result = fit()
        np.testing.assert_allclose(result.interpolate(0.5).jointAngles, [0.1, 0.2])
        self.assertTrue(result.values(0.5).exists(gtd.PoseKey(12, 0)))
        self.assertEqual([7, 18], result.jointIds())
        with self.assertRaises(TypeError):
            gtd.FloatingBaseInterpolation()


if __name__ == "__main__":
    unittest.main()
