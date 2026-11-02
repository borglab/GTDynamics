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
        gp.solve()
        state: gtd.FloatingBaseState = gp.interpolate(time=0.6)
        np.testing.assert_allclose(state.jointAngles, [0.12, 0.24], atol=1e-9)
        np.testing.assert_allclose(state.jointVelocities, [0.2, 0.4], atol=1e-9)
        self.assertEqual([7, 18], gp.jointIds())
        self.assertEqual(["hinge", "slider"], gp.jointNames())
        values: gtsam.Values = gp.values(0.6)
        self.assertTrue(values.exists(gtd.PoseKey(12, 0)))
        np.testing.assert_allclose(gtd.Twist(values, 3, 0), np.zeros(6), atol=1e-9)
        self.assertLess(gp.jointMeasurementRms(), 1e-9)
        self.assertGreaterEqual(gp.solveSeconds(), 0)

    def test_validation_and_invalidation(self) -> None:
        """Invalid inputs and stale solutions fail through the actual wrapper."""
        gp = self.make_interpolator()
        with self.assertRaises(ValueError):
            gp.addKnot(0.0, gtsam.Pose3(), np.zeros(1))
        for time in (0.0, 1.0):
            gp.addKnot(time, gtsam.Pose3(), np.zeros(2))
        gp.solve()
        with self.assertRaises(IndexError):
            gp.interpolate(2.0)
        gp.addJointMeasurement(0.5, np.zeros(2))
        with self.assertRaises(RuntimeError):
            gp.interpolate(0.5)


if __name__ == "__main__":
    unittest.main()
