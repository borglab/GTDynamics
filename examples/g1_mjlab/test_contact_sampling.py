"""Buffered native selection, sparse key mapping, and irregular-time IMU seeds."""

import unittest

import gtsam
import gtdynamics as gtd
import numpy as np

from imu_aiding import BIAS_KEY, ImuAiding, preintegrate_intervals
from kinematic_walk import build_problem, contact_slices, initialize_contact_slices
from mujoco_io import KinematicRecording


class ContactSamplingTests(unittest.TestCase):
    def recording(self, flight: bool = False) -> KinematicRecording:
        root = gtd.Link(1, "root", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"root": root}, {})
        points = gtd.PointOnLinks(
            [
                gtd.PointOnLink(root, np.array(p, dtype=float))
                for p in [[0, 0, -1], [1, 0, -1], [0, 1, -1]]
            ]
        )
        contact = gtd.FootContactConstraintSpec(points)
        phases = [gtd.Phase(0, 30, contact)]
        if flight:
            phases = [
                gtd.Phase(0, 11, contact),
                gtd.Phase(12, 14, gtd.FootContactConstraintSpec(gtd.PointOnLinks([]))),
                gtd.Phase(15, 30, contact),
            ]
        return KinematicRecording(
            robot,
            root,
            gtsam.Values(),
            gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 1.0])),
            gtd.Trajectory(gtd.WalkCycle(phases), 1),
            31,
            0.1,
            0.0,
            np.arange(93, dtype=float).reshape(31, 3),
            {},
        )

    def test_retained_slices_have_native_poses_and_source_times(self) -> None:
        source = self.recording()
        walk, seed, info = initialize_contact_slices(source)
        np.testing.assert_array_equal(walk.source_indices, [1, 10, 20, 29])
        np.testing.assert_allclose(walk.times, [0.1, 1.0, 2.0, 2.9])
        np.testing.assert_array_equal(
            walk.reference_root_positions,
            source.reference_root_positions[[1, 10, 20, 29]],
        )
        self.assertEqual(info["contact_buffer"], 1)
        self.assertEqual(
            [k for k, _, _ in contact_slices(walk.trajectory)], list(range(4))
        )
        costs, constraints, keys = build_problem(walk)
        for k in range(4):
            self.assertTrue(seed.atPose3(gtd.PoseKey(1, k)).equals(source.anchor, 1e-8))
        self.assertLess(costs.error(seed.extract(keys)), 1e-10)
        self.assertEqual(constraints.size(), 0)

    def test_no_stationary_edge_across_skipped_flight(self) -> None:
        options = gtd.contact_init.Options()
        options.addBetweenFactors = True
        walk, seed, _ = initialize_contact_slices(self.recording(flight=True), options)
        self.assertEqual(len(walk.contact_breaks), 1)
        k, link = next(iter(walk.contact_breaks))
        groups = {}
        build_problem(walk, objective_groups=groups)
        self.assertTrue(
            all(
                groups["stationary"].at(i).keys()[1] != gtd.PoseKey(link, k)
                for i in range(groups["stationary"].size())
            )
        )
        self.assertEqual(
            list(contact_slices(walk.trajectory, walk.contact_breaks))[k][2], []
        )

    def test_irregular_velocity_seed_and_pim_durations(self) -> None:
        walk, seed, _ = initialize_contact_slices(self.recording())
        velocity = np.array([0.3, -0.2, 0.1])
        for k, time in enumerate(walk.times):
            seed.update(gtd.PoseKey(1, k), gtsam.Pose3(gtsam.Rot3(), time * velocity))
        bias = gtsam.imuBias.ConstantBias()
        aiding = ImuAiding(gtsam.NonlinearFactorGraph(), {}, bias)
        initial = aiding.initialize(walk, seed)
        for k in range(walk.count):
            np.testing.assert_allclose(
                initial.atVector(gtd.LinearVelocityKey(1, k)), velocity, atol=1e-12
            )
        times = np.linspace(0, 3, 601)
        pims = preintegrate_intervals(
            times,
            np.tile([0, 0, 9.81], (len(times), 1)),
            np.zeros((len(times), 3)),
            walk.times,
            gtsam.PreintegrationParams.MakeSharedU(9.81),
            bias,
        )
        np.testing.assert_allclose(
            [pim.deltaTij() for pim in pims], [0.9, 1.0, 0.9], atol=1e-12
        )
        factors = gtd.Kinematics().imuFactors(walk.trajectory, 1, pims, BIAS_KEY)
        self.assertEqual(factors.size(), 3)
        self.assertLess(factors.error(initial), 1e-15)


if __name__ == "__main__":
    unittest.main()
