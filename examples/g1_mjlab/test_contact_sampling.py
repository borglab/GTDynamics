# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file test_contact_sampling.py
# @brief Verify phase sampling, stance continuity, and irregular IMU intervals.
# @author GTDynamics contributors
"""Buffered native selection, sparse key mapping, and irregular-time IMU seeds."""

import unittest
from dataclasses import replace
from pathlib import Path

import gtsam
import gtdynamics as gtd
import numpy as np

from imu_aiding import BIAS_KEY, ImuAiding, preintegrate_intervals
from contact_phases import contact_slices, initialization_slices, stance_observations
from kinematic_smoothing import build_problem, solve_cm
from kinematic_initialization import (
    align_contact_ground,
    initialize_contact_slices,
    seed_contact_landmarks,
)
from mujoco_io import KinematicRecording


class ContactSamplingTests(unittest.TestCase):
    def test_supported_selection_preserves_foothold_breaks(self) -> None:
        source = self.recording()
        robot = gtd.CreateRobotFromFile(
            str(
                Path(__file__).resolve().parents[2]
                / "models/urdfs/test/simple_urdf_prismatic.urdf"
            )
        )
        source = replace(source, robot=robot, root=robot.link("l1"))
        left = gtd.PoseOnLink(robot.link("l1"), gtsam.Pose3())
        right = gtd.PoseOnLink(robot.link("l2"), gtsam.Pose3())
        double = gtd.PoseContactConstraintSpec([left, right])
        phases = [
            gtd.Phase(0, 2, double),
            gtd.Phase(3, 5, gtd.PoseContactConstraintSpec([left])),
            gtd.Phase(6, 6, double),
            gtd.Phase(7, 10, gtd.PoseContactConstraintSpec([right])),
            gtd.Phase(11, 30, double),
        ]
        measured = gtsam.Values()
        for k in range(31):
            for joint in robot.joints():
                measured.insert(gtd.JointAngleKey(joint.id(), k), 0.0)
        source = replace(
            source,
            measured=measured,
            dt=0.02,
            trajectory=gtd.Trajectory(gtd.WalkCycle(phases), 1),
        )
        options = gtd.pose_contact_init.Options()
        options.addGroundFactors = False
        diagnostics = gtd.pose_contact_init.Result()
        walk, seed, info = initialize_contact_slices(
            source,
            options,
            rotation_sigma_degrees=45.0,
            contact_diagnostics=diagnostics,
        )
        self.assertEqual(
            [slice.k for slice in diagnostics.slices()], [1, 4, 6, 8, 12, 21, 29]
        )
        self.assertTrue(
            all(len(snapshot.poses) == walk.count for snapshot in diagnostics.history)
        )
        self.assertAlmostEqual(options.rotationSigma, np.deg2rad(45.0))
        self.assertAlmostEqual(info["rotation_sigma_degrees"], 45.0)
        self.assertEqual(info["translation_sigma_m"], 0.01)
        np.testing.assert_array_equal(walk.source_indices, [1, 4, 6, 8, 12, 21, 29])
        np.testing.assert_allclose(
            walk.times, [0.02, 0.08, 0.12, 0.16, 0.24, 0.42, 0.58]
        )
        self.assertEqual(info["max_slice_interval_ms"], 200)
        self.assertTrue(info["supported_phases"])
        self.assertEqual(walk.contact_breaks, frozenset())
        groups = {}
        _, constraints, keys = build_problem(walk, objective_groups=groups)
        self.assertEqual(groups["contact"].size(), 12)
        self.assertEqual(constraints.size(), len(robot.joints()) * walk.count)
        self.assertEqual(
            seed_contact_landmarks(walk, seed).extract(keys).size(), len(keys)
        )

    def test_phase_sampling_is_even_and_respects_the_interval_limit(self) -> None:
        source = self.recording()
        left = gtd.PoseOnLink(source.root, gtsam.Pose3())
        right_link = gtd.Link(2, "right", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        right = gtd.PoseOnLink(right_link, gtsam.Pose3())
        double = gtd.PoseContactConstraintSpec([left, right])
        trajectory = gtd.Trajectory(
            gtd.WalkCycle(
                [
                    gtd.Phase(0, 30, double),
                    gtd.Phase(31, 49, gtd.PoseContactConstraintSpec([left])),
                    gtd.Phase(50, 51, double),
                ]
            ),
            1,
        )
        self.assertEqual(
            [s.k for s in initialization_slices(trajectory, 0.02)],
            [1, 10, 20, 29, 32, 40, 48, 50],
        )
        # 200 ms is not a multiple of 30 ms: seven source steps would be too long.
        retained = [s.k for s in initialization_slices(trajectory, 0.03)]
        gaps = np.diff([k for k in retained if k <= 30])
        self.assertLessEqual(float(gaps.max() * 0.03), 0.2)
        self.assertLessEqual(int(gaps.max() - gaps.min()), 1)
        self.assertEqual(
            [s.k for s in initialization_slices(trajectory, 0.02, 1000)],
            [1, 29, 32, 48, 50],
        )
        for invalid in (0, 10, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                initialization_slices(trajectory, 0.02, invalid)

    def recording(self, flight: bool = False) -> KinematicRecording:
        root = gtd.Link(1, "root", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"root": root}, {})
        contact = gtd.PoseContactConstraintSpec(
            [
                gtd.PoseOnLink(
                    root, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -1.0]))
                )
            ]
        )
        phases = [gtd.Phase(0, 30, contact)]
        if flight:
            phases = [
                gtd.Phase(0, 11, contact),
                gtd.Phase(12, 14, gtd.PoseContactConstraintSpec([])),
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

    def test_ground_frame_uses_fixed_gauge_and_fk_not_later_reference(self) -> None:
        source = self.recording()
        source = replace(
            source, anchor=gtsam.Pose3(gtsam.Rot3(), np.array([2.0, -3.0, 4.0]))
        )
        aligned, shift = align_contact_ground(source, anchor_slice=1)
        # The fixed sole is one metre below the root: its world height is 3 m.
        np.testing.assert_allclose(shift, [0.0, 0.0, -3.0])
        np.testing.assert_allclose(aligned.anchor.translation(), [2.0, -3.0, 1.0])
        np.testing.assert_allclose(
            aligned.reference_root_positions, source.reference_root_positions + shift
        )
        changed = replace(
            source, reference_root_positions=source.reference_root_positions * 1000
        )
        repeated, repeated_shift = align_contact_ground(changed, anchor_slice=1)
        np.testing.assert_array_equal(repeated_shift, shift)
        self.assertTrue(repeated.anchor.equals(aligned.anchor, 1e-12))
        self.assertTrue(aligned.measured.equals(source.measured, 1e-12))
        np.testing.assert_allclose(source.anchor.translation(), [2.0, -3.0, 4.0])

    def test_retained_slices_have_native_poses_and_source_times(self) -> None:
        source = self.recording()
        walk, seed, info = initialize_contact_slices(source)
        np.testing.assert_array_equal(walk.source_indices, np.arange(1, 30, 2))
        np.testing.assert_allclose(walk.times, np.arange(1, 30, 2) * 0.1)
        np.testing.assert_array_equal(
            walk.reference_root_positions,
            source.reference_root_positions[1:30:2],
        )
        self.assertEqual(info["contact_buffer"], 1)
        self.assertEqual(
            [k for k, _, _ in contact_slices(walk.trajectory)], list(range(walk.count))
        )
        costs, constraints, keys = build_problem(walk)
        for k in range(walk.count):
            self.assertTrue(seed.atPose3(gtd.PoseKey(1, k)).equals(source.anchor, 1e-8))
        self.assertLess(
            costs.error(seed_contact_landmarks(walk, seed).extract(keys)), 1e-10
        )
        self.assertEqual(constraints.size(), 0)

    def test_recontact_gets_new_landmark_across_skipped_flight(self) -> None:
        options = gtd.pose_contact_init.Options()
        options.addBetweenFactors = True
        walk, seed, _ = initialize_contact_slices(self.recording(flight=True), options)
        self.assertEqual(len(walk.contact_breaks), 1)
        k, link = next(iter(walk.contact_breaks))
        groups = {}
        build_problem(walk, objective_groups=groups)
        observations = list(stance_observations(walk))
        before = {key for i, _, key in observations if i < k}
        after = {key for i, _, key in observations if i >= k}
        self.assertFalse(before & after)
        self.assertEqual(
            list(contact_slices(walk.trajectory, walk.contact_breaks))[k][2], []
        )

    def test_contact_residual_matches_initializer_with_consistent_frames(self) -> None:
        # A root-FK observation and an optimized-foot observation represent
        # exactly the same predicted world sole pose, including COM offsets.
        root = gtsam.Pose3(
            gtsam.Rot3.RzRyRx(0.2, -0.1, 0.5), np.array([2.0, -1.0, 1.0])
        )
        root_to_foot = gtsam.Pose3(
            gtsam.Rot3.RzRyRx(-0.15, 0.08, -0.2), np.array([0.1, 0.2, -0.8])
        )
        foot_to_sole = gtsam.Pose3(
            gtsam.Rot3.RzRyRx(0.05, 0.02, -0.04), np.array([0.03, 0.0, -0.07])
        )
        landmark = (
            root
            * root_to_foot
            * foot_to_sole
            * gtsam.Pose3.Expmap(np.array([0.1, -0.2, 0.15, 0.01, -0.03, 0.02]))
        )
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([np.deg2rad(90.0)] * 3 + [0.01] * 3)
        )
        values = gtsam.Values()
        for key, value in [(1, root), (2, root * root_to_foot), (3, landmark)]:
            values.insert(key, value)
        initialization = gtd.ContactPoseFactor(1, 3, noise, root_to_foot * foot_to_sole)
        smoothing = gtd.ContactPoseFactor(2, 3, noise, foot_to_sole)
        self.assertAlmostEqual(
            initialization.error(values), smoothing.error(values), places=12
        )

    def test_shared_stance_pose_factor_and_optional_priors(self) -> None:
        # Independent foot observations connect to a shared optimized landmark.
        # CM-Opt must retain these free Pose3 keys outside kinematic constraints.
        walk, seed, _ = initialize_contact_slices(self.recording())
        seed = seed_contact_landmarks(walk, seed)
        groups = {}
        costs, constraints, keys = build_problem(
            walk, objective_groups=groups, upright_prior=True, height_prior=True
        )
        observations = list(stance_observations(walk))
        self.assertEqual(len({key for _, _, key in observations}), 1)
        self.assertEqual(groups["contact"].size(), walk.count)
        self.assertEqual(groups["height"].size(), 1)
        self.assertEqual(groups["upright"].size(), walk.count - 1)
        self.assertNotIn("root", groups)
        seed.update(
            gtd.PoseKey(1, 2),
            gtsam.Pose3(
                gtsam.Rot3.RzRyRx(0.1, -0.08, 0.12), np.array([0.1, 0.04, 1.05])
            ),
        )
        initial = seed.extract(keys)
        solution, _ = solve_cm(costs, constraints, initial, 1, 30)
        self.assertLess(costs.error(solution), costs.error(initial) * 1e-6)
        self.assertTrue(solution.exists(observations[0][2]))
        for k in range(walk.count):
            self.assertTrue(
                solution.atPose3(gtd.PoseKey(1, k)).equals(walk.anchor, 1e-5)
            )
        bare = {}
        build_problem(walk, objective_groups=bare)
        self.assertEqual(set(bare), {"joint", "contact", "anchor"})
        # The height prior leaves all orientation and horizontal position free.
        height = groups["height"]
        value = gtsam.Values()
        value.insert(
            observations[0][2],
            gtsam.Pose3(gtsam.Rot3.RzRyRx(0.6, -0.4, 0.8), np.array([12.0, -4.0, 0.2])),
        )
        self.assertAlmostEqual(height.error(value), 0.5 * (0.2 / 0.1) ** 2)

    def test_irregular_velocity_seed_and_pim_durations(self) -> None:
        walk, seed, _ = initialize_contact_slices(
            self.recording(), max_slice_interval_ms=350
        )
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
            [pim.deltaTij() for pim in pims],
            [0.3, 0.3, 0.2, 0.3, 0.3, 0.3, 0.3, 0.2, 0.3, 0.3],
            atol=1e-12,
        )
        factors = gtd.Kinematics().imuFactors(walk.trajectory, 1, pims, BIAS_KEY)
        self.assertEqual(factors.size(), walk.count - 1)
        self.assertLess(factors.error(initial), 1e-15)


if __name__ == "__main__":
    unittest.main()
