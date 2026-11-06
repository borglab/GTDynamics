# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file test_kinodynamics.py
# @brief Verify native initialization, frame conventions and coupled mechanics.
# @author GTDynamics contributors
"""Dataset-independent tests for the private G1 native helper."""

import unittest
import gtdynamics as gtd
import gtsam
import numpy as np
from _g1_kinodynamics import solve
from kinodynamic_settings import solver_settings


class TestKinodynamics(unittest.TestCase):
    def test_sparse_support_preparation_preserves_times_and_stances(self) -> None:
        """Select before assembly; keep swing knots and continuous foothold IDs."""
        import gzip
        import json
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from types import SimpleNamespace
        from unittest.mock import patch

        from contact_phases import stance_observations
        from kinodynamic_preparation import prepare_walk

        body = gtd.Link(0, "pelvis", 10.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        foot = gtd.Link(1, "foot", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        joint = gtd.RevoluteJoint(
            0, "joint", gtsam.Pose3(), body, foot, np.array([0.0, 0.0, 1.0])
        )
        body.addJoint(joint)
        foot.addJoint(joint)
        robot = gtd.Robot({"pelvis": body, "foot": foot}, {"joint": joint})
        frames = {
            "left": gtd.PoseOnLink(body, gtsam.Pose3(), True),
            "right": gtd.PoseOnLink(foot, gtsam.Pose3(), True),
        }
        model = SimpleNamespace(
            body_ipos=np.zeros((2, 3)),
            body_iquat=np.array([[1.0, 0.0, 0.0, 0.0]] * 2),
            qpos0=np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            jnt_qposadr=[7],
            joint=lambda name: SimpleNamespace(id=0),
        )
        qpos = np.zeros((131, 8))
        qpos[:, 2:4] = 1
        qpos[:, 7] = np.arange(131) * 0.001
        states = [2] * 3 + [0] * 3 + [2] + [1] * 4 + [2] * 20
        selected = np.array([1, 4, 6, 8, 12, 21, 29])
        with TemporaryDirectory() as directory:
            labels = Path(directory) / "labels.json.gz"
            labels.write_bytes(
                gzip.compress(
                    json.dumps(
                        {
                            "times": (2 + np.arange(31) * 0.02).tolist(),
                            "supportGraph": {"states": states},
                        }
                    ).encode()
                )
            )
            with patch.multiple(
                "kinodynamic_preparation",
                load_recording=lambda _: ({"qpos": qpos}, {}),
                load_g1_model=lambda: model,
                model_bodies=lambda _: [],
                import_robot=lambda _: robot,
                contact_frames=lambda *_: frames,
            ):
                walk, metadata = prepare_walk(Path(directory), labels, 2.0, 2.6)
        self.assertEqual(walk.count, 7)
        np.testing.assert_array_equal(walk.source_indices, 100 + selected)
        np.testing.assert_allclose(walk.times, 2 + selected * 0.02)
        np.testing.assert_allclose(
            [
                walk.measured.atDouble(gtd.JointAngleKey(0, k))
                for k in range(walk.count)
            ],
            (100 + selected) * 0.001,
        )
        self.assertEqual(metadata["initializationSlices"], list(range(7)))
        self.assertEqual(metadata["optimizationSourceSlices"], selected.tolist())
        self.assertEqual(metadata["optimizationSampling"]["singleSupportSlices"], 2)
        self.assertEqual(metadata["optimizationSampling"]["sourceSlices"], 31)
        observations = {
            (k, contact.link.id()): key for k, contact, key in stance_observations(walk)
        }
        # Left stays down across the first double/single/double transition.
        self.assertEqual(observations[0, 0], observations[2, 0])
        # The right foot lifts during the first single-support phase.
        self.assertNotEqual(observations[0, 1], observations[2, 1])
        self.assertEqual(observations[2, 1], observations[4, 1])

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
        self.assertEqual(options["initialization"], "contact-gp-dynamics")
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

    def test_gravity_support_seed_frames_and_schedule(self) -> None:
        """World support sums to weight; swing feet stay zero and COM moments agree."""
        body = gtd.Link(0, "body", 8.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        links, joints, soles = {"body": body}, {}, []
        for c, (x, yaw) in enumerate([(-0.05, 0.3), (0.05, -0.4)]):
            home = gtsam.Pose3(gtsam.Rot3(), np.array([x, 0.0, -0.4]))
            foot = gtd.Link(c + 1, f"foot{c}", 1.0, np.eye(3), home, home)
            joint = gtd.RevoluteJoint(
                c, f"joint{c}", home, body, foot, np.array([0.0, 0.0, 1.0])
            )
            body.addJoint(joint)
            foot.addJoint(joint)
            links[foot.name()] = foot
            joints[joint.name()] = joint
            soles.append(
                gtd.PoseOnLink(
                    foot,
                    gtsam.Pose3(gtsam.Rot3.Rz(yaw), np.array([0.02, 0.0, -0.1])),
                    True,
                )
            )
        robot = gtd.Robot(links, joints)
        supports = [[soles[0]], soles, [soles[1]]]
        trajectory = gtd.Trajectory(
            gtd.WalkCycle(
                [
                    gtd.Phase(k, k, gtd.PoseContactConstraintSpec(contacts))
                    for k, contacts in enumerate(supports)
                ]
            ),
            1,
        )
        root_pose = gtsam.Pose3(gtsam.Rot3.Ry(0.1), np.array([0.0, 0.0, 0.5]))
        initial = gtsam.Values()
        for k in range(3):
            known = gtsam.Values()
            known.insert(gtd.PoseKey(0, k), root_pose)
            for joint in robot.joints():
                known.insert(gtd.JointAngleKey(joint.id(), k), 0.0)
                known.insert(gtd.JointVelKey(joint.id(), k), 0.0)
            initial.insert(robot.forwardKinematics(known, k, "body"))
        options = solver_settings()
        options["initialization"] = "fk"
        options["maxIterations"] = 1
        options["retractionThreads"] = 1
        result = solve(
            robot,
            trajectory,
            [gtd.Slice(k) for k in range(3)],
            [0.0, 0.02, 0.04],
            initial,
            [],
            [],
            [gtd.fixed_dynamics.Contact(s, 0.1, 0.04, 0.6) for s in soles],
            np.empty(0),
            options,
        )
        seed = result["initialValues"]
        for k, contacts in enumerate(supports):
            total = np.zeros(3)
            for c, sole in enumerate(soles):
                wrench = seed.atVector(result["contactKeys"][2 * k + c])
                np.testing.assert_array_equal(wrench[:3], np.zeros(3))
                force = (
                    (seed.atPose3(gtd.PoseKey(sole.link.id(), k)) * sole.comTcontact)
                    .rotation()
                    .rotate(wrench[3:])
                )
                expected = (
                    np.array([0.0, 0.0, 98.1 / len(contacts)])
                    if sole in contacts
                    else np.zeros(3)
                )
                np.testing.assert_allclose(force, expected, atol=1e-12)
                total += force
                com_key = gtd.DynamicsSymbol.LinkJointSymbol(
                    "ec", sole.link.id(), c, k
                ).key()
                transform = sole.comTcontact.inverse().AdjointMap().T
                np.testing.assert_allclose(
                    seed.atVector(com_key), transform @ wrench, atol=1e-12
                )
            np.testing.assert_allclose(total, [0.0, 0.0, 98.1], atol=1e-12)
            np.testing.assert_array_equal(gtd.Twist(seed, 0, k), np.zeros(6))
            np.testing.assert_array_equal(gtd.TwistAccel(seed, 0, k), np.zeros(6))

    def test_contact_mode_keeps_the_original_zero_motion_seed(self) -> None:
        """Explicit contact mode retains the zero-motion alternative."""
        from unittest.mock import patch
        from kinodynamic_initialization import initialize_walk
        from kinodynamic_walk import argument_parser
        from mujoco_io import KinematicRecording

        body = gtd.Link(0, "body", 10.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"body": body}, {})
        sole = gtd.PoseOnLink(
            body, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -0.5])), True
        )
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 2, gtd.PoseContactConstraintSpec([sole]))]), 1
        )
        anchor = gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.5]))
        walk = KinematicRecording(
            robot,
            body,
            gtsam.Values(),
            anchor,
            trajectory,
            3,
            0.02,
            0.0,
            np.zeros((3, 3)),
            {},
        )
        self.assertEqual(
            argument_parser().parse_args([]).initialization, "contact-gp-dynamics"
        )
        with patch.object(
            gtd,
            "KinodynamicInitializer",
            side_effect=AssertionError(
                "The contact-only mode must skip the full pipeline"
            ),
        ):
            values, metadata = initialize_walk(walk, [0, 2], "contact", [], np.empty(0))
        self.assertNotIn("dynamicsInitializationSeconds", metadata)
        for k in range(3):
            self.assertTrue(values.atPose3(gtd.PoseKey(0, k)).equals(anchor, 1e-10))
            np.testing.assert_array_equal(gtd.Twist(values, 0, k), np.zeros(6))
            self.assertFalse(values.exists(gtd.TwistAccelKey(0, k)))
            self.assertFalse(values.exists(gtd.ContactWrenchKey(0, 0, k)))

    def test_default_initializer_stops_after_the_linear_dynamics_seed(self) -> None:
        """The wrapped default returns feasible dynamics without CMC iterations."""
        from kinodynamic_initialization import initialize_walk
        from mujoco_io import KinematicRecording

        body = gtd.Link(0, "body", 10.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        child = gtd.Link(1, "child", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        joint = gtd.RevoluteJoint(
            0, "joint", gtsam.Pose3(), body, child, np.array([0.0, 0.0, 1.0])
        )
        body.addJoint(joint)
        child.addJoint(joint)
        robot = gtd.Robot({"body": body, "child": child}, {"joint": joint})
        measured = gtsam.Values()
        for k in range(3):
            gtd.InsertJointAngle(measured, 0, k, 0.0)
        sole = gtd.PoseOnLink(
            body, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -0.5])), True
        )
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 2, gtd.PoseContactConstraintSpec([sole]))]), 1
        )
        walk = KinematicRecording(
            robot,
            body,
            measured,
            gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, 0.5])),
            trajectory,
            3,
            0.02,
            0.0,
            np.zeros((3, 3)),
            {},
        )
        values, diagnostics = initialize_walk(
            walk,
            [0, 2],
            "contact-gp-dynamics",
            [gtd.fixed_dynamics.Contact(sole, 0.1, 0.04, 0.8)],
            np.empty(0),
        )
        self.assertTrue(diagnostics["dynamicsInitializationFeasible"])
        self.assertEqual(diagnostics["dynamicsInitializationIterations"], 0)
        self.assertEqual(diagnostics["dynamicsInitializationManifoldSeconds"], 0)
        self.assertEqual(diagnostics["dynamicsInitializationOptimizationSeconds"], 0)
        np.testing.assert_allclose(
            values.atVector(gtd.ContactWrenchKey(0, 0, 1)),
            [0, 0, 0, 0, 0, 107.91],
            atol=1e-8,
        )

    def test_complete_native_seed_is_retained(self) -> None:
        """Coupled assembly preserves fitted motion and solved support dynamics."""
        body = gtd.Link(0, "body", 10.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        robot = gtd.Robot({"body": body}, {})
        sole = gtd.PoseOnLink(
            body, gtsam.Pose3(gtsam.Rot3(), np.array([0.0, 0.0, -0.5])), True
        )
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 2, gtd.PoseContactConstraintSpec([sole]))]), 1
        )
        initial = gtsam.Values()
        wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 98.1])
        twist = np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0])
        for k in range(3):
            initial.insert(
                gtd.PoseKey(0, k),
                gtsam.Pose3(gtsam.Rot3(), np.array([0.006 * k, 0.0, 0.5])),
            )
            gtd.InsertTwist(initial, 0, k, twist)
            gtd.InsertTwistAccel(initial, 0, k, np.zeros(6))
            initial.insert(gtd.LinearVelocityKey(0, k), twist[3:])
            initial.insert(gtd.ContactWrenchKey(0, 0, k), wrench)
            initial.insert(
                gtd.DynamicsSymbol.LinkJointSymbol("ec", 0, 0, k).key(), wrench
            )
        options = solver_settings()
        options["initialization"] = "contact-gp-dynamics"
        options["maxIterations"] = 1
        result = solve(
            robot,
            trajectory,
            [gtd.Slice(k) for k in range(3)],
            [0.0, 0.02, 0.04],
            initial,
            [],
            [],
            [gtd.fixed_dynamics.Contact(sole, 0.1, 0.04, 0.8)],
            np.empty(0),
            options,
        )
        for k in range(3):
            np.testing.assert_array_equal(
                gtd.Twist(result["initialValues"], 0, k), twist
            )
            np.testing.assert_array_equal(
                result["initialValues"].atVector(gtd.ContactWrenchKey(0, 0, k)), wrench
            )
            np.testing.assert_array_equal(
                result["initialValues"].atVector(gtd.LinearVelocityKey(0, k)), twist[3:]
            )
        self.assertTrue(result["feasible"])

    def test_fk_seed_keeps_root_fixed_and_uses_measured_joints(self) -> None:
        """FK retains the measured joint motion without fitting a root trajectory."""
        from kinodynamic_initialization import initialize_fk
        from mujoco_io import KinematicRecording

        root = gtd.Link(0, "root", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        child = gtd.Link(1, "child", 1.0, np.eye(3), gtsam.Pose3(), gtsam.Pose3())
        joint = gtd.RevoluteJoint(
            0, "hinge", gtsam.Pose3(), root, child, np.array([0.0, 0.0, 1.0])
        )
        root.addJoint(joint)
        child.addJoint(joint)
        robot = gtd.Robot({"root": root, "child": child}, {"hinge": joint})
        measured = gtsam.Values()
        for k, angle in enumerate([0.1, 0.4, -0.2]):
            measured.insert(gtd.JointAngleKey(0, k), angle)
        anchor = gtsam.Pose3(gtsam.Rot3.Rz(0.5), np.array([1.0, 2.0, 0.7]))
        trajectory = gtd.Trajectory(
            gtd.WalkCycle([gtd.Phase(0, 2, gtd.PoseContactConstraintSpec([]))]), 1
        )
        walk = KinematicRecording(
            robot,
            root,
            measured,
            anchor,
            trajectory,
            3,
            0.02,
            0.0,
            np.zeros((3, 3)),
            {},
        )
        seed = initialize_fk(walk)
        for k, angle in enumerate([0.1, 0.4, -0.2]):
            self.assertTrue(seed.atPose3(gtd.PoseKey(0, k)).equals(anchor, 1e-12))
            self.assertEqual(seed.atDouble(gtd.JointAngleKey(0, k)), angle)
            self.assertTrue(
                seed.atPose3(gtd.PoseKey(1, k))
                .rotation()
                .equals(anchor.rotation() * gtsam.Rot3.Rz(angle), 1e-12)
            )
            np.testing.assert_array_equal(gtd.Twist(seed, 1, k), np.zeros(6))


if __name__ == "__main__":
    unittest.main()
