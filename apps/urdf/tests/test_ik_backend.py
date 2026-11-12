"""
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_ik_backend.py
 * @brief Tests for the URDF IK app's kinematics backend.
"""

import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import gtsam
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import ik_backend as ik  # noqa: E402

ROOT = ik.DEFAULT_URDF_ROOT


def urdf_link_frames(model, q):
    """Reference FK computed directly from the URDF XML."""
    _, joints = ik.parse_urdf(ROOT / model.rel_path)
    by_child = {j.child: j for j in joints.values()}
    frames = {}

    def frame(link):
        if link not in frames:
            joint = by_child.get(link)
            if joint is None:
                frames[link] = gtsam.Pose3()
            else:
                v = q.get(joint.name, 0.0)
                if joint.type in ("revolute", "continuous"):
                    motion = gtsam.Pose3(
                        gtsam.Rot3.Expmap(joint.axis * v), gtsam.Point3(0, 0, 0)
                    )
                elif joint.type == "prismatic":
                    motion = gtsam.Pose3(gtsam.Rot3(), joint.axis * v)
                else:
                    motion = gtsam.Pose3()
                frames[link] = frame(joint.parent).compose(joint.origin).compose(motion)
        return frames[link]

    return {name: frame(name) for name in model.links}


class TestRobotModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ur10e = ik.RobotModel(ROOT, "ur10e/ur10e.urdf")
        cls.panda = ik.RobotModel(ROOT, "panda/panda.urdf")
        cls.a1 = ik.RobotModel(ROOT, "a1/a1.urdf")

    def test_link_frames_match_urdf(self):
        rng = np.random.default_rng(0)
        for model in (self.ur10e, self.panda, self.a1):
            q = {name: rng.uniform(-1, 1) for name in model.joints}
            expected = urdf_link_frames(model, q)
            for name, pose in model.link_frames(q).items():
                self.assertTrue(pose.equals(expected[name], 1e-9), name)

    def test_tool_frame_is_default_end_effector(self):
        self.assertEqual(self.ur10e.default_ee, "tool0")
        self.assertEqual(self.ur10e.end_effectors["tool0"].link, "wrist_3_link")
        self.assertEqual(len(self.ur10e.movable_chain("tool0")), 6)

    def test_jacobian_matches_finite_differences(self):
        for orientation in (True, False):
            req = ik.IkRequest(
                "tool0",
                gtsam.Pose3(),
                {name: 0.3 for name in self.ur10e.joints},
                orientation,
            )
            p = ik.IkProblem(self.ur10e, req)
            frames, T = p.fk(p.x0)
            J = p.jacobian(frames, T)
            h = 1e-6
            for i in range(len(p.x0)):
                dx = np.zeros_like(p.x0)
                dx[i] = h
                _, T_plus = p.fk(p.x0 + dx)
                # World-frame angular and linear velocity of the EE frame.
                omega = (
                    gtsam.Rot3.Logmap(T_plus.rotation().compose(T.rotation().inverse()))
                    / h
                )
                v = (T_plus.translation() - T.translation()) / h
                column = np.concatenate([p.w_rot * omega, v]) if orientation else v
                np.testing.assert_allclose(column, J[:, i], atol=1e-5)

    def reachable_target(self, model, ee, seed):
        rng = np.random.default_rng(seed)
        q_goal = {name: rng.uniform(-0.6, 0.6) for name in model.joints}
        frames = model.link_frames(q_goal)
        q0 = {name: v + rng.uniform(-0.3, 0.3) for name, v in q_goal.items()}
        return model.ee_pose(frames, ee), q0

    def test_all_schemes_solve_reachable_pose(self):
        target, q0 = self.reachable_target(self.ur10e, "tool0", 1)
        # CCD and Jacobian transpose converge too slowly on orientation.
        for scheme in ("gtd", "dls", "pinv"):
            req = ik.IkRequest("tool0", target, q0)
            result = ik.solve(self.ur10e, scheme, req)
            self.assertEqual(result["status"], "solved", (scheme, result["reasons"]))

    def test_all_schemes_solve_reachable_position(self):
        target, q0 = self.reachable_target(self.panda, self.panda.default_ee, 2)
        for scheme in self.panda.scheme_ids(self.panda.default_ee, False):
            req = ik.IkRequest(self.panda.default_ee, target, q0, False)
            result = ik.solve(self.panda, scheme, req)
            self.assertEqual(result["status"], "solved", (scheme, result["reasons"]))

    def test_out_of_reach_is_explained(self):
        target = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(5.0, 0.0, 1.0))
        q0 = self.ur10e.default_configuration()
        for scheme in self.ur10e.scheme_ids("tool0", False):
            result = ik.solve(
                self.ur10e, scheme, ik.IkRequest("tool0", target, q0, False)
            )
            self.assertEqual(result["status"], "failed")
            self.assertIn("out of reach", result["reasons"][0], scheme)

    def test_too_few_joints_for_pose_is_explained(self):
        # An A1 leg has 3 joints, so an arbitrary 6-DoF foot pose fails.
        ee = "FR_lower"
        frames = self.a1.link_frames(self.a1.default_configuration())
        target = self.a1.ee_pose(frames, ee).compose(
            gtsam.Pose3(gtsam.Rot3.Rz(0.8), gtsam.Point3(0.02, 0.0, 0.0))
        )
        result = ik.solve(
            self.a1, "dls", ik.IkRequest(ee, target, self.a1.default_configuration())
        )
        self.assertEqual(result["status"], "failed")
        self.assertTrue(
            any("only 3 movable joints" in r for r in result["reasons"]),
            result["reasons"],
        )

    def test_joint_limits_are_explained(self):
        # Lock the panda's elbow (joint4) with tight limits, then ask for a
        # target that needs the elbow to bend.
        model = ik.RobotModel(ROOT, "panda/panda.urdf")
        elbow = model.joints["joint4"]
        q = model.default_configuration()
        q["joint4"] = -1.5
        elbow.lower, elbow.upper = -1.55, -1.45
        frames = model.link_frames(q)
        target = model.ee_pose(frames, model.default_ee).compose(
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 0.0, 0.3))
        )
        result = ik.solve(
            model, "dls", ik.IkRequest(model.default_ee, target, q, False)
        )
        self.assertEqual(result["status"], "failed")
        self.assertIn("joint4", result["at_limits"])
        self.assertTrue(
            any("Blocked by joint limits" in r for r in result["reasons"]),
            result["reasons"],
        )

    def test_rigidly_attached_link_is_explained(self):
        req = ik.IkRequest(
            self.ur10e.base,
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1, 1, 1)),
            self.ur10e.default_configuration(),
        )
        result = ik.solve(self.ur10e, "dls", req)
        self.assertEqual(result["status"], "failed")
        self.assertIn("rigidly attached", result["reasons"][0])

    def test_pose_json_round_trip(self):
        pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.1, -0.2, 0.3), gtsam.Point3(1, 2, 3))
        self.assertTrue(ik.pose_from_json(ik.pose_to_json(pose)).equals(pose, 1e-12))


@unittest.skipUnless(hasattr(ik.gtd, "PandaIKFast"), "Panda bindings not built")
class TestPandaIKFast(unittest.TestCase):
    def setUp(self):
        self.model = ik.RobotModel(ROOT, "panda/panda.urdf")
        self.angles = np.array([0.2, -0.4, 0.3, -1.6, 0.5, 1.2, -0.2])

    def configuration(self, angles):
        q = self.model.default_configuration()
        q.update(zip(ik.PANDA_JOINTS, angles))
        return q

    def test_binding_frames_and_keyword_arguments(self):
        q = self.configuration(self.angles)
        reference = urdf_link_frames(self.model, q)["link7"]
        pose = ik.gtd.PandaIKFast.forward(jointValues=self.angles)
        self.assertTrue(pose.equals(reference, 1e-8))
        solutions = ik.gtd.PandaIKFast.inverse(bTe=pose, theta7=self.angles[-1])
        self.assertGreater(len(solutions), 0)
        for solution in solutions:
            self.assertTrue(ik.gtd.PandaIKFast.forward(solution).equals(pose, 1e-8))

    def test_detection_and_supported_frames(self):
        self.assertTrue(self.model.panda_ikfast)
        for ee in ("link7", "link8", "hand"):
            self.assertIn("panda_ikfast", self.model.scheme_ids(ee))
            self.assertNotIn("panda_ikfast", self.model.scheme_ids(ee, False))
        self.assertNotIn("panda_ikfast", self.model.scheme_ids("link6"))
        ur10e = ik.RobotModel(ROOT, "ur10e/ur10e.urdf")
        self.assertNotIn("panda_ikfast", ur10e.scheme_ids("tool0"))
        with patch.dict(ik.gtd.__dict__):
            del ik.gtd.PandaIKFast
            self.assertFalse(self.model._detect_panda_ikfast())

    def test_detection_checks_geometry_not_filename(self):
        xml = (ROOT / "panda/panda.urdf").read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "renamed.urdf"
            path.write_text(xml)
            self.assertTrue(ik.RobotModel(Path(directory), path.name).panda_ikfast)
            path.write_text(xml.replace('xyz="0 0 0.333"', 'xyz="0 0 0.343"'))
            self.assertFalse(ik.RobotModel(Path(directory), path.name).panda_ikfast)

    def test_tool_offsets_limits_and_nearest_branch(self):
        for joint6 in (1.2, 3.4):
            goal = self.angles.copy()
            goal[5] = joint6
            q_goal = self.configuration(goal)
            seed = goal + np.array([0.02] * 6 + [0.0])
            for ee in ("link7", "link8", "hand"):
                target = self.model.ee_pose(self.model.link_frames(q_goal), ee)
                result = ik.solve(
                    self.model,
                    "panda_ikfast",
                    ik.IkRequest(
                        ee,
                        target,
                        self.configuration(seed),
                        max_iters=1,
                    ),
                )
                self.assertEqual(result["status"], "solved", result["reasons"])
                self.assertLess(result["pos_err"], 1e-8)
                self.assertLess(result["rot_err_deg"], 1e-5)
                for name, joint in self.model.joints.items():
                    self.assertGreaterEqual(result["q"][name], joint.lower)
                    self.assertLessEqual(result["q"][name], joint.upper)
                np.testing.assert_allclose(
                    [result["q"][name] for name in ik.PANDA_JOINTS],
                    goal,
                    atol=1e-7,
                )

    def test_sampling_and_no_solution_preserve_limits(self):
        goal = self.angles.copy()
        goal[-1] = self.model.joints["joint7"].lower
        q_goal = self.configuration(goal)
        target = self.model.ee_pose(self.model.link_frames(q_goal), "hand")
        for name, value in zip(ik.PANDA_JOINTS[:-1], goal[:-1]):
            self.model.joints[name].lower = value - 0.001
            self.model.joints[name].upper = value + 0.001
        seed = dict(q_goal, joint7=0.1)
        limited = ik.solve(
            self.model,
            "panda_ikfast",
            ik.IkRequest(
                "hand",
                target,
                seed,
                max_iters=1,
            ),
        )
        self.assertEqual(limited["status"], "failed")
        self.assertEqual(limited["q"], seed)
        self.assertIn("does not prove", limited["reasons"][0])
        sampled = ik.solve(
            self.model,
            "panda_ikfast",
            ik.IkRequest(
                "hand",
                target,
                seed,
                max_iters=33,
            ),
        )
        self.assertEqual(sampled["status"], "solved", sampled["reasons"])

    def test_rejects_unsupported_requests(self):
        q = self.configuration(self.angles)
        for ee, orientation in (("link6", True), ("hand", False)):
            with self.assertRaisesRegex(ik.ModelError, "requires a compatible Panda"):
                ik.solve(
                    self.model,
                    "panda_ikfast",
                    ik.IkRequest(
                        ee,
                        gtsam.Pose3(),
                        q,
                        orientation,
                    ),
                )


class TestLoadability(unittest.TestCase):

    def test_bad_urdf_reports_error(self):
        error = ik.check_loadable(ROOT / "test" / "four_bar_linkage.urdf")
        self.assertIsNotNone(error)

    def test_good_urdf_loads(self):
        self.assertIsNone(ik.check_loadable(ROOT / "ur5" / "ur5.urdf"))


if __name__ == "__main__":
    unittest.main()
