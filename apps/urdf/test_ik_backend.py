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
import unittest

import gtsam
import numpy as np

import ik_backend as ik

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
                    motion = gtsam.Pose3(gtsam.Rot3.Expmap(joint.axis * v),
                                         gtsam.Point3(0, 0, 0))
                elif joint.type == "prismatic":
                    motion = gtsam.Pose3(gtsam.Rot3(), joint.axis * v)
                else:
                    motion = gtsam.Pose3()
                frames[link] = frame(joint.parent).compose(
                    joint.origin).compose(motion)
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
            req = ik.IkRequest("tool0", gtsam.Pose3(), {
                name: 0.3 for name in self.ur10e.joints
            }, orientation)
            p = ik.IkProblem(self.ur10e, req)
            frames, T = p.fk(p.x0)
            J = p.jacobian(frames, T)
            h = 1e-6
            for i in range(len(p.x0)):
                dx = np.zeros_like(p.x0)
                dx[i] = h
                _, T_plus = p.fk(p.x0 + dx)
                # World-frame angular and linear velocity of the EE frame.
                omega = gtsam.Rot3.Logmap(T_plus.rotation().compose(
                    T.rotation().inverse())) / h
                v = (T_plus.translation() - T.translation()) / h
                column = (np.concatenate([p.w_rot * omega, v])
                          if orientation else v)
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
            self.assertEqual(result["status"], "solved",
                             (scheme, result["reasons"]))

    def test_all_schemes_solve_reachable_position(self):
        target, q0 = self.reachable_target(self.panda, self.panda.default_ee, 2)
        for scheme in ik.SCHEMES:
            req = ik.IkRequest(self.panda.default_ee, target, q0, False)
            result = ik.solve(self.panda, scheme, req)
            self.assertEqual(result["status"], "solved",
                             (scheme, result["reasons"]))

    def test_out_of_reach_is_explained(self):
        target = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(5.0, 0.0, 1.0))
        q0 = self.ur10e.default_configuration()
        for scheme in ik.SCHEMES:
            result = ik.solve(self.ur10e, scheme,
                              ik.IkRequest("tool0", target, q0, False))
            self.assertEqual(result["status"], "failed")
            self.assertIn("out of reach", result["reasons"][0], scheme)

    def test_too_few_joints_for_pose_is_explained(self):
        # An A1 leg has 3 joints, so an arbitrary 6-DoF foot pose fails.
        ee = "FR_lower"
        frames = self.a1.link_frames(self.a1.default_configuration())
        target = self.a1.ee_pose(frames, ee).compose(
            gtsam.Pose3(gtsam.Rot3.Rz(0.8), gtsam.Point3(0.02, 0.0, 0.0)))
        result = ik.solve(
            self.a1, "dls",
            ik.IkRequest(ee, target, self.a1.default_configuration()))
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any("only 3 movable joints" in r
                            for r in result["reasons"]), result["reasons"])

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
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 0.0, 0.3)))
        result = ik.solve(model, "dls",
                          ik.IkRequest(model.default_ee, target, q, False))
        self.assertEqual(result["status"], "failed")
        self.assertIn("joint4", result["at_limits"])
        self.assertTrue(any("Blocked by joint limits" in r
                            for r in result["reasons"]), result["reasons"])

    def test_rigidly_attached_link_is_explained(self):
        req = ik.IkRequest(self.ur10e.base, gtsam.Pose3(gtsam.Rot3(),
                                                        gtsam.Point3(1, 1, 1)),
                           self.ur10e.default_configuration())
        result = ik.solve(self.ur10e, "dls", req)
        self.assertEqual(result["status"], "failed")
        self.assertIn("rigidly attached", result["reasons"][0])

    def test_pose_json_round_trip(self):
        pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.1, -0.2, 0.3),
                           gtsam.Point3(1, 2, 3))
        self.assertTrue(ik.pose_from_json(ik.pose_to_json(pose)).equals(
            pose, 1e-12))


class TestLoadability(unittest.TestCase):

    def test_bad_urdf_reports_error(self):
        error = ik.check_loadable(ROOT / "test" / "four_bar_linkage.urdf")
        self.assertIsNotNone(error)

    def test_good_urdf_loads(self):
        self.assertIsNone(ik.check_loadable(ROOT / "ur5" / "ur5.urdf"))


if __name__ == "__main__":
    unittest.main()
