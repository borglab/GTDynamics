# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_structure.py
# @brief Check the CMRG structure builder, RRT seeding, boxes and trajectory sampling.
# @author GTDynamics contributors

"""Tests for the pure-geometry parts of the STEAP assembly exporter."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import rrt  # noqa: E402
import scene as sc  # noqa: E402
import sequence as sq  # noqa: E402
import structure as st  # noqa: E402


class TestStructure(unittest.TestCase):
    """Members, staging and grips of each preset."""

    def test_members_are_2x4_and_ordered_posts_then_lintels(self):
        """Every member shares the 2x4 section and posts precede lintels."""
        for spec in st.PRESETS.values():
            structure = st.build_structure(spec)
            kinds = [m.kind for m in structure.members if not m.fixed]
            self.assertEqual(kinds, sorted(kinds, key=lambda k: k == "lintel"))
            for member in structure.members:
                self.assertAlmostEqual(member.size[1], 3.5 * 0.0254)
                self.assertAlmostEqual(member.size[2], 1.5 * 0.0254)

    def test_furthest_stock_is_two_metres_from_the_site(self):
        """The furthest staged member lies 2.0 m from the build site."""
        for spec in st.PRESETS.values():
            structure = st.build_structure(spec)
            distances = [
                np.linalg.norm(member.staging[:2, 3] - structure.site)
                for member in structure.members
                if not member.fixed
            ]
            self.assertAlmostEqual(max(distances), 2.0)

    def test_portal_members_are_flush(self):
        """Base ends meet post faces and lintel ends line up with the posts' outer faces."""
        structure = st.build_structure(st.PRESETS["portal"])
        members = {m.name: m for m in structure.members}
        ys = np.sort([members[n].target[1, 3] for n in ("post_1", "post_2")])
        thick = st.PROFILE_THICK
        self.assertAlmostEqual(
            members["base"].size[0], (ys[1] - thick / 2.0) - (ys[0] + thick / 2.0)
        )
        self.assertAlmostEqual(
            members["lintel_1"].size[0], (ys[1] + thick / 2.0) - (ys[0] - thick / 2.0)
        )
        self.assertAlmostEqual(
            members["base"].target[2, 3] - thick / 2.0, st.TABLE_TOP_Z
        )
        for name in ("post_1", "post_2"):
            self.assertAlmostEqual(
                members[name].target[2, 3] - members[name].size[0] / 2.0, st.TABLE_TOP_Z
            )

    def test_workbench_matches_structure_and_platform_holds_stock(self):
        """The workbench has the assembled footprint; the platform is separate and holds the stock."""
        structure = st.build_structure(st.PRESETS["portal"])
        members = {m.name: m for m in structure.members}
        bench, platform = structure.tables["workbench"], structure.tables["platform"]
        np.testing.assert_allclose(
            bench.half[:2], [st.PROFILE_WIDTH / 2.0, members["lintel_1"].size[0] / 2.0]
        )
        np.testing.assert_allclose(bench.center[:2], structure.site)
        self.assertLess(
            platform.center[0] + platform.half[0], bench.center[0] - bench.half[0]
        )
        for member in structure.members:
            if not member.fixed:
                corners = st.footprint(member.staging, member.size)
                self.assertTrue(
                    np.all(np.abs(corners - platform.center[:2]) <= platform.half[:2])
                )

    def test_upper_gripper_pad_is_on_the_un_nailed_end_of_each_post(self):
        """Both gripper pads hold each post: the upper ends at the top end, the lower stays on the wood."""
        import cell
        import urdf_display as ud

        vertices = ud.read_stl(cell.TOOL_MESHES["gripper"])[0].reshape(-1, 3)
        tool0 = np.linalg.inv(cell.tool_tcp_pose("gripper").matrix())
        for member in st.build_structure(st.PRESETS["portal"]).members:
            if member.kind != "post":
                continue
            world_tool0 = member.grip_staging.matrix() @ tool0
            to_post = np.linalg.inv(member.staging) @ world_tool0
            local = vertices @ to_post[:3, :3].T + to_post[:3, 3]
            pads = local[np.abs(local[:, 1]) < member.size[1] / 2.0]
            half = member.size[0] / 2.0
            self.assertAlmostEqual(float(pads[:, 0].max()), half, delta=0.01)
            self.assertGreater(float(pads[:, 0].min()), -half)
            # Top end in the build: the staged +x end stands highest at the target.
            self.assertGreater(member.target[2, 0], 0.0)

    def test_grip_offset_carries_the_member_into_place(self):
        """The in-hand offset maps the target grip back to the target member pose."""
        structure = st.build_structure(st.PRESETS["portal"])
        for member in structure.members:
            if member.fixed:
                continue
            placed = member.grip.matrix() @ member.grip_offset
            np.testing.assert_allclose(placed, member.target, atol=1e-9)

    def test_lintel_rests_on_post_tops(self):
        """Each lintel sits just above the tops of the posts it joins."""
        structure = st.build_structure(st.PRESETS["double-portal"])
        posts = {m.name: m for m in structure.members if m.kind == "post"}
        for member in structure.members:
            if member.kind != "lintel":
                continue
            top = st.TABLE_TOP_Z + max(p.size[0] for p in posts.values())
            self.assertAlmostEqual(member.target[2, 3] - member.size[2] / 2.0, top)


class TestSequence(unittest.TestCase):
    """Maneuver order, nails and RRT seeding."""

    def test_member_maneuvers_and_seeding(self):
        """Posts take four nails before release; the lintel takes eight and the build ends on the last."""
        structure = st.build_structure(st.PRESETS["portal"])
        segments = sq.build_sequence(structure)
        for member, count, clear in (
            ("post_1", 4, False),
            ("post_2", 4, True),
            ("lintel_1", 8, True),
        ):
            steps = [s.key.split(":")[1] for s in segments if s.member == member]
            nails = ["reposition", "nail", "withdraw"] * count
            expected = (
                (["clear"] if clear else [])
                + ["reach", "grip", "carry"]
                + nails
                + (["pull"] if member == "post_2" else [])
                + ["release"]
            )
            if member == "lintel_1":
                # Paper's beam: lift, hover, lower; the carry ends at the first standoff.
                expected = [
                    "clear",
                    "reach",
                    "grip",
                    "lift",
                    "carry_clear",
                    "carry",
                ] + nails[1:-1]
            self.assertEqual(steps, expected)
        self.assertEqual(segments[-1].key, f"lintel_1:nail:7")
        for segment in segments:
            self.assertEqual(segment.use_rrt, not segment.nailing, segment.key)

    def test_support_arm_holds_through_every_nail(self):
        """Every nailing step leaves arm 1's goal unset, and release comes after the last withdraw."""
        segments = sq.build_sequence(st.build_structure(st.PRESETS["portal"]))
        for segment in segments:
            if segment.key.split(":")[1] in ("reposition", "nail", "withdraw"):
                self.assertIsNone(segment.goals[0], segment.key)
        for member in ("post_1", "post_2"):
            steps = [s.key.split(":")[1] for s in segments if s.member == member]
            self.assertEqual(steps.index("release"), len(steps) - 1)
            self.assertEqual(steps.count("nail"), 4)


class TestShortcut(unittest.TestCase):
    """RRT path cleanup."""

    def test_shortcut_shortens_and_stays_free(self):
        """A zig-zag becomes shorter, keeps its ends and never crosses the wall."""

        def free(q):
            return not (0.4 < q[0] < 0.6 and q[1] < 0.8)

        path = [
            np.array(p, dtype=float)
            for p in (
                [0.1, 0.1],
                [0.2, 0.7],
                [0.1, 0.9],
                [0.5, 0.95],
                [0.9, 0.85],
                [0.8, 0.3],
                [0.9, 0.1],
            )
        ]
        span = np.ones(2)
        short = rrt.shortcut(path, free, span, 0.01, seed=1)
        length = lambda points: sum(
            np.linalg.norm(b - a) for a, b in zip(points, points[1:])
        )  # noqa: E731
        self.assertLess(length(short), length(path))
        np.testing.assert_allclose(short[0], path[0])
        np.testing.assert_allclose(short[-1], path[-1])
        for a, b in zip(short, short[1:]):
            self.assertTrue(rrt.edge_clear(a, b, free, span, 0.01))


class TestPlayback(unittest.TestCase):
    """Interpolation between executed states."""

    def test_playback_matches_the_support_states_and_velocities(self):
        """The GP mean passes through both states with zero-velocity ends staying between them."""
        import export_recording as ex

        q0, q1, zero = np.array([0.0, 1.0]), np.array([0.4, 1.0]), np.zeros(2)
        np.testing.assert_allclose(ex.hermite(q0, q1, zero, zero, 0.5, 0.0), q0)
        np.testing.assert_allclose(ex.hermite(q0, q1, zero, zero, 0.5, 1.0), q1)
        np.testing.assert_allclose(ex.hermite(q0, q1, zero, zero, 0.5, 0.5), [0.2, 1.0])


class TestSpheres(unittest.TestCase):
    """Sphere models of wood and tables."""

    def test_member_spheres_enclose_the_member(self):
        """Corners and mid-points of a 2x4 lie inside the chained spheres."""
        size = (1.2, 3.5 * 0.0254, 1.5 * 0.0254)
        pose = st.standing_pose(2.0, 3.0, 0.9, 0.7)
        group = sc.member_spheres(pose, size)
        signs = np.array(
            [[sx, sy, sz] for sx in (-1, 0, 1) for sy in (-1, 1) for sz in (-1, 1)],
            dtype=float,
        )
        corners = pose[:3, :3] @ (signs * np.array(size) / 2.0).T + pose[:3, 3:4]
        self.assertTrue(np.all(group.signed_distance(corners.T) < 0.0))

    def test_table_spheres_stay_inside_and_cover_the_box(self):
        """Spheres never cross a face, and cover even a table narrower than the sphere size."""
        for half in (
            [0.7, 1.2, st.TABLE_TOP_Z / 2.0],
            [st.PROFILE_WIDTH / 2.0, 1.16, st.TABLE_TOP_Z / 2.0],
        ):
            box = sc.Box.axis_aligned([0.0, 0.0, half[2]], half)
            group = sc.table_spheres(box)
            np.testing.assert_array_less(
                (np.abs(group.centers - box.center) + group.radius).max(axis=0),
                box.half + 1e-9,
            )
            top = np.array(
                [
                    [0.0, y, 2.0 * half[2]]
                    for y in np.linspace(-0.9 * half[1], 0.9 * half[1], 25)
                ]
            )
            self.assertLess(group.signed_distance(top).max(), 0.02)

    def test_table_top_is_nearly_flat_and_clears_the_floor(self):
        """The table's sphere union reaches its top surface and is clear just above it."""
        box = sc.Box.axis_aligned(
            [0.0, 0.0, st.TABLE_TOP_Z / 2.0], [0.7, 1.2, st.TABLE_TOP_Z / 2.0]
        )
        group = sc.table_spheres(box)
        above = np.array(
            [[0.0, 0.0, st.TABLE_TOP_Z + 0.02], [0.33, 0.51, st.TABLE_TOP_Z + 0.02]]
        )
        below = np.array(
            [[0.0, 0.0, st.TABLE_TOP_Z - 0.03], [0.33, 0.51, st.TABLE_TOP_Z - 0.03]]
        )
        self.assertTrue(np.all(group.signed_distance(above) > 0.0))
        self.assertTrue(np.all(group.signed_distance(below) < 0.0))


class TestRrt(unittest.TestCase):
    """RRT-Connect seeding."""

    def test_path_avoids_a_wall_and_resamples_with_zero_end_velocity(self):
        """The path detours around an obstacle; velocities vanish at both ends."""
        lower, upper = np.zeros(2), np.ones(2)

        def free(q):
            return not (0.4 < q[0] < 0.6 and q[1] < 0.8)

        path = rrt.rrt_connect(
            [0.1, 0.1],
            [0.9, 0.1],
            lower,
            upper,
            free,
            step=0.05,
            resolution=0.01,
            seed=1,
        )
        self.assertTrue(all(free(q) for q in path))
        line, velocities = rrt.path_to_states(path, 10, 2.0, upper - lower)
        np.testing.assert_allclose(line[0], [0.1, 0.1])
        np.testing.assert_allclose(line[-1], [0.9, 0.1])
        np.testing.assert_allclose(velocities[0], 0.0)
        np.testing.assert_allclose(velocities[-1], 0.0)


if __name__ == "__main__":
    unittest.main()
