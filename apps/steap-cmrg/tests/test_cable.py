# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_cable.py
# @brief Check the drawn and constrained CMRG cables against the Grasshopper anchors.
# @author GTDynamics contributors

"""Tests that both cable models end on the anchors, with straight exit guides."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import cable_display as cd  # noqa: E402
import cell  # noqa: E402
import planner as pl  # noqa: E402
import urdf_display as ud  # noqa: E402


class TestCable(unittest.TestCase):
    """Drawn and constrained cables share their anchors on both arms."""

    @classmethod
    def setUpClass(cls):
        cls.robot = cell.load_robot()
        cls.names = [n for arm in (1, 2) for n in cell.arm_joint_names(arm)]
        cls.cables = {
            arm: pl.build_cable(cls.robot, arm, cls.names, 9 * (arm - 1))
            for arm in (1, 2)
        }
        cls.display = cd.CableDisplay()
        cls.model = ud.parse_urdf(cell.URDF_FILE)
        cls.q = np.concatenate([cell.park_configuration(1), cell.park_configuration(2)])
        cls.q[3:9] = [0.3, -0.5, 0.4, 0.2, 0.6, -0.3]
        cls.q[12:18] = [-0.3, -0.2, 0.5, 0.1, 0.4, 0.2]
        cls.world = ud.link_transforms(cls.model, dict(zip(cls.names, cls.q)))

    def anchors(self, arm):
        """Return the world anchor positions of an arm's link 3 and link 6 ends."""
        return [
            (
                self.world[f"robot{arm}_{link}"]
                @ np.append(cell.CABLE_ANCHORS[arm][link], 1.0)
            )[:3]
            for link in ("link_3", "link_6")
        ]

    def test_arms_share_the_canvas_anchors(self):
        """The independently stored anchors of the two arms agree to a millimetre."""
        for link in ("link_3", "link_6"):
            np.testing.assert_allclose(
                cell.CABLE_ANCHORS[1][link], cell.CABLE_ANCHORS[2][link], atol=1e-3
            )

    def test_drawn_and_constrained_cables_end_on_the_anchors(self):
        """Both cable models start and end exactly on the FK-placed anchors."""
        for arm in (1, 2):
            start, end = self.anchors(arm)
            drawn = self.display.samples(self.world, self.q, arm, 57)
            checked = np.asarray(self.cables[arm].worldPoints(self.q))
            np.testing.assert_allclose(drawn[0], start, atol=1e-9)
            np.testing.assert_allclose(drawn[-1], end, atol=1e-9)
            np.testing.assert_allclose(checked[:, 0], start, atol=1e-9)
            np.testing.assert_allclose(checked[:, -1], end, atol=1e-9)

    def test_nodes_inside_the_guides_lie_on_the_exit_lines(self):
        """Nodes within 50 mm of arc from an end sit on that end's exit normal."""
        for arm in (1, 2):
            nodes = self.display.nodes(self.world, self.q, arm)
            arc = self.display.arc[arm]
            for label, end, link, distance in (
                ("link 3", nodes[0], "link_3", arc * cd.LABEL_SEGMENTS),
                ("link 6", nodes[-1], "link_6", (1.0 - arc) * cd.LABEL_SEGMENTS),
            ):
                normal = (
                    self.world[f"robot{arm}_{link}"][:3, :3] @ cell.CABLE_NORMALS[link]
                )
                inside = (distance > 0) & (distance <= cd.GUIDE_SEGMENTS)
                self.assertTrue(inside.any(), label)
                expected = end + np.outer(distance[inside] * cd.GUIDE_STEP, normal)
                np.testing.assert_allclose(nodes[inside], expected, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
