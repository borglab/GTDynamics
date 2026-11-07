# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_urdf_display.py
# @brief Check URDF-to-display-geometry conversion for the CMRG model.
# @author GTDynamics contributors

"""Tests for the CMRG URDF display-geometry converter."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import urdf_display as ud  # noqa: E402

BAR_LAB = ud.URDF_ROOT / "bar_lab_tools.urdf"


class TestUrdfDisplay(unittest.TestCase):
    """The CMRG model converts to consistent display geometry."""

    @classmethod
    def setUpClass(cls):
        cls.model = ud.parse_urdf(BAR_LAB)
        cls.geometry = ud.display_geometry(
            cls.model,
            {
                "robot1_tool0": [
                    ud.Visual(
                        ud.URDF_ROOT / "bar_tools" / "TimberGripper.stl",
                        np.zeros(3),
                        np.zeros(3),
                        (0.3, 0.3, 0.3, 1.0),
                    )
                ]
            },
        )

    def test_rpy_quaternion_roundtrip(self):
        """A yaw of 90 degrees maps to the expected unit quaternion."""
        rotation = ud.rotation_from_rpy(np.array([0.0, 0.0, np.pi / 2]))
        quaternion = ud.matrix_to_quaternion_xyzw(rotation)
        np.testing.assert_allclose(
            quaternion, [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)], atol=1e-12
        )

    def test_every_mesh_resolves_and_is_meters(self):
        """All visual meshes exist, and lie within a 20 m cell envelope."""
        for visuals in self.model.visuals.values():
            for visual in visuals:
                vertices, _ = ud.read_stl(visual.mesh)
                self.assertLess(np.abs(vertices).max(), 20.0, visual.mesh)

    def test_geometry_arrays_are_consistent(self):
        """Per-geom arrays match ngeom and mesh indices stay in range."""
        g = self.geometry
        self.assertEqual(g["nbody"], len(self.model.links))
        self.assertEqual(len(g["geom_type"]), g["ngeom"])
        self.assertEqual(len(g["geom_rgba"]), 4 * g["ngeom"])
        self.assertEqual(len(g["geom_quat"]), 4 * g["ngeom"])
        self.assertEqual(len(g["mesh_vert"]) // 3, sum(g["mesh_vertnum"]))
        self.assertEqual(len(g["mesh_face"]) // 3, sum(g["mesh_facenum"]))
        self.assertLess(max(g["mesh_face"]), len(g["mesh_vert"]) // 3)
        self.assertLess(max(g["geom_bodyid"]), g["nbody"])

    def test_prismatic_gantry_moves_arm_base(self):
        """A 1 m EA_X move shifts robot 1's tool0 by 1 m along world x."""
        home = ud.link_transforms(self.model)["robot1_tool0"][:3, 3]
        moved = ud.link_transforms(self.model, {"bridge1_joint_EA_X": 1.0})
        np.testing.assert_allclose(
            moved["robot1_tool0"][:3, 3] - home, [1.0, 0.0, 0.0], atol=1e-9
        )


if __name__ == "__main__":
    unittest.main()
