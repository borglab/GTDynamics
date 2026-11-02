"""Compiled MuJoCo geometry shared by the offline G1 app exporters."""

from typing import Any

import mujoco
import numpy as np


def browser_geometry(model: mujoco.MjModel) -> dict[str, Any]:
    """Return compiled geometry for the shared browser robot renderer.

    Geometry comes from the canonical model, independently of motion recordings.
    MuJoCo geom quaternions remain wxyz; saved world body poses are xyzw.
    """
    fields = (
        "geom_group",
        "geom_type",
        "geom_size",
        "geom_dataid",
        "geom_matid",
        "geom_rgba",
        "geom_pos",
        "geom_quat",
        "geom_bodyid",
        "mat_rgba",
        "mesh_vertadr",
        "mesh_vertnum",
        "mesh_faceadr",
        "mesh_facenum",
        "mesh_vert",
        "mesh_face",
    )
    geometry: dict[str, Any] = {"nbody": model.nbody, "ngeom": model.ngeom}
    geometry.update(
        {name: np.asarray(getattr(model, name)).ravel().tolist() for name in fields}
    )
    return geometry
