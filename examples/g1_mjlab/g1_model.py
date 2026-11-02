"""Canonical G1 model assembled through Menagerie, MJLab and MuJoCo Python APIs.

Install requirements-model.txt. No source checkout or raw XML discovery is used.
The material sole frames use body +X forward, +Y left, +Z up, with origins on the
capsules' lower envelope. Binary models and browser geometry are derived caches.
"""

from __future__ import annotations

import hashlib
from importlib.metadata import version
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

REPO = Path(__file__).resolve().parents[2]
MODEL_DIRECTORY = REPO / "models/mujoco/.cache/g1-mjlab"
VERSIONS = {"mujoco-menagerie": "2026.10.6", "mjlab": "1.6.0", "mujoco": "3.11.0"}


def robot_cfg() -> Any:
    """MJLab public G1 configuration with the canonical Menagerie specification."""
    import mujoco_menagerie as menagerie
    from mjlab.asset_zoo.robots import get_g1_robot_cfg

    robot = menagerie.get("unitree_g1")
    if "g1_mjlab" not in robot.entry_names:
        raise ValueError("Pinned Menagerie package must provide g1_mjlab")
    cfg = get_g1_robot_cfg()
    cfg.spec_fn = lambda: robot.spec("g1_mjlab")
    return cfg


def sole_frame(model: mujoco.MjModel, side: str) -> NDArray[np.float64]:
    """Return body_T_contact as a 4x4 matrix, from the compiled foot capsules.

    XY uses the model's foot site. Z is the minimum capsule surface coordinate
    in the ankle-roll body frame. The two feet are inspected independently.
    """
    body = model.body(f"robot/{side}_ankle_roll_link").id
    bottom = []
    for geom in range(model.ngeom):
        if (
            model.geom_bodyid[geom] != body
            or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_CAPSULE
        ):
            continue
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
        half_axis = rotation.reshape(3, 3)[:, 2] * model.geom_size[geom, 1]
        bottom.append(
            model.geom_pos[geom, 2] - abs(half_axis[2]) - model.geom_size[geom, 0]
        )
    if len(bottom) != 7:
        raise ValueError(f"Expected seven {side} foot capsules, got {len(bottom)}")
    transform = np.eye(4)
    transform[:3, 3] = model.site(f"robot/{side}_foot").pos
    transform[2, 3] = min(bottom)
    return transform


def model_description(model: mujoco.MjModel) -> dict[str, Any]:
    """Names and derived geometry for exporters, independent of any recording.

    soleOutline retains the previous rectangular envelope solely for the legacy
    height/speed contact schedule. Kinematics uses the explicit contact frames.
    """
    bodies = [
        b for b in range(1, model.nbody) if model.body(b).name.startswith("robot/")
    ]
    body_map = {b: i for i, b in enumerate(bodies)}
    feet = {s: model.body(f"robot/{s}_ankle_roll_link").id for s in ("left", "right")}
    bounds = []
    for geom in range(model.ngeom):
        if (
            model.geom_bodyid[geom] == feet["left"]
            and model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_CAPSULE
        ):
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
            axis = rotation.reshape(3, 3)[:, 2] * model.geom_size[geom, 1]
            for sign in (-1, 1):
                center = model.geom_pos[geom] + sign * axis
                bounds.extend(
                    [
                        center - model.geom_size[geom, 0],
                        center + model.geom_size[geom, 0],
                    ]
                )
    lo, hi = np.min(bounds, axis=0), np.max(bounds, axis=0)
    outline = [
        [lo[0], lo[1], lo[2]],
        [hi[0], lo[1], lo[2]],
        [hi[0], hi[1], lo[2]],
        [lo[0], hi[1], lo[2]],
    ]
    return {
        "bodyNames": [model.body(b).name.removeprefix("robot/") for b in bodies],
        "bodyParents": [body_map.get(int(model.body_parentid[b]), -1) for b in bodies],
        "jointNames": [
            model.joint(j).name.removeprefix("robot/")
            for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
        ],
        "footBodies": {s: body_map[b] for s, b in feet.items()},
        "contactFrames": {s: sole_frame(model, s).tolist() for s in feet},
        "soleOutline": outline,
        "soleCenter": np.mean(outline, axis=0).tolist(),
        "soleDefinition": "One material frame per foot; +X forward, +Y left, +Z up. Outline is retained only for the legacy contact schedule.",
    }


def prepare_model(directory: Path = MODEL_DIRECTORY) -> Path:
    """Export a pinned MJLab scene and provenance using supported Python APIs."""
    installed = {name: version(name) for name in VERSIONS}
    if installed != VERSIONS:
        raise RuntimeError(
            f"Install requirements-model.txt: expected {VERSIONS}, got {installed}"
        )
    manifest = directory / "provenance.json"
    if manifest.exists():
        saved = json.loads(manifest.read_text())
        if (
            saved.get("packages") == VERSIONS
            and saved.get("pipelineSha256")
            == hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        ):
            if all(
                (directory / name).is_file()
                and hashlib.sha256((directory / name).read_bytes()).hexdigest()
                == digest
                for name, digest in saved["files"].items()
            ):
                return directory
    import mujoco_menagerie as menagerie
    from mjlab.scene import Scene, SceneCfg
    from mjlab.sim import MujocoCfg

    scene = Scene(SceneCfg(entities={"robot": robot_cfg()}), device="cpu")
    scene.spec.worldbody.add_geom(
        name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05]
    )
    model = scene.compile()
    settings = MujocoCfg(timestep=0.005)
    settings.apply(model)
    # Simulation applies options to MjModel separately from Scene. Copy them
    # explicitly into the exported spec so standalone runtimes receive them.
    option_fields = (
        "jacobian",
        "cone",
        "integrator",
        "solver",
        "timestep",
        "impratio",
        "gravity",
        "iterations",
        "tolerance",
        "ls_iterations",
        "ls_tolerance",
        "ccd_iterations",
        "disableflags",
        "enableflags",
    )
    for field in option_fields:
        setattr(scene.spec.option, field, getattr(model.opt, field))
    directory.mkdir(parents=True, exist_ok=True)
    scene.spec.assets.update(menagerie.get("unitree_g1").assets("g1_mjlab"))
    scene.write(directory)
    model = mujoco.MjModel.from_xml_path(str(directory / "scene.xml"))
    if model.nq != 36 or model.nv != 35 or model.nu != 29:
        raise ValueError("Expected the 29-actuator floating G1")
    robot = menagerie.get("unitree_g1")
    (directory / "LICENSE").write_bytes((robot.path() / "LICENSE").read_bytes())
    (directory / "contact-frames.json").write_text(
        json.dumps(model_description(model), indent=2) + "\n"
    )
    files = {
        str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(directory.rglob("*"))
        if p.is_file() and p.name not in ("provenance.json", "model.mjb")
    }
    manifest.write_text(
        json.dumps(
            {
                "packages": installed,
                "entry": "g1_mjlab",
                "modelRevision": robot.oid,
                "pipelineSha256": hashlib.sha256(
                    Path(__file__).read_bytes()
                ).hexdigest(),
                "files": files,
            },
            indent=2,
        )
        + "\n"
    )
    mujoco.mj_saveModel(model, str(directory / "model.mjb"))
    return directory


def load_g1_model() -> mujoco.MjModel:
    """Load the canonical portable scene; binary recordings do not select geometry."""
    return mujoco.MjModel.from_xml_path(str(prepare_model() / "scene.xml"))


if __name__ == "__main__":
    print(prepare_model())
