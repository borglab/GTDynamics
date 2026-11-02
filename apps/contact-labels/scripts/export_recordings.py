#!/usr/bin/env python3
"""Derive privileged SSIO stationary-contact targets with the native utility.

Only labels and display transforms are exported. No contact detector, initializer,
classifier, or optimization runs in the browser. The LiDAR reference is deliberately
used to construct training targets, never represented as a classifier input.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any
from time import perf_counter

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[3]
APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "apps/kinematics-inspector/scripts"))
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
from import_openio import (
    CHECKSUMS,
    CLIP,
    DATASET,
    JOINT_NAMES,
    REVISION,
    download,
    resample_qpos,
    validate_mapping,
)
from export_recording import export_model
from mujoco_io import pose, point_velocity_in_frame
from factor_report import write_factor_report
from g1_model import load_g1_model, sole_frame, prepare_model

SNIPPETS = [
    {"id": "onset", "label": "First turn and walking · 24–44 s", "start": 24},
    {"id": "middle", "label": "Mid-clip turns · 78–98 s", "start": 78},
    {"id": "return", "label": "Return turn · 198–218 s", "start": 198},
]
PRESETS = {
    "strict": [0.05, 0.15, 0.005, 0.015],
    "default": [0.1, 0.25, 0.01, 0.025],
    "tolerant": [0.15, 0.4, 0.015, 0.04],
}


def compressed(path: Path, value: Any) -> None:
    """Write one deterministic JSON+gzip archive without duplicate plain JSON."""
    content = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
    path.write_bytes(gzip.compress(content, mtime=0))


def support_graph_result(
    foot_poses: dict[str, list[gtsam.Pose3]],
    body_velocities: dict[str, list[list[float]]],
    times: np.ndarray,
) -> dict[str, Any]:
    """Solve support with/without joint-velocity unaries.

    Graphs include ten context slices per end plus one preceding pose.
    Displacement at output slice k
    belongs to the edge arriving at that sample. Forward velocity is +X in the
    pelvis frame and comes from measured joint dq through the model Jacobian.
    """
    distances = {
        side: np.linalg.norm(np.diff([p.translation() for p in poses], axis=0), axis=1)
        for side, poses in foot_poses.items()
    }
    params = gtd.ContactSupportParams()
    graph: gtsam.DiscreteFactorGraph = gtd.contactSupportGraph(
        distances["left"].tolist(), distances["right"].tolist(), params
    )
    count = len(times) - 1

    def display_states(values: gtsam.DiscreteValues) -> list[int]:
        # Native pair order WW/WS/SW/SS maps to Neither/R/L/Double stance.
        return [
            [3, 1, 0, 2][2 * values[2 * k] + values[2 * k + 1]]
            for k in range(10, count - 10)
        ]

    def solve(candidate: gtsam.DiscreteFactorGraph) -> dict[str, Any]:
        started = perf_counter()
        states: gtsam.DiscreteValues = candidate.optimize(
            gtsam.Ordering.OrderingType.NATURAL
        )
        elapsed = perf_counter() - started
        return {
            "states": display_states(states),
            "solveSeconds": elapsed,
            "factorCount": candidate.size(),
        }

    baseline = solve(graph)
    constrained = gtsam.DiscreteFactorGraph()
    constrained.push_back(graph)
    constrained.push_back(gtd.contactWalkingConstraints(count))
    constrained_baseline = solve(constrained)
    velocity_scale, minimum_probability = 0.1, 0.02
    unaries: gtsam.DiscreteFactorGraph = gtd.contactVelocityFactors(
        [v[0] for v in body_velocities["left"][1:]],
        [v[0] for v in body_velocities["right"][1:]],
        velocity_scale,
        minimum_probability,
    )
    graph.push_back(unaries)
    unconstrained_aided = solve(graph)
    constrained.push_back(unaries)
    result = {
        **solve(constrained),
        "withoutVelocity": constrained_baseline,
        "withoutWalkingConstraints": {
            **unconstrained_aided,
            "withoutVelocity": baseline,
        },
        "enforceWalkingSequence": True,
        "walkingConstraintFactorCount": 2 * count - 1,
        "stateNames": ["L", "R", "Double stance", "Neither"],
        "binaryStateNames": ["Swing", "Stance"],
        "jointConfigurationOrder": ["WW", "WS", "SW", "SS"],
        "displacements": {s: d[10:-10].tolist() for s, d in distances.items()},
        "bodyVelocities": {s: v[11:-10] for s, v in body_velocities.items()},
        "velocityScale": velocity_scale,
        "minimumProbability": minimum_probability,
        "velocityFrame": "Derivative of sole-origin position relative to pelvis, in pelvis axes; +X forward; m/s",
        "velocitySource": "Measured joint dq (rad/s), linearly interpolated to 50 Hz; MuJoCo point Jacobian; no LiDAR velocity",
        "transition": params.transition.tolist(),
        "supportSigma": params.supportSigma,
        "swingSigma": params.swingSigma,
        "sampleIntervalSeconds": 0.02,
        "initialPrior": [0.25] * 4,
        "noSupportWeight": params.noSupportWeight,
        "singleSliceWeight": params.singleSliceWeight,
        "sliceCount": count,
        "variableCount": 2 * count,
        "contextSamplesPerEnd": 10,
        "inference": "GTSAM DiscreteFactorGraph.optimize(NATURAL): max-product and traceback",
        "measurement": "Two binary variables per slice. Each per-foot binary factor uses "
        "the incoming displacements at both adjacent slices, with overlap weights. "
        "Four-variable transitions, within-slice support coupling, three-slice per-foot "
        "duration priors, unary joint velocities, and optional hard walking constraints.",
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=REPO / "apps/kinematics-inspector/.cache/openio/source" / CLIP,
    )
    parser.add_argument("--output", type=Path, default=APP / "public")
    args = parser.parse_args()
    download(args.source)
    arrays = {
        name: np.load(args.source / f"{name}.npy")
        for name in ("time_ns", "pose", "joints")
    }
    spec = json.loads((args.source / "spec.json").read_text())
    times = (arrays["time_ns"] - arrays["time_ns"][0]).astype(float) * 1e-9
    channels = json.loads((args.source / "joints.json").read_text())["channels"]
    angles = arrays["joints"][:, :, channels["q"][0]]
    joint_velocities = arrays["joints"][:, :, channels["dq"][0]]
    if not np.isfinite(joint_velocities).all():
        raise ValueError("Nonfinite measured joint dq")
    model = load_g1_model()
    addresses = [
        int(model.jnt_qposadr[model.joint("robot/" + name).id]) for name in JOINT_NAMES
    ]
    dof_addresses = [
        int(model.jnt_dofadr[model.joint("robot/" + name).id]) for name in JOINT_NAMES
    ]
    pelvis_id = model.body("robot/pelvis").id
    mapping = validate_mapping(
        model,
        arrays["pose"],
        angles,
        spec["names"],
        addresses,
        model.site("robot/imu_in_pelvis").pos,
    )
    contact_frames = {
        side: gtsam.Pose3(sole_frame(model, side)) for side in ("left", "right")
    }
    foot_ids = {
        side: model.body(f"robot/{side}_ankle_roll_link").id
        for side in ("left", "right")
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "snippets").mkdir(exist_ok=True)
    export_model(model, args.output)
    shutil.copyfile(
        REPO / "apps/kinematics-inspector/public/MODEL_LICENSE.txt",
        args.output / "MODEL_LICENSE.txt",
    )
    notice = (
        f"SSIO / Open-IO G1 Inertial Odometry\n{DATASET}\nClip: {CLIP}\nRevision: {REVISION}\n"
        "Creators: Yuheng Qiu and the SSIO authors / Open-IO\nLicense: Creative Commons Attribution 4.0\n"
        "https://creativecommons.org/licenses/by/4.0/\n"
        "Changes: selected three 20-second windows; resampled reference poses and encoders to 50 Hz; "
        "applied one constant world translation per window for display; derived contact-frame motion and "
        "stationarity targets, walking support inferences, and discrete max-product support labels with GTDynamics. "
        "Added pelvis-relative sole velocities from measured joint dq and unary support factors. "
        "World displacement targets use privileged LiDAR-inertial reference poses; "
        "they are not measured force/contact ground truth. Raw source files remain in an ignored cache.\n"
    )
    (args.output / "SSIO_ATTRIBUTION.txt").write_text(notice)
    fingerprints = {
        str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [
            Path(__file__),
            Path(__file__).with_name("factor_report.py"),
            Path(__file__).with_name("typeset_report.mjs"),
            REPO / "examples/g1_mjlab/mujoco_io.py",
            REPO / "gtdynamics/utils/ContactMotion.h",
            REPO / "gtdynamics/utils/ContactMotion.cpp",
            REPO / "examples/g1_mjlab/g1_model.py",
            REPO / "apps/kinematics-inspector/scripts/import_openio.py",
        ]
    }
    for snippet in SNIPPETS:
        start = snippet["start"]
        # Extra context ensures labels at the display edges have full windows.
        target = start + np.arange(-11, 1010) * 0.02
        qpos = resample_qpos(model, times, target, arrays["pose"], angles, addresses)
        data = mujoco.MjData(model)
        transforms, foot_poses = [], {side: [] for side in foot_ids}
        body_velocities = {side: [] for side in foot_ids}
        sampled_dq = np.column_stack(
            [np.interp(target, times, dq) for dq in joint_velocities.T]
        )
        for row, dq in zip(qpos, sampled_dq):
            data.qpos[:] = row
            data.qvel[:] = 0
            data.qvel[dof_addresses] = dq
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            transforms.append(np.c_[data.xpos, data.xquat[:, [1, 2, 3, 0]]])
            for side, body in foot_ids.items():
                body_velocities[side].append(
                    point_velocity_in_frame(
                        model, data, body, contact_frames[side].translation(), pelvis_id
                    ).tolist()
                )
                foot_poses[side].append(
                    pose(data.xpos[body], data.xquat[body]) * contact_frames[side]
                )
        support_graph = support_graph_result(foot_poses, body_velocities, target)
        # The first pose supplies d[0] only; retain ten context slices per end.
        target = target[1:]
        foot_poses = {side: poses[1:] for side, poses in foot_poses.items()}
        transforms = np.array(transforms[1:])
        # A CONSTANT translation changes no speed/excursion or derived label.
        origin = transforms[10, model.body("robot/pelvis").id, :3].copy()
        origin[2] = min(foot_poses[side][10].translation()[2] for side in foot_ids)
        transforms[:, :, :3] -= origin
        feet = {}
        raw_samples = {}
        preset_params = {}
        for side, poses in foot_poses.items():
            motion = None
            targets = {}
            for name, thresholds in PRESETS.items():
                params = gtd.ContactPoseMotionParams()
                (
                    params.stationarySpeed,
                    params.movingSpeed,
                    params.stationaryExcursion,
                    params.movingExcursion,
                ) = thresholds
                angular_scale = {"strict": 0.5, "default": 1.0, "tolerant": 1.5}[name]
                params.stationaryAngularSpeed = 0.3 * angular_scale
                params.movingAngularSpeed = 0.8 * angular_scale
                params.stationaryAngularExcursion = 0.03 * angular_scale
                params.movingAngularExcursion = 0.08 * angular_scale
                samples = gtd.contactPoseMotionLabels(target.tolist(), poses, params)
                selected = samples[10:-10]
                raw_samples[side, name] = selected
                preset_params[name] = params
                targets[name] = {
                    "labels": [s.label for s in selected],
                    "scores": [s.score for s in selected],
                }
                if motion is None:
                    motion = {
                        "maxSpeed": [s.maxSpeed for s in selected],
                        "maxExcursion": [s.maxExcursion for s in selected],
                        "angularSpeed": [s.angularSpeed for s in selected],
                        "angularExcursion": [s.angularExcursion for s in selected],
                        "completeWindow": [s.completeWindow for s in selected],
                    }
            feet[side] = {
                "body": foot_ids[side],
                "motion": motion,
                "targets": targets,
                "bodyTcontact": contact_frames[side].matrix().tolist(),
                "worldContactPoses": [
                    np.r_[
                        p.translation() - origin, p.rotation().toQuaternion().coeffs()
                    ]
                    .round(6)
                    .tolist()
                    for p in poses[10:-10]
                ],
            }

        for side, other in (("left", "right"), ("right", "left")):
            for name, params in preset_params.items():
                feet[side]["targets"][name]["walkingLabels"] = gtd.walkingContactLabels(
                    raw_samples[side, name], raw_samples[other, name], params
                )

        record = {
            "schemaVersion": 7,
            "supportGraph": support_graph,
            "walkingPrior": {
                "inferredLabel": 2,
                "meaning": "Likely contact, not necessarily a stationary sole",
                "rule": "Both windows complete; other origin at or above either moving "
                "translation bound, this origin below both. Keep raw stationary labels.",
                "assumption": "Walking with support; not a calibrated probability or force measurement",
            },
            "id": snippet["id"],
            "label": snippet["label"],
            "times": target[10:-10].tolist(),
            "sampleRateHz": 50,
            "durationSeconds": 20,
            "bodyCount": model.nbody,
            "rootBody": model.body("robot/pelvis").id,
            "poses": np.round(transforms[10:-10].reshape(1000, -1), 6).tolist(),
            "feet": feet,
            "windowSeconds": 0.2,
            "presets": {
                name: dict(
                    zip(
                        [
                            "stationarySpeed",
                            "movingSpeed",
                            "stationaryExcursion",
                            "movingExcursion",
                        ],
                        thresholds,
                    )
                )
                for name, thresholds in PRESETS.items()
            },
            "source": {
                "dataset": DATASET,
                "clip": CLIP,
                "revision": REVISION,
                "sourceUrl": f"{DATASET}/tree/{REVISION}/{CLIP}",
                "checksums": CHECKSUMS,
                "mappingValidation": mapping,
                "displayWorldTranslation": (-origin).tolist(),
                "targetInputs": [
                    "SuperOdom LiDAR-inertial pelvis pose",
                    "joint encoders",
                    "canonical robot contact frames",
                ],
                "futureClassifierInputs": [
                    "q",
                    "dq",
                    "tau_est",
                    "pelvis accelerometer",
                    "pelvis gyroscope",
                ],
            },
            "provenance": {
                "sources": fingerprints,
                "nativeModule": hashlib.sha256(
                    Path(gtd.gtdynamics.__file__).read_bytes()
                ).hexdigest(),
            },
        }
        for name, thresholds in record["presets"].items():
            scale = {"strict": 0.5, "default": 1.0, "tolerant": 1.5}[name]
            thresholds.update(
                stationaryAngularSpeed=0.3 * scale,
                movingAngularSpeed=0.8 * scale,
                stationaryAngularExcursion=0.03 * scale,
                movingAngularExcursion=0.08 * scale,
            )
        record["provenance"]["model"] = json.loads(
            (prepare_model() / "provenance.json").read_text()
        )
        compressed(args.output / "snippets" / f"{snippet['id']}.json.gz", record)
        print(
            f"Exported {snippet['id']}: 1000 frames, native labels for {len(PRESETS)} presets",
            flush=True,
        )
    write_factor_report(args.output / "contact-factors.html", support_graph)
    (args.output / "snippets.json").write_text(json.dumps(SNIPPETS, indent=2) + "\n")


if __name__ == "__main__":
    main()
