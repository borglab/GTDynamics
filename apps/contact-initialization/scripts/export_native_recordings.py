#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file export_native_recordings.py
# @brief Export native contact initialization results.
# @author GTDynamics contributors
"""Record the native pose-contact initializer; the browser only replays results.

The input schedule is preserved. Frozen observations, stance associations and
selected slices come back from the native initializer, not a second graph builder.
Reference poses are accessed only after solving, for first-pose-aligned evaluation.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import itertools
from collections.abc import Iterator
import hashlib
import json
from pathlib import Path
import platform
import sys
from time import perf_counter
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[3]
APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
from g1_model import load_g1_model, model_description, prepare_model
from mujoco_io import contact_frames, contact_phases, import_robot, model_bodies, pose
from contact_phases import initialization_slices


CONTACT_ROTATION_SIGMA_DEGREES = 90.0
UPRIGHT_SIGMA_DEGREES = 15.0
MOTION_ROTATION_SIGMA_DEGREES = 180.0
GROUND_SIGMA_METERS = 0.10


def prior_configurations() -> (
    Iterator[tuple[str, bool, bool, gtd.pose_contact_init.Options]]
):
    """Shared app priors; callers supply slice selection and the ground frame."""
    for between, ground, upright in itertools.product((False, True), repeat=3):
        name = (
            "-".join(
                n
                for n, on in (
                    ("between", between),
                    ("ground", ground),
                    ("upright", upright),
                )
                if on
            )
            or "none"
        )
        options = gtd.pose_contact_init.Options()
        options.initializeUpright = True
        options.rotationSigma = float(np.deg2rad(CONTACT_ROTATION_SIGMA_DEGREES))
        options.addUprightFactors = upright
        options.uprightSigma = float(np.deg2rad(UPRIGHT_SIGMA_DEGREES))
        options.betweenRotationSigma = float(np.deg2rad(MOTION_ROTATION_SIGMA_DEGREES))
        options.betweenTranslationSigma = 10.0
        options.addNormalFactors = False
        options.groundSigma = GROUND_SIGMA_METERS
        yield name, between, ground, options


def write_gzip(path: Path, data: dict[str, Any]) -> None:
    """Write a deterministic compressed offline replay bundle."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        gzip.compress(
            json.dumps(data, separators=(",", ":"), allow_nan=False).encode(), mtime=0
        )
    )


def encoded_pose(value: gtsam.Pose3) -> dict[str, Any]:
    """World poses use row-major rotations and metre translations."""
    return {
        "rotation": value.rotation().matrix().ravel().tolist(),
        "translation": value.translation().tolist(),
    }


def prepare_inputs(
    data: dict[str, Any], model: mujoco.MjModel
) -> tuple[gtd.Robot, gtsam.Values, gtd.Trajectory, gtsam.Pose3]:
    """Build the native measured-joint trajectory using the canonical model."""
    robot = import_robot(model_bodies(model))
    frames = contact_frames(model, robot)
    names = data["model"]["jointNames"]
    measured = gtsam.Values()
    mjdata = mujoco.MjData(model)
    description = model_description(model)
    bodies = [model.body("robot/" + name).id for name in description["bodyNames"]]
    for k, sample in enumerate(data["samples"]):
        mjdata.qpos[:] = model.qpos0
        mjdata.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        for name, angle in zip(names, sample["jointAngles"], strict=True):
            joint = robot.joint(name)
            address = model.jnt_qposadr[model.joint("robot/" + name).id]
            measured.insert(
                gtd.JointAngleKey(joint.id(), k), angle - float(model.qpos0[address])
            )
            mjdata.qpos[address] = angle
        mujoco.mj_kinematics(model, mjdata)
        sample["bodyPoints"] = mjdata.xpos[bodies].round(6).tolist()
        sample["feet"] = {
            side: {
                "p": mjdata.xpos[model.body(f"robot/{side}_ankle_roll_link").id]
                .round(9)
                .tolist(),
                "q": mjdata.xquat[model.body(f"robot/{side}_ankle_roll_link").id]
                .round(12)
                .tolist(),
            }
            for side in frames
        }
    # Retain the recording's joint order; all mappings are by name.
    description["jointNames"] = names
    for name in ("soleOutline", "soleCenter"):
        description.pop(name)
    description["soleDefinition"] = (
        "One material sole pose per foot: +X forward, +Y left, +Z up; metres."
    )
    data["model"] = description
    phases = [
        gtd.Phase(
            a, b, gtd.PoseContactConstraintSpec([frames[side] for side in active])
        )
        for a, b, active in contact_phases(data["contacts"], len(data["samples"]))
    ]
    root = robot.link("pelvis")
    bodyTcom = pose(model.body_ipos[root.id()], model.body_iquat[root.id()])
    return robot, measured, gtd.Trajectory(gtd.WalkCycle(phases), 1), bodyTcom


def statistics(samples: list[float]) -> dict[str, Any]:
    return {
        "medianMs": float(np.median(samples)),
        "minMs": min(samples),
        "p95Ms": float(np.quantile(samples, 0.95, method="higher")),
        "samplesMs": samples,
    }


def record_run(
    robot: gtd.Robot,
    measured: gtsam.Values,
    trajectory: gtd.Trajectory,
    bodyTcom: gtsam.Pose3,
    dt: float,
    between: bool,
    ground: bool,
    iterations: int,
    repetitions: int,
    warmup: int,
    options: gtd.pose_contact_init.Options | None = None,
    anchor: gtsam.Pose3 | None = None,
    selected_slices: list[gtd.Slice] | None = None,
) -> tuple[gtd.pose_contact_init.Result, dict[str, Any]]:
    """Time actual native calls without history, then record one separate solve."""
    options = options or gtd.pose_contact_init.Options()
    options.maxIterations = iterations
    options.addBetweenFactors = between
    options.addGroundFactors = ground
    options.recordHistory = False

    phases: dict[str, list[float]] = {
        name: [] for name in ("graphBuild", "optimization", "total", "call")
    }
    for repeat in range(warmup + repetitions + 1):
        options.recordHistory = repeat == warmup + repetitions
        result = gtd.pose_contact_init.Result()
        started = perf_counter()
        values = gtd.PoseContactInitializer(robot=robot, options=options).run(
            rootLinkName="pelvis",
            initialPose=anchor or bodyTcom,
            measuredJoints=measured,
            trajectory=trajectory,
            dt=dt,
            diagnostics=result,
            selectedSlices=selected_slices or [],
        )
        elapsed = (perf_counter() - started) * 1000
        if warmup <= repeat < warmup + repetitions:
            for name in ("graphBuild", "optimization", "total"):
                phases[name].append(getattr(result.timings, name + "Ms"))
            phases["call"].append(elapsed)
        # Dense expansion is part of call wall time, but not the native sparse timer.
        del values
    return result, {
        "runs": repetitions,
        "warmup": warmup,
        "phases": {name: statistics(samples) for name, samples in phases.items()},
        "scope": "Native graphBuild includes measured FK, contact seed and graph construction. Optimization is native LM. Total excludes dense expansion; call includes it. History, evaluation and JSON are excluded.",
    }


def package_run(
    result: gtd.pose_contact_init.Result,
    data: dict[str, Any],
    bodyTcom: gtsam.Pose3,
    root_id: int,
    between: bool,
    ground: bool,
    benchmark: dict[str, Any],
    options: gtd.pose_contact_init.Options | None = None,
) -> dict[str, Any]:
    """Inspect the returned native graph, then evaluate against held-out poses."""
    options = options or gtd.pose_contact_init.Options()
    indices = [s.k for s in result.slices()]
    # This is the first use of reference data; it cannot affect the native solve.
    reference = [
        pose(data["reference"]["poses"][k]["p"], data["reference"]["poses"][k]["q"])
        * bodyTcom
        for k in indices
    ]
    alignment = result.poses[0] * reference[0].inverse()
    reference = [alignment * p for p in reference]
    contacts = [
        {
            "id": i,
            "foot": c.link.name().split("_")[0],
            "comTcontact": encoded_pose(c.comTcontact),
        }
        for i, c in enumerate(result.contactFrames)
    ]
    frames = [
        {"sampleIndex": k, "time": result.times[i], "observations": []}
        for i, k in enumerate(indices)
    ]
    for o in result.observations:
        frames[o.poseIndex]["observations"].append(
            {"contactId": o.contactIndex, "localPose": encoded_pose(o.rootTcontact)}
        )
    groups: dict[str, list[int]] = {
        name: [] for name in ("contact", "motion", "height", "normal", "upright")
    }
    for i in range(result.graph.size()):
        factor = result.graph.at(i)
        if isinstance(factor, gtd.contact_init.UprightFactor):
            groups["upright"].append(i)
            continue
        group = (
            "normal"
            if isinstance(factor, gtd.ContactNormalFactor)
            else (
                "height"
                if isinstance(factor, gtd.ContactHeightFactor)
                else (
                    "contact"
                    if any(gtsam.Symbol(key).chr() == ord("c") for key in factor.keys())
                    else "motion"
                )
            )
        )
        groups[group].append(i)
    history = []
    for snapshot in result.history:
        values = gtsam.Values(result.values)
        for k, p in zip(indices, snapshot.poses):
            values.update(gtd.PoseKey(root_id, k), p)
        contact_keys = sorted(
            k for k in result.values.keys() if gtsam.Symbol(k).chr() == ord("c")
        )
        for key, p in zip(contact_keys, snapshot.contactPoses, strict=True):
            values.update(key, p)
        errors = {
            name: sum(result.graph.at(i).error(values) for i in members)
            for name, members in groups.items()
        }
        np.testing.assert_allclose(
            sum(errors.values()), snapshot.objective / 2, rtol=1e-10, atol=1e-8
        )
        rotation = [
            gtsam.Rot3.Logmap(
                (snapshot.poses[o.poseIndex] * o.rootTcontact)
                .rotation()
                .between(snapshot.contactPoses[o.contactIndex].rotation())
            )
            for o in result.observations
        ]
        position_error = [
            np.linalg.norm(a.translation() - b.translation())
            for a, b in zip(snapshot.poses, reference)
        ]
        angle_error = [
            np.linalg.norm(gtsam.Rot3.Logmap(a.rotation().between(b.rotation())))
            for a, b in zip(snapshot.poses, reference)
        ]
        history.append(
            {
                "iteration": snapshot.iteration,
                "objective": snapshot.objective,
                "rms": snapshot.rms,
                "rotationRms": float(np.sqrt(np.mean(np.square(rotation)) * 3)),
                "poses": [encoded_pose(p) for p in snapshot.poses],
                "contactPoses": [encoded_pose(p) for p in snapshot.contactPoses],
                "factorErrors": errors,
                "referenceError": {
                    "positionRmseMeters": float(
                        np.sqrt(np.mean(np.square(position_error)))
                    ),
                    "orientationRmseDegrees": float(
                        np.rad2deg(np.sqrt(np.mean(np.square(angle_error))))
                    ),
                },
            }
        )
    # Height is derived from the initial gauge and FK only.
    first_contacts = [o["contactId"] for o in frames[0]["observations"]]
    height = float(
        np.mean([result.history[0].contactPoses[i].z() for i in first_contacts])
    )
    return {
        "graph": {
            "frames": frames,
            "contacts": contacts,
            "rootBodyTcom": encoded_pose(bodyTcom),
            "anchorFrameIndices": [0],
            "components": [list(range(len(indices)))],
            "groundHeight": (
                options.groundHeight if options.groundHeight is not None else height
            ),
            "freeDimensions": (len(indices) - 1 + len(contacts)) * 6,
            "factorGroups": [
                {
                    "id": name,
                    "count": len(members),
                    "residualDimension": sum(result.graph.at(i).dim() for i in members),
                }
                for name, members in groups.items()
            ],
            "noise": {
                "contactTranslation": options.translationSigma,
                "contactRotationDegrees": float(np.rad2deg(options.rotationSigma)),
                "height": options.groundSigma,
                "normalDegrees": float(np.rad2deg(options.normalSigma)),
                "uprightDegrees": float(np.rad2deg(options.uprightSigma)),
                "betweenTranslation": options.betweenTranslationSigma,
                "betweenRotationDegrees": float(
                    np.rad2deg(options.betweenRotationSigma)
                ),
            },
            "upright": options.addUprightFactors,
            "initializeUpright": options.initializeUpright,
            "normal": ground and options.addNormalFactors,
            "between": between,
            "ground": ground,
        },
        "result": {
            "history": history,
            "iterations": result.iterations,
            "converged": result.converged,
            "stalled": result.stalled,
        },
        "evaluation": {"alignedReferencePoses": [encoded_pose(p) for p in reference]},
        "benchmark": benchmark,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequences", type=Path, default=APP / "public/sequences.json")
    parser.add_argument("--sequence", action="append")
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--repetitions", type=int, default=11)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--max-slice-interval-ms", type=float, default=200.0)
    args = parser.parse_args()
    if args.iterations < 1 or args.repetitions < 1 or args.warmup < 0:
        parser.error("iterations/repetitions must be positive; warmup nonnegative")
    model = load_g1_model()
    sources = [
        Path(__file__),
        REPO / "examples/g1_mjlab/mujoco_io.py",
        REPO / "examples/g1_mjlab/g1_model.py",
        REPO / "examples/g1_mjlab/contact_phases.py",
        REPO / "gtdynamics/kinematics/PoseContactInitializer.cpp",
        REPO / "gtdynamics/kinematics/PoseContactInitializer.h",
        REPO / "gtdynamics.i",
    ]
    catalogue = json.loads(args.sequences.read_text())
    for sequence in catalogue["sequences"]:
        # SSIO has explicit supported-phase selection and separate prior settings.
        # Its dedicated driver reuses record_run/package_run below.
        if sequence.get("contactEvidence") == "privileged-max-product":
            continue
        if args.sequence and sequence["id"] not in args.sequence:
            continue
        path = args.sequences.parent / sequence["data"]
        data = json.loads(path.read_text())
        robot, measured, trajectory, bodyTcom = prepare_inputs(data, model)
        dt = 1 / sequence["sampleRateHz"]
        selected = initialization_slices(trajectory, dt, args.max_slice_interval_ms)
        np.testing.assert_allclose(
            [s["t"] for s in data["samples"]],
            np.arange(len(data["samples"])) * dt,
            atol=1e-6,
        )
        runs, unavailable = {}, {}
        for name, between, ground, options in prior_configurations():
            options.interpolateOutput = False
            try:
                result, benchmark = record_run(
                    robot,
                    measured,
                    trajectory,
                    bodyTcom,
                    dt,
                    between,
                    ground,
                    args.iterations,
                    args.repetitions,
                    args.warmup,
                    options,
                    selected_slices=selected,
                )
            except ValueError as error:
                if not between and "disconnected contact graph" in str(error):
                    unavailable[name] = (
                        "Disconnected support components require the explicit weak motion prior."
                    )
                    continue
                raise
            runs[name] = package_run(
                result,
                data,
                bodyTcom,
                robot.link("pelvis").id(),
                between,
                ground,
                benchmark,
                options,
            )
            print(
                f"{sequence['id']} / {name}: {result.iterations} iterations; native LM median {benchmark['phases']['optimization']['medianMs']:.2f} ms",
                flush=True,
            )
        data["metadata"]["kinematicModel"] = json.loads(
            (prepare_model() / "provenance.json").read_text()
        )
        path.write_text(json.dumps(data, separators=(",", ":")) + "\n")
        output = {
            "schemaVersion": 3,
            "solver": "PoseContactInitializer.run",
            "recordedAtUtc": datetime.now(timezone.utc).isoformat(),
            "sequenceId": sequence["id"],
            "defaultVariant": "between-ground-upright",
            "runs": runs,
            "unavailable": unavailable,
            "hardware": {"machine": platform.machine(), "system": platform.system()},
            "source": {
                "datasetSha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "sources": {
                    str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sources
                },
                "referenceExcludedFromOptimization": True,
                "sampling": {
                    "mode": "supported-phases",
                    "maxSliceIntervalMs": args.max_slice_interval_ms,
                    "contactBuffer": 1,
                    "longPhaseSeconds": min(0.2, args.max_slice_interval_ms / 1000.0),
                },
            },
        }
        previous_path = args.sequences.parent / sequence["recording"]
        sequence["recording"] = f"recordings/{sequence['id']}.json.gz"
        output_path = args.sequences.parent / sequence["recording"]
        write_gzip(output_path, output)
        # Retire only the superseded output for this regenerated sequence.
        if previous_path != output_path:
            previous_path.unlink(missing_ok=True)
        args.sequences.write_text(json.dumps(catalogue, indent=2) + "\n")


if __name__ == "__main__":
    main()
