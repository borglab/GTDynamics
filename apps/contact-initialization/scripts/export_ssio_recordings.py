#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file export_ssio_recordings.py
# @brief Export contact initialization for SSIO snippets.
# @author GTDynamics contributors
"""Replay the Contact Labels snippets through the native pose initializer.

The saved max-product labels (velocity, duration and walking constraints on)
are used verbatim. No new label inference or full-trajectory solve is performed.
The three soft priors are independently selectable through offline recordings.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import platform
import sys

from export_native_recordings import (
    APP,
    REPO,
    package_run,
    prepare_inputs,
    record_run,
    prior_configurations,
    write_gzip,
)
import gtsam
import gtdynamics as gtd
import numpy as np

sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
from g1_model import load_g1_model, model_description, prepare_model
from mujoco_io import contact_frames, load_recording, pose
from contact_phases import initialization_slices


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=REPO / "apps/kinematics-inspector/.cache/openio-contact-source",
    )
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--repetitions", type=int, default=11)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--max-slice-interval-ms", type=float, default=200.0)
    args = parser.parse_args()
    arrays, _ = load_recording(args.source)
    model = load_g1_model()
    description = model_description(model)
    names = description["jointNames"]
    addresses = [model.jnt_qposadr[model.joint("robot/" + n).id] for n in names]
    labels_root = REPO / "apps/contact-labels/public"
    snippets = json.loads((labels_root / "snippets.json").read_text())
    entries = []
    for snippet in snippets:
        labels_path = labels_root / "snippets" / (snippet["id"] + ".json.gz")
        labels = json.loads(gzip.decompress(labels_path.read_bytes()))
        states = labels["supportGraph"]["states"]
        indices = np.rint(np.asarray(labels["times"]) * 50).astype(int)
        np.testing.assert_allclose(indices / 50, labels["times"], atol=1e-7)
        qpos = arrays["qpos"][indices]
        support = [{"left": s in (0, 2), "right": s in (1, 2)} for s in states]
        contacts = []
        for foot in ("left", "right"):
            start = None
            for k in range(len(states) + 1):
                active = k < len(states) and support[k][foot]
                if active and start is None:
                    start = k
                elif not active and start is not None:
                    contacts.append({"foot": foot, "start": start, "end": k - 1})
                    start = None
        data = {
            "model": {"jointNames": names},
            "samples": [
                {
                    "index": k,
                    "t": k / 50,
                    "jointAngles": q[addresses].tolist(),
                    "support": support[k],
                }
                for k, q in enumerate(qpos)
            ],
            "contacts": contacts,
            "reference": {
                "poses": [{"p": q[:3].tolist(), "q": q[3:7].tolist()} for q in qpos]
            },
            "metadata": {
                "kinematicModel": json.loads(
                    (prepare_model() / "provenance.json").read_text()
                ),
                "source": labels["source"],
            },
        }
        robot, measured, trajectory, body_t_com = prepare_inputs(data, model)
        selected = initialization_slices(trajectory, 0.02, args.max_slice_interval_ms)
        # The fixed first root is the reference at the FIRST SELECTED slice,
        # including its height. Subsequent reference poses are evaluation only.
        first = selected[0].k
        anchor = pose(qpos[first, :3], qpos[first, 3:7]) * body_t_com
        # Normalize THIS snippet's world frame using its first selected soles.
        # The full recording's world origin does not imply local ground at z=0.
        # Apply one rigid translation to both the gauge and every reference pose;
        # relative FK, labels, distances and all later measurements are unchanged.
        root = robot.link("pelvis")
        fk_input = gtsam.Values(measured)
        fk_input.insert(gtd.PoseKey(root.id(), first), anchor)
        for joint in robot.joints():
            fk_input.insert(gtd.JointVelKey(joint.id(), first), 0.0)
        fk = robot.forwardKinematics(fk_input, first, "pelvis")
        frames = contact_frames(model, robot)
        ground_height = float(
            np.mean(
                [
                    frames[side].predict(fk, first).z()
                    for side, active in support[first].items()
                    if active
                ]
            )
        )
        shift = np.array([0.0, 0.0, -ground_height])
        anchor = gtsam.Pose3(anchor.rotation(), anchor.translation() + shift)
        for reference in data["reference"]["poses"]:
            reference["p"] = (np.asarray(reference["p"]) + shift).tolist()
        data["metadata"]["snippetWorldTranslation"] = shift.tolist()
        data["metadata"][
            "groundSource"
        ] = "Mean sole height at the first selected supported slice"
        runs = {}
        for name, between, ground, options in prior_configurations():
            options.interpolateOutput = False
            options.groundHeight = 0.0
            result, benchmark = record_run(
                robot,
                measured,
                trajectory,
                body_t_com,
                0.02,
                between,
                ground,
                args.iterations,
                args.repetitions,
                args.warmup,
                options,
                anchor,
                selected_slices=selected,
            )
            run = package_run(
                result,
                data,
                body_t_com,
                robot.link("pelvis").id(),
                between,
                ground,
                benchmark,
                options,
            )
            run["benchmark"][
                "scope"
            ] = "Native graph build includes measured FK and starting values; optimization is LM. No dense expansion. History, evaluation and JSON are excluded from timed calls."
            runs[name] = run
            final = run["result"]["history"][-1]
            print(
                f"{snippet['id']}/{name}: {result.iterations} iterations; converged={result.converged}; reference RMS={final['referenceError']['positionRmseMeters']:.3f} m; LM={benchmark['phases']['optimization']['medianMs']:.2f} ms",
                flush=True,
            )
        sequence_id = "ssio-" + snippet["id"]
        data_path = APP / "public" / "snippets" / (sequence_id + ".json.gz")
        write_gzip(data_path, data)
        source_paths = [
            Path(__file__),
            APP / "scripts/export_native_recordings.py",
            REPO / "examples/g1_mjlab/contact_phases.py",
            REPO / "gtdynamics/kinematics/PoseContactInitializer.cpp",
            REPO / "gtdynamics/kinematics/PoseContactInitializer.h",
            REPO / "gtdynamics/utils/ContactInitializationFactors.h",
            REPO / "gtdynamics.i",
            labels_path,
        ]
        recording = {
            "schemaVersion": 3,
            "solver": "PoseContactInitializer.run",
            "recordedAtUtc": datetime.now(timezone.utc).isoformat(),
            "sequenceId": sequence_id,
            "defaultVariant": "between-ground-upright",
            "runs": runs,
            "unavailable": {},
            "hardware": {"machine": platform.machine(), "system": platform.system()},
            "source": {
                "datasetSha256": hashlib.sha256(data_path.read_bytes()).hexdigest(),
                "sources": {
                    str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in source_paths
                },
                "referenceExcludedFromOptimization": False,
                "referencePoseFactorCount": 0,
                "referenceUsage": "LiDAR-derived max-product labels and first selected root gauge; no subsequent reference-pose factors.",
                "labelStates": states,
                "labelSource": str(labels_path.relative_to(REPO)),
                "sourceIndices": indices.tolist(),
                "selectedSourceIndices": [int(indices[s.k]) for s in selected],
                "sampling": {
                    "mode": "supported-phases",
                    "maxSliceIntervalMs": args.max_slice_interval_ms,
                    "contactBuffer": 1,
                    "longPhaseSeconds": min(0.2, args.max_slice_interval_ms / 1000.0),
                },
            },
        }
        recording_path = APP / "public" / "recordings" / (sequence_id + ".json.gz")
        write_gzip(recording_path, recording)
        entries.append(
            {
                "id": sequence_id,
                "title": snippet["label"],
                "data": str(data_path.relative_to(APP / "public")),
                "recording": str(recording_path.relative_to(APP / "public")),
                "description": "SSIO G1 · saved max-product support labels",
                "sourceUrl": labels["source"]["sourceUrl"],
                "contactEvidence": "privileged-max-product",
                "durationSeconds": 20,
                "sampleRateHz": 50,
            }
        )
    catalogue_path = APP / "public/sequences.json"
    catalogue = json.loads(catalogue_path.read_text())
    catalogue["sequences"] = entries + [
        e for e in catalogue["sequences"] if not e["id"].startswith("ssio-")
    ]
    catalogue_path.write_text(json.dumps(catalogue, indent=2) + "\n")


if __name__ == "__main__":
    main()
