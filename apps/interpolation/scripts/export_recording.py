#!/usr/bin/env python3
"""Fit native floating-base GPs and export an offline Interpolation recording."""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import shutil
import sys
import time
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
from g1_model import VERSIONS, MODEL_DIRECTORY
from browser_geometry import browser_geometry
from interpolate_walk import checksum, fit_walk, sample_walk, validate_model_frames


def write_json(path: Path, value: Any) -> None:
    """Write deterministic compressed metadata and model geometry."""
    path.write_bytes(
        gzip.compress(
            json.dumps(value, separators=(",", ":"), allow_nan=False).encode(), mtime=0
        )
    )


def export(args: argparse.Namespace) -> None:
    """Select the winner, fit once, sample dense output, and publish its inputs."""
    started = time.perf_counter()
    walk = fit_walk(
        args.kinematics,
        args.dense_data,
        args.joint_process_sigma_degrees,
        args.joint_measurement_sigma_degrees,
        args.base_angular_process_sigma_degrees,
        args.base_linear_process_sigma,
        args.base_velocity_sigma,
    )
    print(
        f"GP solve: {walk.gp.solveSeconds():.3f} s; encoder RMS {np.rad2deg(walk.gp.jointMeasurementRms()):.3f} deg",
        flush=True,
    )
    arrays = sample_walk(walk)
    oracle = validate_model_frames(walk, arrays)
    arrays["denseAngles"] = walk.dense_angles
    contacts = np.zeros((len(walk.dense_times), 2), dtype=bool)
    for phase in walk.record["contactPhaseInference"]["phases"]:
        selected = (walk.dense_indices >= phase["start"]) & (
            walk.dense_indices <= phase["end"]
        )
        contacts[selected] = [side in phase["contacts"] for side in ("left", "right")]
    stance = {}
    for foot, side in enumerate(("left", "right")):
        twist = arrays["footTwists"][:, foot]
        active = contacts[:, foot]
        stance[side] = {
            "samples": int(active.sum()),
            "linearRmsMetersPerSecond": float(
                np.sqrt(np.mean(np.sum(twist[active, 3:] ** 2, axis=1)))
            ),
            "angularRmsDegreesPerSecond": float(
                np.rad2deg(np.sqrt(np.mean(np.sum(twist[active, :3] ** 2, axis=1))))
            ),
        }
    args.output.mkdir(parents=True, exist_ok=True)
    # Reuse the existing replay codec, with one block per frame and field.
    # Quantization is for display only; diagnostics above use native precision.
    sys.path.insert(0, str(REPO / "apps/kinematics-inspector/scripts"))
    from pack_recordings import encode_chunks

    layout, blocks, offset = {}, [], 0
    for name, array in arrays.items():
        layout[name] = {"offset": offset, "shape": list(array.shape)}
        scaled = np.rint(array.reshape(len(walk.dense_times), -1) * 1e6)
        if (
            not np.isfinite(scaled).all()
            or np.max(np.abs(scaled)) > np.iinfo(np.int32).max
        ):
            raise ValueError(f"Display quantization overflow in {name}")
        blocks.extend(scaled.astype("<i4"))
        offset += array.size
    (args.output / "states.bin.gz").write_bytes(encode_chunks(blocks))
    root = walk.model.body("robot/pelvis").id
    metadata = {
        "schemaVersion": 1,
        "title": "Interpolation",
        "times": walk.dense_times.tolist(),
        "jointNames": walk.gp.jointNames(),
        "jointIds": walk.gp.jointIds(),
        "bodies": walk.model.nbody,
        "rootBody": root,
        "contactBodies": [
            walk.model.body(f"robot/{side}_ankle_roll_link").id
            for side in ("left", "right")
        ],
        "contacts": contacts.astype(int).tolist(),
        "knots": {
            "times": walk.knot_times.tolist(),
            "frames": np.searchsorted(walk.dense_times, walk.knot_times).tolist(),
            "jointAngles": walk.source_angles.tolist(),
        },
        "source": {
            "variant": walk.variant,
            "checksums": walk.input_checksums,
            "motionSource": walk.record["motionSource"],
        },
        "settings": {
            "jointProcessSigmaDegreesPerSecondPowerThreeHalves": args.joint_process_sigma_degrees,
            "jointMeasurementSigmaDegrees": args.joint_measurement_sigma_degrees,
            "baseAngularProcessSigmaDegreesPerSecondPowerThreeHalves": args.base_angular_process_sigma_degrees,
            "baseLinearProcessSigmaMetersPerSecondPowerThreeHalves": args.base_linear_process_sigma,
            "baseVelocitySigmaMetersPerSecond": args.base_velocity_sigma,
            "fixedKnotConfigurations": True,
            "measurementNoiseIncludesConditionalCovariance": True,
            "contactConstraintsApplied": False,
        },
        "diagnostics": {
            "mujocoOracle": oracle,
            "solveSeconds": walk.gp.solveSeconds(),
            "exportSeconds": time.perf_counter() - started,
            "jointMeasurementRmsDegrees": float(
                np.rad2deg(walk.gp.jointMeasurementRms())
            ),
            "jointRmsDegrees": np.rad2deg(
                np.sqrt(
                    np.mean((arrays["jointAngles"] - walk.dense_angles) ** 2, axis=0)
                )
            ).tolist(),
            "processError": walk.gp.processError(),
            "stance": stance,
            "knotIntervalsSeconds": np.diff(walk.knot_times).tolist(),
        },
        "frames": {
            "basePose": "world_T_base_COM",
            "baseTwist": "angular-first, base COM axes",
            "footTwists": "angular-first, world axes, sole origin",
            "angles": "radians",
            "velocities": "radians/second and metres/second",
        },
        "modelVersions": VERSIONS,
        "modelFingerprint": walk.record["modelFingerprint"],
        "provenance": {
            "sources": {
                name: checksum(REPO / name)
                for name in (
                    "gtdynamics/utils/FloatingBaseInterpolator.h",
                    "gtdynamics/utils/FloatingBaseInterpolator.cpp",
                    "gtdynamics.i",
                    "examples/g1_mjlab/interpolate_walk.py",
                    "examples/g1_mjlab/browser_geometry.py",
                    "apps/interpolation/scripts/export_recording.py",
                )
            }
        },
        "storage": {
            "states": "states.bin.gz",
            "model": "model.json.gz",
            "codec": "delta-shuffle-int32-v1",
            "scale": 1000000,
            "blocks": [len(block) for block in blocks],
            "length": offset,
            "arrays": layout,
        },
    }
    write_json(args.output / "recording.json.gz", metadata)
    write_json(args.output / "model.json.gz", browser_geometry(walk.model))
    shutil.copyfile(MODEL_DIRECTORY / "LICENSE", args.output / "MODEL_LICENSE.txt")
    shutil.copyfile(
        args.dense_data / "OPENIO_ATTRIBUTION.txt", args.output / "SSIO_ATTRIBUTION.txt"
    )
    print(json.dumps(metadata["diagnostics"], indent=2)[:1600], flush=True)


def main() -> None:
    """Command-line offline export; no timing or optimization runs in the browser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--kinematics", type=Path, default=REPO / "apps/kinematics-inspector/public"
    )
    parser.add_argument(
        "--dense-data",
        type=Path,
        default=REPO / "apps/kinematics-inspector/.cache/openio-contact-source",
    )
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).resolve().parents[1] / "public"
    )
    parser.add_argument("--joint-process-sigma-degrees", type=float, default=100.0)
    parser.add_argument("--joint-measurement-sigma-degrees", type=float, default=0.5)
    parser.add_argument(
        "--base-angular-process-sigma-degrees", type=float, default=90.0
    )
    parser.add_argument("--base-linear-process-sigma", type=float, default=3.0)
    parser.add_argument("--base-velocity-sigma", type=float, default=0.5)
    export(parser.parse_args())


if __name__ == "__main__":
    main()
