#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file capture_run.py
# @brief Capture a sparse G1 clip or full trajectory with input provenance.
# @author GTDynamics contributors
"""Run with capture.so preloaded; successful capture intentionally exits 86."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess

import mujoco
import numpy as np

import g1_model


def sha256(path):
    """Hash one input or binary without keeping its contents in memory."""
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def revision(directory):
    """Return the exact Git revision used to generate the problem."""
    return subprocess.check_output(
        ["git", "-C", str(directory), "rev-parse", "HEAD"], text=True
    ).strip()


def main():
    """Record the actual native options and capture the second global QP."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--name", default="G1SP20")
    parser.add_argument("--start", type=float, default=78.0)
    parser.add_argument("--end", type=float, default=98.0)
    parser.add_argument(
        "--full",
        action="store_true",
        help="Infer support on the complete recording and capture its QP",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    inputs = args.inputs
    cache = inputs / "models/mujoco/.cache/g1-mjlab"
    recording = inputs / "apps/kinematics-inspector/.cache/openio-contact-source"
    labels = inputs / "apps/contact-labels/public/snippets/middle.json.gz"
    g1_model.load_g1_model = lambda: mujoco.MjModel.from_xml_path(
        str(cache / "scene.xml")
    )
    g1_model.prepare_model = lambda: cache
    import kinodynamic_walk
    import _g1_kinodynamics
    import gtdynamics

    if args.full:
        from contact_phases import infer_contact_phases
        from mujoco_io import load_kinematic_recording

        source = load_kinematic_recording(recording, recording / "contacts.json")
        _, phase_info = infer_contact_phases(source, recording)
        states = np.full(source.count, -1, dtype=int)
        for phase in phase_info["phases"]:
            contacts = set(phase["contacts"])
            state = {
                frozenset(["left"]): 0,
                frozenset(["right"]): 1,
                frozenset(["left", "right"]): 2,
            }[frozenset(contacts)]
            states[phase["start"] : phase["end"] + 1] = state
        if np.any(states < 0):
            raise ValueError("Full contact schedule is incomplete")
        labels = args.output / "support-labels.json.gz"
        labels.write_bytes(
            gzip.compress(
                json.dumps(
                    {
                        "times": source.times.tolist(),
                        "supportGraph": {"states": states.tolist()},
                        "provenance": phase_info,
                    }
                ).encode(),
                mtime=0,
            )
        )
        args.start, args.end = float(source.times[0]), float(source.times[-1])
        print(
            f"Full schedule: {source.count} source samples, end={args.end}", flush=True
        )
        del source

    library = Path(os.environ["G1_QP_GTSAM_LIBRARY"])
    gtsam_source = library.parents[2]
    gtd_source = Path(kinodynamic_walk.__file__).resolve().parents[2]
    files = list(recording.glob("*")) + [
        labels,
        cache / "scene.xml",
        cache / "provenance.json",
    ]
    binaries = [library, Path(_g1_kinodynamics.__file__)]
    record = {
        "gtdynamics_revision": revision(gtd_source),
        "gtsam_revision": revision(gtsam_source),
        "gtdynamics_module": gtdynamics.__file__,
        "numpy": np.__version__,
        "mujoco": mujoco.__version__,
        "input_sha256": {str(path): sha256(path) for path in files if path.is_file()},
        "binary_sha256": {str(path): sha256(path) for path in binaries},
        "recording_metadata": json.loads((recording / "walk.json").read_text()),
        "model_provenance": json.loads((cache / "provenance.json").read_text()),
        "loaded_libraries": sorted(
            {
                line.split()[-1]
                for line in Path("/proc/self/maps").read_text().splitlines()
                if "/libtbb.so" in line or "/libgtsam.so" in line
            }
        ),
        "gtsam_with_tbb": "GTSAM_WITH_TBB:BOOL=ON"
        in (gtsam_source / "build/CMakeCache.txt").read_text(),
        "gtdynamics_with_tbb": "GTDYNAMICS_WITH_TBB:BOOL=ON"
        in (gtd_source / "build/CMakeCache.txt").read_text(),
        "dataset_name": args.name,
        "clip_seconds": [args.start, args.end],
        "source_sha256": {
            str(path.relative_to(gtd_source)): sha256(path)
            for path in sorted((gtd_source / "examples/g1_mjlab").glob("*.py"))
        },
    }
    (args.output / "source_changes.patch").write_text(
        subprocess.check_output(
            ["git", "-C", str(gtd_source), "diff", "--", "examples/g1_mjlab"], text=True
        )
    )
    original_prepare = kinodynamic_walk.prepare_walk

    def record_preparation(*parameters):
        walk, metadata = original_prepare(*parameters)
        record["sampling"] = metadata["optimizationSampling"]
        record["optimization_source_slices"] = metadata["optimizationSourceSlices"]
        record["retained_source_indices"] = walk.source_indices.tolist()
        return walk, metadata

    kinodynamic_walk.prepare_walk = record_preparation
    original = kinodynamic_walk.solve_coupled_problem

    def record_and_solve(walk, initial, measurements, patches, armature, options):
        record["solver_settings"] = options
        record["samples"] = len(walk.times)
        record["first_sample_seconds"] = float(walk.times[0])
        record["last_sample_seconds"] = float(walk.times[-1])
        (args.output / "provenance.json").write_text(
            json.dumps(record, indent=2) + "\n"
        )
        return original(walk, initial, measurements, patches, armature, options)

    kinodynamic_walk.solve_coupled_problem = record_and_solve
    kinodynamic_walk.solve_walk(
        recording,
        labels,
        start=args.start,
        end=args.end,
        max_iterations=5,
        retraction_threads=8,
        initialization="contact-gp-dynamics",
    )
    raise RuntimeError("Capture hook did not stop at the second global QP")


if __name__ == "__main__":
    main()
