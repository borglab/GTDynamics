#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
# Authors: Yetong Zhang and Frank Dellaert.
# ----------------------------------------------------------------------------
"""Time direct and QP linear steps on the same equality-only G1 relaxation.

Run with the same Python/library environment used by kinodynamic_walk.py.
Each repetition starts a fresh process; no build runs concurrently with timing.
The omitted inequalities are reported as diagnostics, not enforced constraints.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def worker(args):
    import numpy as np

    # An explicit prepared model cache makes offline benchmark runs reproducible.
    if args.model_cache:
        import g1_model
        import mujoco

        g1_model.load_g1_model = lambda: mujoco.MjModel.from_xml_path(
            str(args.model_cache / "scene.xml")
        )
        g1_model.prepare_model = lambda: args.model_cache

    from kinodynamic_walk import solve_walk, solution_arrays, summary
    if args.worker == "manifold-lm":
        import kinodynamic_settings
        original_settings = kinodynamic_settings.solver_settings
        kinodynamic_settings.solver_settings = lambda: {
            **original_settings(),
            "equalityOnlyBaseline": "manifold-lm",
        }

    output = args.output / f"{args.worker}-{args.repetition}"
    output.mkdir(parents=True)
    solution = solve_walk(
        args.data, args.labels, start=78.0, end=98.0,
        max_iterations=5,
        retraction_threads=1 if args.worker in ("manifold-lm", "gaussian-serial") else 8,
        initialization="contact-gp-dynamics", equality_only=True,
        linear_solver="gaussian" if args.worker in ("gaussian", "gaussian-serial") else "qp",
        qp_backend="proxqp" if args.worker == "proxqp" else "gtsam",
        qp_use_feasible_step=False,
        qp_use_projected_gradient_initialization=False,
        qp_use_temporal_ordering=False,
    )
    report = summary(solution)
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(output / "states.npz", **solution_arrays(solution))
    assert report["samples"] == 171, report["samples"]
    assert report["iterations"] == 5, report["iterations"]
    assert report["inequalityDimension"] == 0
    assert report["equalityL2"] < 1e-5
    if args.worker != "manifold-lm":
        gaussian = args.worker in ("gaussian", "gaussian-serial")
        unused = "qpSolves" if gaussian else "gaussianSolves"
        used = "gaussianSolves" if gaussian else "qpSolves"
        assert report[unused] == 0 and report[used] > 0
    loaded = sorted({
        line.split()[-1] for line in Path("/proc/self/maps").read_text().splitlines()
        if any(name in line for name in ("libgtsam.so", "libgtdynamics.so", "libtbb"))
    })
    assert any("libtbb.so" in name for name in loaded), loaded
    import _g1_kinodynamics
    provenance = {
        "nativeModuleSha256": digest(_g1_kinodynamics.__file__),
        "loadedLibraries": {name: digest(name) for name in loaded},
        "cpuAffinity": sorted(os.sched_getaffinity(0)),
        "threads": {key: os.environ.get(key) for key in (
            "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
        "labelsSha256": digest(args.labels),
        "inputFiles": {path.name: digest(path) for path in args.data.iterdir()
                       if path.is_file()},
        "modelSceneSha256": digest(args.model_cache / "scene.xml")
            if args.model_cache else None,
        "gitHead": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True).strip(),
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in (
        "optimizationSeconds", "objectiveFinal", "equalityL2",
        "originalInequalityL2", "linearSolveSeconds", "gaussianSolves", "qpSolves"
    )}), flush=True)


def compare(args, cases):
    import numpy as np

    fields = (
        "assemblySeconds", "initializationSeconds", "linearizationSeconds",
        "linearSolveSeconds", "retractionSeconds", "optimizationSeconds",
        "totalInitializationAndSolveSeconds", "objectiveFinal", "equalityL2",
        "originalInequalityL2",
    )
    comparison = {"scope": "Equality-only relaxation; original bounds not enforced"}
    for case in cases:
        runs = [json.loads((args.output / f"{case}-{i}" / "summary.json").read_text())
                for i in range(1, args.repetitions + 1)]
        comparison[case] = {
            "median": {field: statistics.median(run[field] for run in runs)
                       if all(run[field] is not None for run in runs) else None
                       for field in fields},
            "optimizationRuns": [run["optimizationSeconds"] for run in runs],
            "objectives": [run["objectiveFinal"] for run in runs],
            "equalityL2": [run["equalityL2"] for run in runs],
            "gaussianSolves": [run["gaussianSolves"] for run in runs],
            "qpSolves": [run["qpSolves"] for run in runs],
        }
    with np.load(args.output / "gaussian-1" / "states.npz") as reference:
        differences = {}
        for case in cases:
            maxima = {}
            for i in range(1, args.repetitions + 1):
                with np.load(args.output / f"{case}-{i}" / "states.npz") as other:
                    for key in reference.files:
                        if reference[key].dtype.kind not in "fciu":
                            assert np.array_equal(reference[key], other[key]), key
                            continue
                        delta = float(np.max(np.abs(reference[key] - other[key]), initial=0))
                        maxima[key] = max(maxima.get(key, 0), delta)
            differences[case] = maxima
        comparison["maximumArrayDifferencesFromGaussian"] = differences
    (args.output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    # Compare serial manifold LM directly, without the eight-worker confound.
    if "manifold-lm" in cases:
        old = comparison["manifold-lm"]["median"]["optimizationSeconds"]
        new = comparison["gaussian-serial"]["median"]["optimizationSeconds"]
        comparison["serialCmcOverManifoldLmTimeRatio"] = new / old
        with np.load(args.output / "manifold-lm-1" / "states.npz") as a, np.load(
            args.output / "gaussian-serial-1" / "states.npz") as b:
            comparison["serialManifoldLmArrayDifferences"] = {
                key: float(np.max(np.abs(a[key] - b[key]), initial=0))
                for key in a.files if a[key].dtype.kind in "fciu"
            }
        for case in ("manifold-lm", "gaussian-serial"):
            report = json.loads((args.output / f"{case}-1" / "summary.json").read_text())
            comparison[case]["errorHistory"] = report["errorHistory"]
        (args.output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    print(json.dumps(comparison, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model-cache", type=Path)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--include-proxqp", action="store_true")
    parser.add_argument("--include-manifold-lm", action="store_true")
    parser.add_argument("--worker", choices=("gaussian", "gtsam", "proxqp", "manifold-lm", "gaussian-serial"))
    parser.add_argument("--repetition", type=int, default=1)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be positive")
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:8])
    if args.worker:
        worker(args)
        return
    args.output.mkdir(parents=True)
    # Snapshot local edits as well as the commit identity before starting work.
    (args.output / "source.patch").write_text(subprocess.check_output(
        ["git", "diff", "--binary", "HEAD"], text=True))
    source_names = subprocess.check_output(
        ["git", "ls-files", "-c", "-o", "--exclude-standard"], text=True).splitlines()
    (args.output / "source-hashes.json").write_text(json.dumps({
        name: digest(name) for name in sorted(set(source_names))
        if Path(name).is_file() and Path(name).suffix in (".cpp", ".h", ".py", ".cmake")
    }, indent=2) + "\n")
    cases = ["gaussian", "gtsam"] + (["proxqp"] if args.include_proxqp else [])
    cases += ["manifold-lm", "gaussian-serial"] if args.include_manifold_lm else []
    for repetition in range(1, args.repetitions + 1):
        # Alternate run order to reduce systematic warm-cache or thermal bias.
        for case in cases if repetition % 2 else list(reversed(cases)):
            name = f"{case}-{repetition}"
            print("START", name, flush=True)
            with (args.output / f"{name}.log").open("w") as log:
                subprocess.run(
                    [sys.executable, __file__, *sys.argv[1:], "--worker", case,
                     "--repetition", str(repetition)],
                    stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180,
                )
            print("FINISH", name, flush=True)
    compare(args, cases)


if __name__ == "__main__":
    main()
