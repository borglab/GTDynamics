#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
# Authors: Yetong Zhang and Frank Dellaert.
# ----------------------------------------------------------------------------
"""Compare affine Dynamics app solvers without changing app recordings.

All cases use the same frozen GP kinematics and 57 source knots in 78--98 s.
Equality-only cases omit bounds explicitly; the constrained reference retains
those bounds. Run sequentially in fresh processes with single-thread BLAS.
"""

import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

from benchmark_equality_only import digest


def worker(args):
    import numpy as np
    import gtsam
    import gtdynamics as gtd

    # Patch the model provider before interpolate_walk imports it.
    if args.model_cache:
        import g1_model
        import mujoco

        g1_model.load_g1_model = lambda: mujoco.MjModel.from_xml_path(
            str(args.model_cache / "scene.xml"))
        g1_model.prepare_model = lambda: args.model_cache

    import dynamics_walk
    import _g1_kinodynamics

    options = gtd.fixed_dynamics.Options()
    options.maxIterations = 50
    options.equalityOnly = args.worker != "constrained"
    options.retractionThreads = 1
    options.linearSolver = {
        "constrained": gtd.fixed_dynamics.LinearSolver.Automatic,
        "gaussian": gtd.fixed_dynamics.LinearSolver.Gaussian,
        "gtsam": gtd.fixed_dynamics.LinearSolver.GtsamQP,
        "proxqp": gtd.fixed_dynamics.LinearSolver.ProxQP,
        "manifold-lm": gtd.fixed_dynamics.LinearSolver.Gaussian,
    }[args.worker]
    native = gtd.solveFixedKinematicsDynamics
    extra = {}

    # Both calls consume exactly the public app's prepared inputs and objective.
    def solve(*inputs):
        before = gtsam.Values(inputs[4])
        if args.worker == "manifold-lm":
            result, preparation = _g1_kinodynamics.solve_fixed_dynamics_manifold_lm(*inputs)
            extra["manifold_lm_preparation_seconds"] = preparation
        else:
            result = native(*inputs)
        assert before.equals(inputs[4], 1e-12), "Solver mutated fixed kinematics"
        return result

    gtd.solveFixedKinematicsDynamics = solve
    solution = dynamics_walk.solve_walk(args.kinematics, args.dense_data, options=options)
    report = {**dynamics_walk.summarize(solution), **extra}
    report["reduced_dimension"] = solution.result.reducedDimension
    report["objective_history"] = list(solution.result.objectiveHistory)
    assert report["samples"] == 57
    assert report["affine_equalities"] and report["conditioned_variable_dimension"] > 0
    assert report["feasible"] and report["equality_l2"] < 1e-5
    assert report["max_force_residual_n"] < 1e-6
    assert report["max_moment_residual_nm"] < 1e-6
    if args.worker == "constrained":
        assert report["original_inequality_l2"] < 1e-5
    else:
        assert report["equality_only"] and report["stationary"], report
    if args.worker == "manifold-lm":
        # Manifold LM has no matching per-phase instrumentation.
        for field in ("optimizer_linearization_seconds", "optimizer_linear_solve_seconds",
                      "retraction_seconds", "equality_preparations", "affine_basis_constructions"):
            report[field] = None
    else:
        assert report["equality_preparations"] == report["affine_basis_constructions"] > 0
        if args.worker == "gaussian":
            assert report["gaussian_solves"] > 0 and report["qp_solves"] == 0
        else:
            assert report["qp_solves"] > 0 and report["gaussian_solves"] == 0

    output = args.output / f"{args.worker}-{args.repetition}"
    output.mkdir(parents=True)
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    np.savez_compressed(output / "states.npz", **dynamics_walk.solution_arrays(solution))
    loaded = sorted({line.split()[-1] for line in Path("/proc/self/maps").read_text().splitlines()
                     if any(name in line for name in ("libgtsam.so", "libgtdynamics.so", "libtbb"))})
    assert any("libtbb.so" in name for name in loaded)
    inputs = [p for folder in (args.kinematics, args.dense_data)
              for p in folder.iterdir() if p.is_file()]
    inputs.extend((args.kinematics / "variants" / solution.walk.variant / name)
                  for name in ("final.json.gz", "final.bin.gz"))
    provenance = {
        "nativeModuleSha256": digest(_g1_kinodynamics.__file__),
        "loadedLibraries": {name: digest(name) for name in loaded},
        "cpuAffinity": sorted(os.sched_getaffinity(0)),
        "inputFiles": {str(path): digest(path) for path in sorted(inputs)},
        "modelSceneSha256": digest(args.model_cache / "scene.xml") if args.model_cache else None,
        "threads": {key: os.environ.get(key) for key in
                    ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


def compare(args, cases):
    import numpy as np

    fields = ("dynamics_seconds", "conditioning_seconds", "anchor_seconds", "assembly_seconds",
              "manifold_seconds", "optimization_seconds", "optimizer_linearization_seconds",
              "optimizer_linear_solve_seconds", "retraction_seconds", "objective", "equality_l2",
              "original_inequality_l2", "projected_gradient_norm", "max_force_residual_n",
              "max_moment_residual_nm", "gp_construction_seconds", "gp_solve_seconds")
    comparison = {"scope": "57 fixed-q/v affine knots, 78--98 s; one retraction worker, TBB enabled"}
    reference_provenance = None
    for case in cases:
        runs = [json.loads((args.output / f"{case}-{i}" / "summary.json").read_text())
                for i in range(1, args.repetitions + 1)]
        comparison[case] = {
            "median": {field: statistics.median(r[field] for r in runs)
                       if all(r[field] is not None for r in runs) else None for field in fields},
            "dynamicsRuns": [r["dynamics_seconds"] for r in runs],
            "iterations": [r["iterations"] for r in runs],
            "basisCounts": [r["affine_basis_constructions"] for r in runs],
            "objectiveHistory": runs[0]["objective_history"],
        }
        for i in range(1, args.repetitions + 1):
            provenance = json.loads((args.output / f"{case}-{i}" / "provenance.json").read_text())
            if reference_provenance is None:
                reference_provenance = provenance
            assert provenance == reference_provenance, "Mismatching benchmark inputs/builds"
    differences = {}
    with np.load(args.output / "gaussian-1" / "states.npz") as reference:
        for case in cases:
            maxima = {}
            for i in range(1, args.repetitions + 1):
                with np.load(args.output / f"{case}-{i}" / "states.npz") as other:
                    for key in reference.files:
                        if reference[key].dtype.kind not in "fciu":
                            assert np.array_equal(reference[key], other[key]), key
                        else:
                            delta = float(np.max(np.abs(reference[key] - other[key]), initial=0))
                            maxima[key] = max(maxima.get(key, 0), delta)
            differences[case] = maxima
            if case != "constrained":
                assert abs(comparison[case]["median"]["objective"] - comparison["gaussian"]["median"]["objective"]) < 1e-6
                tolerance = 5e-4 if case == "proxqp" else 1e-7
                assert max(maxima.values()) < tolerance, (case, maxima)
    comparison["maximumArrayDifferencesFromGaussian"] = differences
    (args.output / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n")
    print(json.dumps(comparison, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kinematics", type=Path, required=True)
    parser.add_argument("--dense-data", type=Path, required=True)
    parser.add_argument("--model-cache", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--include-proxqp", action="store_true")
    parser.add_argument("--worker", choices=("constrained", "gaussian", "gtsam", "proxqp", "manifold-lm"))
    parser.add_argument("--repetition", type=int, default=1)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be positive")
    os.sched_setaffinity(0, sorted(os.sched_getaffinity(0))[:8])
    if args.worker:
        worker(args)
        return
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "source.patch").write_text(subprocess.check_output(
        ["git", "diff", "--binary", "HEAD"], text=True))
    sources = subprocess.check_output(
        ["git", "ls-files", "-c", "-o", "--exclude-standard"], text=True).splitlines()
    (args.output / "source-hashes.json").write_text(json.dumps({
        p: digest(p) for p in sorted(set(sources)) if Path(p).is_file()
        and Path(p).suffix in (".h", ".cpp", ".py", ".cmake")}, indent=2) + "\n")
    cases = ["constrained", "gaussian", "gtsam", "manifold-lm"]
    if args.include_proxqp:
        cases.append("proxqp")
    for repetition in range(1, args.repetitions + 1):
        for case in cases if repetition % 2 else list(reversed(cases)):
            print("START", case, repetition, flush=True)
            with (args.output / f"{case}-{repetition}.log").open("w") as log:
                subprocess.run([sys.executable, __file__, *sys.argv[1:], "--worker", case,
                                "--repetition", str(repetition)],
                               stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
            print("FINISH", case, repetition, flush=True)
    compare(args, cases)


if __name__ == "__main__":
    main()
