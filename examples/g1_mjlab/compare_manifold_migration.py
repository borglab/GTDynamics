#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
# Authors: Yetong Zhang and Frank Dellaert.
# ----------------------------------------------------------------------------
"""Compare bounded affine/nonlinear benchmark artifacts across the migration.

Baseline directories come from commit 6aea8ba1 with --include-legacy; migrated
nonlinear runs use --include-manifold-lm. Both campaigns use three repetitions.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def compare(before, after, affine):
    old = json.loads((before / "comparison.json").read_text())
    new = json.loads((after / "comparison.json").read_text())
    timing = "dynamics_seconds" if affine else "optimizationSeconds"
    objective = "objective" if affine else "objectiveFinal"
    equality = "equality_l2" if affine else "equalityL2"
    bounds = "original_inequality_l2" if affine else "originalInequalityL2"
    cases = [case for case in ("constrained", "gaussian", "gtsam", "proxqp", "gaussian-serial")
             if case in old and case in new]
    cases += ["manifold-lm"] if "legacy" in old and "manifold-lm" in new else []
    report = {}
    for case in cases:
        previous = "legacy" if case == "manifold-lm" else case
        a, b = old[previous]["median"], new[case]["median"]
        differences = {}
        before_times, after_times = [], []
        for repeat in range(1, 4):
            old_run = before / f"{previous}-{repeat}"
            new_run = after / f"{case}-{repeat}"
            old_provenance = json.loads((old_run / "provenance.json").read_text())
            new_provenance = json.loads((new_run / "provenance.json").read_text())
            for field in ("inputFiles", "cpuAffinity", "threads", "modelSceneSha256"):
                assert old_provenance[field] == new_provenance[field], field
            if not affine:
                assert old_provenance["labelsSha256"] == new_provenance["labelsSha256"]
            old_summary = json.loads((old_run / "summary.json").read_text())
            new_summary = json.loads((new_run / "summary.json").read_text())
            if not affine:
                old_settings = dict(old_summary["solverSettings"])
                new_settings = dict(new_summary["solverSettings"])
                # The backend label changed when its representation migrated.
                old_settings.pop("equalityOnlyBaseline", None)
                new_settings.pop("equalityOnlyBaseline", None)
                assert old_settings == new_settings, (case, "solver settings")
            before_times.append(old_summary[timing])
            after_times.append(new_summary[timing])
            old_gtsam = {path: digest for path, digest in old_provenance["loadedLibraries"].items()
                         if "libgtsam.so" in path or "libtbb" in path}
            new_gtsam = {path: digest for path, digest in new_provenance["loadedLibraries"].items()
                         if "libgtsam.so" in path or "libtbb" in path}
            assert old_gtsam == new_gtsam, "GTSAM/TBB binaries changed"
            with np.load(old_run / "states.npz") as x, np.load(new_run / "states.npz") as y:
                assert set(x.files) == set(y.files)
                for key in x.files:
                    if x[key].dtype.kind in "fciu":
                        delta = float(np.max(np.abs(x[key] - y[key]), initial=0))
                        differences[key] = max(differences.get(key, 0), delta)
                    else:
                        assert np.array_equal(x[key], y[key]), key
        objective_delta = abs(b[objective] - a[objective])
        assert objective_delta < (1e-6 if affine else 1e-5), (case, objective_delta)
        assert b[equality] < 1e-5, (case, b[equality])
        assert abs(b[bounds] - a[bounds]) < 1e-5, (case, "original bounds")
        # State tolerances permit existing iterative-QP accuracy without retuning it.
        assert max(differences.values()) < (5e-4 if affine else 1e-4), (case, differences)
        ratio = b[timing] / a[timing]
        report[case] = {
            "beforeSeconds": a[timing], "afterSeconds": b[timing],
            "timeRatio": ratio, "slowdownAboveFivePercent": ratio > 1.05,
            "beforeRunsSeconds": before_times, "afterRunsSeconds": after_times,
            "objectiveBefore": a[objective], "objectiveAfter": b[objective],
            "objectiveDifference": objective_delta,
            "equalityL2Before": a[equality], "equalityL2After": b[equality],
            "originalInequalityL2Before": a[bounds], "originalInequalityL2After": b[bounds],
            "maximumArrayDifferences": differences,
        }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--affine-before", type=Path, required=True)
    parser.add_argument("--affine-after", type=Path, required=True)
    parser.add_argument("--nonlinear-before", type=Path, required=True)
    parser.add_argument("--nonlinear-after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = {
        "baselineCommit": "6aea8ba1",
        "affine": compare(args.affine_before, args.affine_after, True),
        "nonlinear": compare(args.nonlinear_before, args.nonlinear_after, False),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    for category in ("affine", "nonlinear"):
        for case, result in report[category].items():
            print(f"{category:9} {case:15} {result['beforeSeconds']:.6f} -> "
                  f"{result['afterSeconds']:.6f} s ({result['timeRatio']:.3f}x)")


if __name__ == "__main__":
    main()
