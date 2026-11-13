#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
"""Run paired point-ADMM/ProxQP benchmarks; never count failed solves as speedups."""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess


def summarize(records: list[dict[str, str]]) -> dict:
    """Require all measured solves to converge before comparing wall times."""
    measured = [row for row in records if int(row["repeat"]) >= 0]
    per_solver = {
        name: [row for row in measured if row["backend"] == name]
        for name in ("admm", "proxqp")
    }
    result: dict = {}
    for name, rows in per_solver.items():
        valid = bool(rows) and all(row["converged"] == "1" for row in rows)
        result[name] = {
            "runs": len(rows),
            "converged": valid,
            "median_seconds": (
                statistics.median(float(row["seconds"]) for row in rows)
                if valid else None
            ),
            "median_iterations": (
                statistics.median(int(row["iterations"]) for row in rows)
                if valid else None
            ),
            "objective": (
                statistics.median(float(row["objective"]) for row in rows)
                if valid else None
            ),
        }
    a, b = result["admm"], result["proxqp"]
    if a["converged"] and b["converged"]:
        difference = abs(a["objective"] - b["objective"])
        tolerance = 1e-6 * max(1.0, abs(b["objective"]))
        result["objective_agrees"] = difference <= tolerance
        result["relative_objective_difference"] = difference / max(
            1.0, abs(b["objective"])
        )
        result["speedup_proxqp_over_admm"] = (
            b["median_seconds"] / a["median_seconds"]
            if result["objective_agrees"] else None
        )
    else:
        result["objective_agrees"] = None
        result["speedup_proxqp_over_admm"] = None
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sizes", type=int, nargs="+", default=[8, 32, 128, 512])
    parser.add_argument("--rhos", type=float, nargs="+", default=[0.1, 1.0, 10.0])
    parser.add_argument("--threads", type=int, nargs="+", default=[1, 4])
    parser.add_argument("--iterations", type=int, default=10000)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=240)
    args = parser.parse_args()
    if min(*args.sizes, *args.threads, args.iterations, args.repeats) <= 0:
        parser.error("sizes, thread counts, iteration limit and repeats must be positive")
    if min(args.rhos) <= 0:
        parser.error("rho must be positive")

    args.output.mkdir(parents=True, exist_ok=True)
    command_prefix = str(args.binary.resolve())
    metadata = {
        "binary": command_prefix,
        "sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        "platform": platform.platform(),
        "cpu_count": os.cpu_count(),
        "parameters": vars(args) | {"binary": command_prefix, "output": str(args.output)},
    }
    (args.output / "provenance.json").write_text(json.dumps(
        metadata, indent=2, default=str
    ) + "\n")
    results: list[dict] = []
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    for size in args.sizes:
        for rho in args.rhos:
            for threads in args.threads:
                label = f"points-{size}-rho-{rho:g}-threads-{threads}"
                command = [
                    command_prefix, str(size), str(rho), str(threads),
                    str(args.iterations), str(args.repeats),
                ]
                try:
                    process = subprocess.run(
                        command, capture_output=True, text=True,
                        timeout=args.timeout, env=env, check=False
                    )
                    rows = list(csv.DictReader(io.StringIO(process.stdout)))
                    report = summarize(rows) if rows else None
                    status = "finished" if process.returncode == 0 else "failed"
                    (args.output / f"{label}.csv").write_text(process.stdout)
                    (args.output / f"{label}.stderr").write_text(process.stderr)
                    outcome = dict(
                        case=label, status=status, returncode=process.returncode,
                        command=command, summary=report
                    )
                except subprocess.TimeoutExpired:
                    outcome = dict(
                        case=label, status="timeout", command=command, summary=None
                    )
                results.append(outcome)
                print(json.dumps(outcome, allow_nan=False), flush=True)
    (args.output / "results.json").write_text(
        json.dumps(results, indent=2, allow_nan=False) + "\n"
    )


if __name__ == "__main__":
    main()
