#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file g1_cases.py
# @brief Measure G1 CMC friction, fixed dynamics, and coupled kinodynamics cases.
# @author GTDynamics contributors
"""Run G1 examples against a specified library snapshot and save all results."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples/g1_mjlab"))


def requests(data_path: Path, count: int) -> list[dict]:
    """Use the existing model snapshot API for deterministic walking samples."""
    import mujoco
    import numpy as np
    from full_model import snapshot
    from mujoco_io import load_model, load_recording

    walk, metadata = load_recording(data_path)
    indices = np.linspace(
        round(2 / metadata["policy_dt"]), len(walk["qpos"]) - 1, count, dtype=int
    ).tolist()
    result = []
    for cone in ("pyramidal", "elliptic"):
        model = load_model(data_path, cone)
        for index in indices:
            data = mujoco.MjData(model)
            for name in ("qpos", "qvel", "ctrl"):
                getattr(data, name)[:] = walk[name][index]
            mujoco.mj_forward(model, data)
            request = snapshot(model, data, cone)
            for variant in (
                ["pyramidal"]
                if cone == "pyramidal"
                else ["elliptic_norm", "signed_square", "friction_factor"]
            ):
                for retractor in ("reduced_al", "ambient_barrier", "reduced_barrier"):
                    result.append(
                        dict(
                            name=f"full-{variant}-{retractor}-{index}",
                            payload=dict(
                                request,
                                solver="CMC",
                                friction_model=variant,
                                friction_sigma=0.001,
                                retractor=retractor,
                            ),
                        )
                    )
    return result


def walking_cases() -> list[dict]:
    """List the fixed-dynamics and coupled trajectory entry points."""
    cases = []
    for name, script, extra in [
        ("fixed-dynamics", "dynamics_walk.py", []),
        ("fixed-dynamics-soft", "dynamics_walk.py", ["--soft-base-balance"]),
        ("kinodynamics-full", "kinodynamic_walk.py", []),
        ("kinodynamics-one", "kinodynamic_walk.py", ["--max-iterations", "1"]),
        ("kinodynamics-short", "kinodynamic_walk.py", ["--end", "79"]),
    ]:
        cases.append(dict(name=name, script=script, extra=extra))
    return cases


def main() -> None:
    """Save full native responses, Python example arrays, and failures."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--paired-snapshot", type=Path)
    parser.add_argument("--paired-output", type=Path)
    parser.add_argument("--requests", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=24)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--python-only", action="store_true")
    parser.add_argument("--full-only", action="store_true")
    parser.add_argument(
        "--cpu", type=int, default=3, help="CPU affinity shared by both versions"
    )
    parser.add_argument(
        "--gtsam-python", type=Path, default=ROOT.parent / "gtsam/build/python"
    )
    parser.add_argument(
        "--walk-python-path",
        type=Path,
        action="append",
        default=[],
        help="Additional Python package directory for pinned walking-model dependencies",
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--shards", type=int, default=1)
    parser.add_argument("--shard", type=int, default=0)
    args = parser.parse_args()
    if args.shards < 1 or not 0 <= args.shard < args.shards:
        parser.error("invalid shard index/count")
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.requests.exists():
        args.requests.write_text(
            json.dumps(requests(ROOT / "examples/g1_mjlab/results", args.samples))
        )
    cases = [] if args.python_only else json.loads(args.requests.read_text())
    if not args.full_only:
        cases.extend(walking_cases())
    cases = cases[args.shard :: args.shards]
    completed = {}
    if args.resume:
        for output in (args.output, args.paired_output):
            if output and (output / "runs.jsonl").exists():
                for line in (output / "runs.jsonl").read_text().splitlines():
                    record = json.loads(line)
                    completed[(str(output), record["case"], record["repeat"])] = record
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    python_paths = [
        args.gtsam_python,
        ROOT / "examples/g1_mjlab",
        *args.walk_python_path,
    ]
    if os.environ.get("PYTHONPATH"):
        python_paths.append(Path(os.environ["PYTHONPATH"]))
    for snapshot_path, output_path in (
        (args.snapshot, args.output),
        (args.paired_snapshot, args.paired_output),
    ):
        if snapshot_path and output_path:
            output_path.mkdir(parents=True, exist_ok=True)
            metadata = dict(
                cpu=args.cpu,
                shard=args.shard,
                shards=args.shards,
                requests_sha256=hashlib.sha256(args.requests.read_bytes()).hexdigest(),
                library_sha256=hashlib.sha256(
                    (snapshot_path / "lib/libgtdynamics.so").read_bytes()
                ).hexdigest(),
                repeats=args.repeats,
                warmups=1,
                timeout_seconds=args.timeout,
                native_timeout_seconds=min(args.timeout, 30),
            )
            (output_path / f"config-{args.shard}.json").write_text(
                json.dumps(metadata, indent=2) + "\n"
            )
    for case in cases:
        for repeat in range(args.repeats + 1):
            versions = [(args.snapshot, args.output)]
            if args.paired_snapshot:
                if args.paired_output is None:
                    parser.error("--paired-output is required with --paired-snapshot")
                versions.append((args.paired_snapshot, args.paired_output))
            if repeat % 2:
                versions.reverse()
            if repeat and all(
                completed.get((str(output), case["name"], 0), {}).get("status")
                == "timeout"
                for _, output in versions
            ):
                break
            for snapshot_path, output_path in versions:
                if (str(output_path), case["name"], repeat) in completed:
                    continue
                env["LD_LIBRARY_PATH"] = str(snapshot_path / "lib")
                env["PYTHONPATH"] = ":".join(
                    map(
                        str,
                        [
                            snapshot_path / "native",
                            snapshot_path / "python",
                            *python_paths,
                        ],
                    )
                )
                folder = output_path / case["name"] / str(repeat)
                folder.mkdir(parents=True, exist_ok=True)
                if "payload" in case:
                    command = [str(snapshot_path / "bin/g1_full_dynamics")]
                    input_text = json.dumps(case["payload"]) + "\n"
                else:
                    command = [
                        sys.executable,
                        str(ROOT / "examples/g1_mjlab" / case["script"]),
                        "--output",
                        str(folder),
                        *case["extra"],
                    ]
                    input_text = None
                command = ["taskset", "-c", str(args.cpu), *command]
                started = time.perf_counter()
                with (folder / "stderr.log").open("w") as log, (
                    folder / "stdout.log"
                ).open("w") as out:
                    try:
                        proc = subprocess.run(
                            command,
                            env=env,
                            cwd=ROOT,
                            input=input_text,
                            text=True,
                            stdout=out,
                            stderr=log,
                            timeout=(
                                min(args.timeout, 30) if input_text else args.timeout
                            ),
                            check=False,
                        )
                        status = proc.returncode
                    except subprocess.TimeoutExpired:
                        status = "timeout"
                record = dict(
                    case=case["name"],
                    repeat=repeat,
                    warmup=repeat == 0,
                    status=status,
                    seconds=time.perf_counter() - started,
                    cpu=args.cpu,
                )
                completed[(str(output_path), case["name"], repeat)] = record
                with (output_path / "runs.jsonl").open("a") as ledger:
                    ledger.write(json.dumps(record) + "\n")
                print(
                    f"{snapshot_path.name} {case['name']} {repeat}: {status}, {record['seconds']:.3f}s",
                    flush=True,
                )


if __name__ == "__main__":
    main()
