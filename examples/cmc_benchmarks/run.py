#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file run.py
# @brief Run the original CMC examples headlessly with reproducible measurements.
# @author GTDynamics contributors
"""Run each case once for warmup and five times for measurement, in isolation."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def cases() -> list[dict]:
    """Enumerate active CMC entry points, including GD and retractor variants."""
    result = []
    for source in sorted((ROOT / "examples/scripts").glob("*.cpp")):
        if not source.name.startswith(("rss", "yetong")):
            continue
        methods = ["LM"]
        if source.name.startswith("rss"):
            methods.append("CM")
        if source.name.startswith(("yetong01", "yetong02", "yetong03", "yetong04")):
            methods.append("GD")
        for method in methods:
            stages = 2 if source.name.startswith(("yetong08", "yetong09")) else 1
            result.append(
                dict(
                    name=f"{source.stem}-{method}",
                    executable=source.stem,
                    method=method,
                    stages=stages,
                    variant="",
                )
            )
        if source.name.startswith("yetong08"):
            result.append(
                dict(
                    name=f"{source.stem}-retractors",
                    executable=source.stem,
                    method="LM",
                    stages=2,
                    variant="retractors",
                )
            )
            result.append(
                dict(
                    name=f"{source.stem}-basis-retractor",
                    executable=source.stem,
                    method="LM",
                    stages=1,
                    variant="basis-retractor",
                )
            )
    return result


def main() -> None:
    """Save individual logs, numerical results, and an append-only run ledger."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binaries", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--filter", default="")
    parser.add_argument(
        "--cpu",
        type=int,
        default=2,
        help="CPU affinity for reproducible TBB scheduling",
    )
    parser.add_argument("--exclude", default="")
    parser.add_argument("--paired-binaries", type=Path)
    parser.add_argument("--paired-library", type=Path)
    parser.add_argument("--paired-output", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--trial",
        type=int,
        help="Run one measured repeat after a separately completed warmup",
    )
    args = parser.parse_args()
    if args.repeats < 1 or args.timeout <= 0:
        parser.error("repeats and timeout must be positive")
    completed = {}
    if args.resume:
        for output in (args.output, args.paired_output):
            if output and (output / "runs.jsonl").exists():
                for line in (output / "runs.jsonl").read_text().splitlines():
                    record = json.loads(line)
                    completed[(str(output), record["case"], record["repeat"])] = record
    if args.trial is not None and not 1 <= args.trial <= args.repeats:
        parser.error("trial must be between 1 and repeats")
    args.output.mkdir(parents=True, exist_ok=True)
    env = dict(
        os.environ,
        LD_LIBRARY_PATH=str(args.library.resolve()),
        OMP_NUM_THREADS="1",
        OPENBLAS_NUM_THREADS="1",
    )
    inventory = [
        case
        for case in cases()
        if args.filter in case["name"]
        and (not args.exclude or args.exclude not in case["name"])
    ]
    metadata = dict(
        cases=inventory,
        repeats=args.repeats,
        warmups=1,
        timeout=args.timeout,
        platform=platform.platform(),
        head=subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        library_sha256=hashlib.sha256(
            (args.library / "libgtdynamics.so").read_bytes()
        ).hexdigest(),
    )
    (args.output / f"manifest-cpu-{args.cpu}.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    for case in inventory:
        for repeat in (
            [args.trial] if args.trial is not None else range(args.repeats + 1)
        ):
            versions = [(args.binaries, args.library, args.output)]
            if args.paired_binaries:
                if not args.paired_library or not args.paired_output:
                    parser.error("paired binaries require paired library and output")
                versions.append(
                    (args.paired_binaries, args.paired_library, args.paired_output)
                )
            if repeat % 2:
                versions.reverse()
            # A paired warmup timeout is a censored outcome, not a timing sample.
            # Repeating an already censored ten-minute solve adds no quality data.
            if repeat and all(
                completed.get((str(output), case["name"], 0), {}).get("status")
                == "timeout"
                for _, _, output in versions
            ):
                break
            for binary_path, library_path, output_path in versions:
                if (str(output_path), case["name"], repeat) in completed:
                    continue
                env["LD_LIBRARY_PATH"] = str(library_path.resolve())
                folder = (output_path / case["name"] / str(repeat)).resolve()
                folder.mkdir(parents=True, exist_ok=True)
                work = folder / "work" / "run"
                work.mkdir(parents=True, exist_ok=True)
                (folder / "data").mkdir(exist_ok=True)
                (folder / "results").mkdir(exist_ok=True)
                (
                    (folder / "models").symlink_to(
                        ROOT / "models", target_is_directory=True
                    )
                    if not (folder / "models").exists()
                    else None
                )
                command = [
                    "taskset",
                    "-c",
                    str(args.cpu),
                    str((binary_path / case["executable"]).resolve()),
                    "--benchmark",
                    case["method"],
                    "--benchmark-output",
                    str(folder / "result.jsonl"),
                    "--benchmark-stop-after",
                    str(case["stages"]),
                    "--benchmark-variant",
                    case["variant"],
                ]
                started = time.perf_counter()
                with (folder / "stdout.log").open("w") as log:
                    try:
                        status = subprocess.run(
                            command,
                            cwd=work,
                            env=env,
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            timeout=args.timeout,
                            check=False,
                        ).returncode
                    except subprocess.TimeoutExpired:
                        status = "timeout"
                record = dict(
                    case=case["name"],
                    repeat=repeat,
                    warmup=repeat == 0,
                    status=status,
                    seconds=time.perf_counter() - started,
                    cpu=args.cpu,
                    command=command,
                )
                completed[(str(output_path), case["name"], repeat)] = record
                with (output_path / "runs.jsonl").open("a") as ledger:
                    ledger.write(json.dumps(record) + "\n")
                print(
                    f"{output_path.name} {case['name']} {repeat}: {status}, {record['seconds']:.3f}s",
                    flush=True,
                )


if __name__ == "__main__":
    main()
