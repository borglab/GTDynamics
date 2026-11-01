#!/usr/bin/env python3
"""Regenerate the Contact vs IK snippets and contact-only Kinematics experiments.

All initialization, CM-Opt iterations, factor errors, and slip locations are
computed offline. This driver calls the existing example and importer through
their command-line entry points; it is not a timing harness.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import subprocess
import sys

from pack_recordings import read_metadata, write_replay_bundles

REPO = Path(__file__).resolve().parents[3]
APP = Path(__file__).resolve().parents[1]
COMPARISON = APP.parent / "contact-vs-ik"
SNIPPETS = [
    {"id": "walk", "label": "Walking · 40–50 s", "start": 40},
    {"id": "corner", "label": "Corner · 100–110 s", "start": 100},
    {"id": "turn", "label": "Turn · 204–214 s", "start": 204},
]


def run(arguments: list[str], log: Path) -> None:
    """Run one native export with its own log and report failures with context."""
    with log.open("w") as stream:
        result = subprocess.run(
            [sys.executable, *arguments], cwd=REPO, stdout=stream, stderr=stream
        )
    if result.returncode:
        raise RuntimeError(f"Export failed: {log}\n{log.read_text()[-6000:]}")
    print(f"Completed {log.stem}", flush=True)


def share_assets(output: Path, public: Path) -> None:
    """Share pinned geometry across app variants; publish only replay inputs."""
    path = output / "recording.json"
    record = read_metadata(output)
    source = output / record["storage"]["model"]
    target = public / "model.json.gz"
    if source.resolve() != target.resolve():
        data = source.read_bytes()
        if target.exists() and target.read_bytes() != data:
            raise ValueError("App variants must use the same pinned robot model")
        target.write_bytes(data)
        source.unlink()
    record["storage"]["model"] = os.path.relpath(target, output)
    path.write_text(json.dumps(record, separators=(",", ":")) + "\n")
    write_replay_bundles(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--only", choices=["all", "snippets", "trajectory"], default="all"
    )
    parser.add_argument("--jobs", type=int, default=3)
    args = parser.parse_args()
    logs = APP / ".cache/split-logs"
    logs.mkdir(parents=True, exist_ok=True)
    importer = str(APP / "scripts/import_openio.py")
    exporter = str(APP / "scripts/export_recording.py")
    jobs: list[tuple[list[str], Path]] = []
    if args.only in ("all", "snippets"):
        for snippet in SNIPPETS:
            directory = APP / ".cache/segments" / snippet["id"]
            run(
                [
                    importer,
                    "--start-seconds",
                    str(snippet["start"]),
                    "--seconds",
                    "10",
                    "--fps",
                    "50",
                    "--allow-disconnected",
                    "--local-floor",
                    "--output",
                    str(directory),
                ],
                logs / f"import-{snippet['id']}.log",
            )
            for imu in (False, True):
                output = COMPARISON / "public/snippets" / snippet["id"]
                if imu:
                    output /= "imu"
                command = [
                    exporter,
                    "--data",
                    str(directory),
                    "--contacts",
                    str(directory / "contacts.json"),
                    "--output",
                    str(output),
                ]
                if imu:
                    command.append("--imu")
                jobs.append(
                    (
                        command,
                        logs
                        / f"snippet-{snippet['id']}-{'imu' if imu else 'plain'}.log",
                    )
                )
    if args.only in ("all", "trajectory"):
        directory = APP / ".cache/openio-contact-source"
        run(
            [
                importer,
                "--full-trajectory",
                "--fps",
                "50",
                "--minimum-support-samples",
                "4",
                "--output",
                str(directory),
            ],
            logs / "import-full.log",
        )
        for sigma in (50, 20, 5):
            for imu in (False, True):
                for closure in (False, True):
                    name = f"{sigma}{'-imu' if imu else ''}{'-loop' if closure else ''}"
                    output = APP / "public/variants" / name
                    command = [
                        exporter,
                        "--data",
                        str(directory),
                        "--contacts",
                        str(directory / "contacts.json"),
                        "--output",
                        str(output),
                        "--methods",
                        "contact",
                        "--smoothing-only",
                        "--final-only",
                        "--contact-slices",
                        "--contact-slip-sigma",
                        str(sigma / 1000),
                    ]
                    if imu:
                        command.append("--imu")
                    if closure:
                        command.append("--loop-closure")
                    jobs.append((command, logs / f"trajectory-{name}.log"))
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run, command, log) for command, log in jobs]
        for future in as_completed(futures):
            future.result()
    for command, _ in jobs:
        output = Path(command[command.index("--output") + 1])
        public = (
            COMPARISON / "public"
            if output.is_relative_to(COMPARISON)
            else APP / "public"
        )
        share_assets(output, public)
    if args.only in ("all", "snippets"):
        (COMPARISON / "public/snippets.json").write_text(
            json.dumps(SNIPPETS, indent=2) + "\n"
        )


if __name__ == "__main__":
    main()
