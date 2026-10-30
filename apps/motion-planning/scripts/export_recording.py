#!/usr/bin/env python3
"""Capture an actual native solve for the serverless GitHub Pages version."""
import argparse
import json
import subprocess
from pathlib import Path


def main():
    app = Path(__file__).resolve().parents[1]
    repo = app.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--optimizer", type=Path, default=(
        repo / "build/apps/motion-planning/backend/motion_planning_optimizer"))
    parser.add_argument("--output", type=Path, default=app / "public/recording.json")
    args = parser.parse_args()
    process = subprocess.Popen([str(args.optimizer.resolve())], cwd=repo,
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               text=True, bufsize=1)
    request_id = 0

    def send(command, **fields):
        nonlocal request_id
        request_id += 1
        process.stdin.write(json.dumps(dict(command=command, request_id=request_id, **fields)) + "\n")
        process.stdin.flush()
        result = json.loads(process.stdout.readline())
        if result.get("type") == "error":
            raise RuntimeError(result.get("message", "Native optimizer failed"))
        if result.pop("request_id") != request_id:
            raise RuntimeError("Native optimizer response is out of order")
        return result

    try:
        description = send("describe")
        snapshot = send("build", intermediate_states=10, duration=4,
                        start=description["start"], goal=description["goal"])
        snapshots = [snapshot]
        for _ in range(80):
            if snapshot.get("done") or snapshot.get("converged"):
                break
            snapshot = send("iterate")
            snapshots.append(snapshot)
        if not snapshot.get("converged"):
            raise RuntimeError("The recorded solve did not converge")
        revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
        recording = {
            "schema_version": 1,
            "source_revision": revision,
            "solver": "Native GTDynamics / GTSAM Levenberg–Marquardt",
            "description": description,
            "snapshots": snapshots,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(recording, separators=(",", ":")) + "\n")
        print(f"Recorded {len(snapshots) - 1} native iterations to {args.output}")
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()


if __name__ == "__main__":
    main()
