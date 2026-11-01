#!/usr/bin/env python3
"""Package existing offline results for staged browser loading, without solving.

The smoothing bundle retains every smoothing iterate and the reference. Full
initializer histories remain in the original archive and are loaded on demand.
Integer coordinates are copied exactly; this does not requantize any poses.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

APP = Path(__file__).resolve().parents[1]


def quantized_chunks(
    encoded: bytes, storage: dict[str, Any]
) -> dict[int, NDArray[np.int32]]:
    """Decode delta-shuffled integers, indexed by their original float offset."""
    if storage["codec"] != "delta-shuffle-int32-v1" or storage["scale"] != 1000000:
        raise ValueError("Unsupported state codec")
    raw = np.frombuffer(gzip.decompress(encoded), dtype=np.uint8)
    if raw.size % 4 or sum(storage["blocks"]) != raw.size // 4:
        raise ValueError("Truncated state buffer")
    integers = raw.reshape(4, -1).T.copy().view("<i4").ravel()
    chunks: dict[int, NDArray[np.int32]] = {}
    previous: dict[int, NDArray[np.int32]] = {}
    offset = 0
    for size in storage["blocks"]:
        current = integers[offset : offset + size]
        if size in previous:
            current += previous[size]
        previous[size] = current
        chunks[offset] = current
        offset += size
    return chunks


def encode_chunks(chunks: list[NDArray[np.int32]]) -> bytes:
    """Encode existing integer blocks without changing their precision."""
    previous: dict[int, NDArray[np.int32]] = {}
    encoded = []
    for chunk in chunks:
        encoded.append(chunk - previous.get(len(chunk), 0))
        previous[len(chunk)] = chunk
    shuffled = np.concatenate(encoded).astype("<i4").view(np.uint8).reshape(-1, 4).T
    return gzip.compress(shuffled.tobytes(), mtime=0)


def position_plots(
    record: dict[str, Any], chunks: dict[int, NDArray[np.int32]], stage: str = "final"
) -> dict[str, Any]:
    """Extract world pelvis and pelvis-frame ankle origins from saved MJ poses.

    Coordinates are metres. Each estimate/reference uses its own pelvis frame;
    body quaternions are normalized xyzw. This only transforms saved display
    coordinates, quantized at 1 micrometre; no FK or optimization is repeated.
    """
    names = record["bodyNames"]
    pelvis = names.index("robot/pelvis")
    feet = {
        side: names.index(f"robot/{side}_ankle_roll_link") for side in ("left", "right")
    }
    series: dict[str, Any] = {
        "pelvisWorld": {},
        "feetBody": {side: {} for side in feet},
        "pelvisBody": pelvis,
        "footBodies": feet,
        "units": "metres",
        "footFrame": "pelvis body frame of the respective estimate/reference",
    }
    for label, state in (
        ("estimate", record["runs"]["contact"][stage]),
        ("reference", record["reference"]),
    ):
        poses = (
            chunks[state["poseOffset"]].reshape(record["samples"], record["bodies"], 7)
            / record["storage"]["scale"]
        )
        root = poses[:, pelvis]
        series["pelvisWorld"][label] = root[:, :3].round(6).tolist()
        quaternion = root[:, 3:]
        quaternion /= np.linalg.norm(quaternion, axis=1, keepdims=True)
        inverse_vector = -quaternion[:, :3]
        for side, body in feet.items():
            relative = poses[:, body, :3] - root[:, :3]
            cross = 2 * np.cross(inverse_vector, relative)
            local = (
                relative + quaternion[:, 3:] * cross + np.cross(inverse_vector, cross)
            )
            series["feetBody"][side][label] = local.round(6).tolist()
    return series


def write_snapshot_bundle(
    output: Path,
    source: dict[str, Any],
    chunks: dict[int, NDArray[np.int32]],
    stage: str,
    state: dict[str, Any],
) -> None:
    """Pack one estimate plus reference so the other estimate loads only on demand."""
    record = deepcopy(source)
    state = deepcopy(state)
    state.pop("frameMetrics", None)
    state["label"] = "Initial estimate" if stage == "initial" else "Final CM-Opt result"
    record["runs"] = {
        "contact": {"solver": source["runs"]["contact"]["solver"], stage: state}
    }
    record["availableStages"] = [stage]
    record["positionPlots"] = position_plots(record, chunks, stage)
    selected: list[NDArray[np.int32]] = []
    offset = 0
    for snapshot in (record["reference"], state):
        for field in (
            "poseOffset",
            "jointOffset",
            "velocityOffset",
            "contactSlipOffset",
        ):
            if field in snapshot:
                chunk = chunks[snapshot[field]]
                snapshot[field] = offset
                selected.append(chunk)
                offset += len(chunk)
    record["storage"].update(
        states=f"{stage}.bin.gz", blocks=[len(chunk) for chunk in selected]
    )
    record["modelFingerprint"] = hashlib.sha256(
        (output / record["storage"]["model"]).read_bytes()
    ).hexdigest()
    record.setdefault("replayPackaging", {}).update(
        scriptSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        operation=f"Retain {stage}/reference poses exactly; extract position plots offline",
    )
    record["notes"] = (
        "Initial and final estimates are stored separately and loaded on demand. "
        "No intermediate iterates are retained. The initial estimate is the contact "
        "initializer output before CM-Opt, with finite-difference velocities and "
        "the bias prior mean when IMU aiding is enabled. Position plots use saved "
        "MuJoCo body origins; feet use each trajectory's own pelvis frame."
    )
    (output / f"{stage}.bin.gz").write_bytes(encode_chunks(selected))
    encoded = json.dumps(record, separators=(",", ":"), allow_nan=False).encode()
    (output / f"{stage}.json.gz").write_bytes(gzip.compress(encoded, mtime=0))
    if stage == "final":
        (output / "recording.json").write_bytes(encoded + b"\n")
        (output / "recording.json.gz").write_bytes(gzip.compress(encoded, mtime=0))


def write_final_bundle(output: Path) -> None:
    """Replace histories with independently loadable initial/final snapshots.

    Also supports repacking existing final-only archives without inventing an
    initial estimate. A fresh export is needed to obtain that missing state.
    """
    metadata = (output / "recording.json").read_bytes()
    record = json.loads(metadata)
    if set(record["runs"]) != {"contact"}:
        raise ValueError("Snapshot export requires a contact-only recording")
    chunks = quantized_chunks(
        (output / record["storage"]["states"]).read_bytes(), record["storage"]
    )
    record.setdefault("replayPackaging", {}).setdefault(
        "sourceMetadataSha256", hashlib.sha256(metadata).hexdigest()
    )
    run = record["runs"]["contact"]
    if "smoothing" in run:
        write_snapshot_bundle(output, record, chunks, "initial", run["smoothing"][0])
    elif (output / "initial.json.gz").exists():
        # Asset sharing moves model.json.gz to the app root after export.
        initial = json.loads(gzip.decompress((output / "initial.json.gz").read_bytes()))
        initial["storage"]["model"] = record["storage"]["model"]
        encoded = json.dumps(initial, separators=(",", ":"), allow_nan=False).encode()
        (output / "initial.json.gz").write_bytes(gzip.compress(encoded, mtime=0))
    final = run["final"] if "final" in run else run["smoothing"][-1]
    write_snapshot_bundle(output, record, chunks, "final", final)
    for name in ("states.bin.gz", "smoothing.bin.gz", "smoothing.json.gz"):
        (output / name).unlink(missing_ok=True)
    print(f"Packed trajectory snapshots: {output}", flush=True)


def write_replay_bundles(output: Path) -> None:
    """Write compressed metadata and a smaller, independently decodable bundle."""
    record = json.loads((output / "recording.json").read_text())
    if record.get("availableStages") == ["final"]:
        write_final_bundle(output)
        return
    chunks = quantized_chunks(
        (output / record["storage"]["states"]).read_bytes(), record["storage"]
    )
    model_fingerprint = hashlib.sha256(
        (output / record["storage"]["model"]).read_bytes()
    ).hexdigest()
    full = {
        **record,
        "availableStages": record.get("availableStages", ["smoothing", "initializer"]),
        "modelFingerprint": model_fingerprint,
    }
    if full["availableStages"] == ["smoothing"]:
        encoded = json.dumps(full, separators=(",", ":"), allow_nan=False).encode()
        for name in ("recording.json.gz", "smoothing.json.gz"):
            (output / name).write_bytes(gzip.compress(encoded, mtime=0))
        print(f"Packed smoothing replay: {output}", flush=True)
        return
    packed = {
        **full,
        "availableStages": ["smoothing"],
        "reference": dict(record["reference"]),
        "runs": {
            method: {
                **run,
                "smoothing": deepcopy(run["smoothing"]),
                "initializer": [],
                "ikFrames": [],
            }
            for method, run in record["runs"].items()
        },
    }
    selected: list[NDArray[np.int32]] = []
    offsets: dict[int, int] = {}
    offset = 0
    states = [packed["reference"]]
    for run in packed["runs"].values():
        states.extend(run["smoothing"])
    for state in states:
        for field in (
            "poseOffset",
            "jointOffset",
            "velocityOffset",
            "contactSlipOffset",
        ):
            if field not in state:
                continue
            original = state[field]
            if original not in offsets:
                offsets[original] = offset
                selected.append(chunks[original])
                offset += len(chunks[original])
            state[field] = offsets[original]
    packed["storage"] = {
        **record["storage"],
        "states": "smoothing.bin.gz",
        "blocks": [len(chunk) for chunk in selected],
    }
    (output / "smoothing.bin.gz").write_bytes(encode_chunks(selected))
    for name, value in (("recording.json.gz", full), ("smoothing.json.gz", packed)):
        encoded = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
        (output / name).write_bytes(gzip.compress(encoded, mtime=0))
    print(f"Packed smoothing replay: {output}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directories", type=Path, nargs="*")
    parser.add_argument("--final-only", action="store_true")
    args = parser.parse_args()
    directories = args.directories or sorted(
        path.parent for path in (APP / "public").rglob("recording.json")
    )
    for directory in directories:
        if args.final_only:
            write_final_bundle(directory)
        else:
            write_replay_bundles(directory)


if __name__ == "__main__":
    main()
