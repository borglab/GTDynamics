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


def write_replay_bundles(output: Path) -> None:
    """Write compressed metadata and a smaller, independently decodable bundle."""
    record = json.loads((output / "recording.json").read_text())
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
    args = parser.parse_args()
    directories = args.directories or sorted(
        path.parent for path in (APP / "public").rglob("recording.json")
    )
    for directory in directories:
        write_replay_bundles(directory)


if __name__ == "__main__":
    main()
