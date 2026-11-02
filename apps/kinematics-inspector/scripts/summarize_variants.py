#!/usr/bin/env python3
"""Summarize final errors without loading any trajectory state buffers."""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
from typing import Any


def write_variant_summary(public: Path) -> dict[str, Any]:
    """Rank saved final results by their equal-slice SE(3)-aligned pelvis RMS."""
    variants: list[dict[str, Any]] = []
    groups: dict[str, dict[str, str]] = {}
    source_indices: list[int] | None = None
    for path in sorted((public / "variants").glob("*/final.json.gz")):
        record = json.loads(gzip.decompress(path.read_bytes()))
        if source_indices is None:
            source_indices = record["sampling"]["sourceIndices"]
        elif source_indices != record["sampling"]["sourceIndices"]:
            raise ValueError("All variants must use identical source slices")
        run = record["runs"]["contact"]
        final = run["final"]
        for group in record["graph"]["factorGroups"]:
            groups[group["id"]] = {
                key: group[key] for key in ("id", "label", "factorType")
            }
        variants.append(
            {
                "id": path.parent.name,
                "upright": record["graph"]["uprightPrior"],
                "height": record["graph"]["heightPrior"],
                "imu": record["imuAiding"]["enabled"],
                "loop": record["loopClosure"]["enabled"],
                "samples": record["samples"],
                "alignedRmsMeters": record["referenceAlignment"]["rmsAfterMeters"],
                "referenceRmsMeters": record["referenceAlignment"]["rmsBeforeMeters"],
                "metrics": final["metrics"],
                "factorErrors": final["factorErrors"],
                "factorCounts": {
                    group["id"]: group["count"]
                    for group in record["graph"]["factorGroups"]
                },
                "closurePositionErrorWorld": final["closurePositionErrorWorld"],
                "iterations": run["solver"]["iterations"],
                "elapsedSeconds": run["solver"]["elapsedSeconds"],
            }
        )
    if len(variants) != 16 or len({row["samples"] for row in variants}) != 1:
        raise ValueError("Comparison requires all 16 variants on the same slices")
    variants.sort(key=lambda row: (row["alignedRmsMeters"], row["id"]))
    summary = {
        "schemaVersion": 1,
        "bestVariant": variants[0]["id"],
        "rankBy": "SE(3)-aligned pelvis-position RMS over all retained slices",
        "factorGroups": list(groups.values()),
        "variants": variants,
    }
    (public / "comparison.json").write_text(
        json.dumps(summary, separators=(",", ":"), allow_nan=False) + "\n"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--public", type=Path, default=Path(__file__).resolve().parents[1] / "public"
    )
    args = parser.parse_args()
    summary = write_variant_summary(args.public)
    print(f"Best aligned pelvis RMS: {summary['bestVariant']}")


if __name__ == "__main__":
    main()
