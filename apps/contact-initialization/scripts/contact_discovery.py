"""Contact refinement and frame sampling from existing measured samples.

Refresh only the frame schedule of an exported dataset with:
  python contact_discovery.py public/g1-walk.json public/g1-steptouch-30s.json
"""

from bisect import bisect_right
import math


def sample_frames(samples, contacts, max_interval_seconds=1.0):
    """Retain contact endpoints and fill longer gaps with original samples.

    The latest sample within one interval of the preceding frame is selected
    until the next contact boundary is close enough. Contact IDs come strictly
    from the existing stance schedule, including at supplemental frames.
    """
    if not math.isfinite(max_interval_seconds) or max_interval_seconds <= 0:
        raise ValueError("Frame interval must be finite and positive")
    times = [sample["t"] for sample in samples]
    if (
        not times
        or any(not math.isfinite(t) for t in times)
        or any(b <= a for a, b in zip(times, times[1:]))
    ):
        raise ValueError("Sample timestamps must be finite and increasing")
    boundaries = sorted({c[key] for c in contacts for key in ("start", "end")})
    if not boundaries:
        raise ValueError("Frame sampling requires at least one contact")
    indices = [boundaries[0]]
    for boundary in boundaries[1:]:
        while times[boundary] - times[indices[-1]] > max_interval_seconds + 1e-8:
            index = (
                bisect_right(
                    times,
                    times[indices[-1]] + max_interval_seconds + 1e-8,
                    indices[-1] + 1,
                    boundary,
                )
                - 1
            )
            if index <= indices[-1]:
                raise ValueError("Source samples are too sparse for the frame interval")
            indices.append(index)
        indices.append(boundary)
    boundary_set = set(boundaries)
    frames = [
        {
            "index": i,
            "sampleIndex": i,
            "t": times[i],
            "selection": "contact" if i in boundary_set else "periodic",
            "starts": [c["id"] for c in contacts if c["start"] == i],
            "ends": [c["id"] for c in contacts if c["end"] == i],
            "contacts": [
                c["id"] for c in contacts if c["start"] <= i < c["endExclusive"]
            ],
        }
        for i in indices
    ]
    sampling = {
        "maxIntervalSeconds": max_interval_seconds,
        "contactFrameCount": len(boundaries),
        "periodicFrameCount": len(frames) - len(boundaries),
        "maxObservedIntervalSeconds": max(
            (b["t"] - a["t"] for a, b in zip(frames, frames[1:])), default=0
        ),
        "rule": f"Keep contact starts and final supported samples; fill longer gaps with original samples at intervals of at most {max_interval_seconds:g} seconds. Preserve the existing contact schedule.",
    }
    return frames, sampling


def component_count(frames):
    """Count bipartite connected components without adding motion factors."""
    parent = list(range(len(frames)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    seen = {}
    for i, frame in enumerate(frames):
        for contact in frame["contacts"]:
            if contact in seen:
                parent[find(i)] = find(seen[contact])
            else:
                seen[contact] = i
    return len({find(i) for i in range(len(frames))})


def refine_handovers(support, clearances, speeds, on_height, off_speed):
    """Look back one sample from a confirmed direct left/right handover.

    Acquisition hysteresis can delay a touchdown until just after the other
    foot's final supported sample. Recover that shared boundary only when the
    incoming foot already meets the height-on and speed-hold thresholds. Never
    extend across a sample where neither foot was supported.
    """
    refined = [list(row) for row in support]
    corrections = []
    for i in range(1, len(support)):
        for incoming in range(2):
            outgoing = 1 - incoming
            if (
                support[i - 1][outgoing]
                and not support[i - 1][incoming]
                and support[i][incoming]
                and not support[i][outgoing]
                and clearances[i - 1][incoming] <= on_height
                and speeds[i - 1][incoming] <= off_speed
            ):
                refined[i - 1][incoming] = True
                corrections.append(
                    {
                        "sampleIndex": i - 1,
                        "confirmedOnsetSample": i,
                        "foot": ("left", "right")[incoming],
                    }
                )
    return refined, corrections


if __name__ == "__main__":
    import argparse
    import json
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("datasets", nargs="+", type=Path)
    args = parser.parse_args()
    for path in args.datasets:
        dataset = json.loads(path.read_text())
        frames, sampling = sample_frames(dataset["samples"], dataset["contacts"])
        dataset["frames"] = frames
        dataset["metadata"]["frameSampling"] = sampling
        dataset["metadata"]["stats"].update(
            frameCount=len(frames),
            graphComponents=component_count(frames),
            observationCount=sum(len(f["contacts"]) for f in frames),
        )
        path.write_text(
            json.dumps(dataset, separators=(",", ":"), allow_nan=False) + "\n"
        )
        print(
            f"{path.name}: {len(frames)} frames, {sampling['periodicFrameCount']} periodic, "
            f"maximum gap {sampling['maxObservedIntervalSeconds']:.3f} s"
        )
