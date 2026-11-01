#!/usr/bin/env python3
"""Download and export a continuous, CC-BY-4.0 G1 Moves stepping capture.

Uses the same 29-DoF G1 kinematics as the original PR recording. Unlike that
dynamic rollout, this is retargeted human motion with no force measurements.
Support labels are explicit height/speed heuristics using world FK. The saved
solver measurements still contain only pelvis-local FK; the recorded root is
separate and does not enter optimization.
"""

import argparse
import json
import urllib.request
from pathlib import Path

import mujoco
import numpy as np

from export_g1_contacts import digest, rounded
from contact_discovery import component_count, refine_handovers, sample_frames

REVISION = "895d064b2385725e6aabf540461c219afc3e0916"
CLIP = "J_Dance0_StepTouch"
CSV_PATH = f"dance/{CLIP}/retarget/{CLIP}.csv"
CSV_SHA256 = "d821d4ce0623242965c1f153fc8f8e0fb306294d1c084535ef27d9a3d36322a0"
CLIPS = {
    CLIP: (CSV_SHA256, "StepTouch", "g1-steptouch-30s", 30.0),
    "J_ShortDance16_JazzWalk": (
        "f8f61ea840a38da7a7c9ce468dcdf4643d47ad1e055e7292a278735e60ae1309",
        "JazzWalk",
        "g1-jazzwalk",
        637 / 60,
    ),
}
DATASET_URL = "https://huggingface.co/datasets/exptech/g1-moves"
SOURCE_URL = f"{DATASET_URL}/resolve/{REVISION}/{CSV_PATH}"
FPS = 60.0


def download(path, url=SOURCE_URL, checksum=CSV_SHA256):
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(urllib.request.urlopen(url, timeout=60).read())
    if digest(path) != checksum:
        raise ValueError(f"Source checksum mismatch: {path}")


def detect_support(
    heights, speeds, floor, on_height, off_height, on_speed, off_speed, bridge, minimum
):
    raw = np.zeros(heights.shape, dtype=bool)
    for side in range(2):
        active = False
        for i in range(len(heights)):
            active = heights[i, side] - floor <= (
                off_height if active else on_height
            ) and speeds[i, side] <= (off_speed if active else on_speed)
            raw[i, side] = active
    clean = raw.copy()
    for side in range(2):
        edges = np.diff(np.r_[True, raw[:, side], True].astype(int))
        for start, end in zip(np.flatnonzero(edges == -1), np.flatnonzero(edges == 1)):
            if start > 0 and end < len(raw) and end - start <= bridge:
                clean[start:end, side] = True
        edges = np.diff(np.r_[False, clean[:, side], False].astype(int))
        for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            if end - start < minimum:
                clean[start:end, side] = False
    clean, handovers = refine_handovers(
        clean.tolist(),
        (heights - floor).tolist(),
        speeds.tolist(),
        on_height,
        off_speed,
    )
    clean = np.asarray(clean, dtype=bool)
    contacts = []
    for side, foot in enumerate(("left", "right")):
        edges = np.diff(np.r_[False, clean[:, side], False].astype(int))
        for ordinal, (start, end) in enumerate(
            zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))
        ):
            contacts.append(
                {
                    "id": f"{foot[0].upper()}{ordinal:02d}",
                    "foot": foot,
                    "start": int(start),
                    "end": int(end - 1),
                    "endExclusive": int(end),
                    "clippedStart": bool(start == 0),
                    "clippedEnd": bool(end == len(clean)),
                }
            )
    return (
        raw,
        clean,
        sorted(contacts, key=lambda c: (c["start"], c["foot"])),
        handovers,
    )


def main():
    app = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clip", choices=CLIPS, default=CLIP)
    parser.add_argument(
        "--recording-output",
        type=Path,
        help="Also save model.mjb, walk.npz and walk.json for the kinematics example",
    )
    parser.add_argument(
        "--model", type=Path, default=app.parents[1] / "models/mujoco/unitree_g1/g1.xml"
    )
    parser.add_argument("--template", type=Path, default=app / "public/g1-walk.json")
    parser.add_argument("--cache", type=Path, default=app / ".cache/g1-moves")
    parser.add_argument(
        "--output", type=Path, default=app / "public/g1-steptouch-30s.json"
    )
    parser.add_argument("--start-seconds", type=float, default=0.0)
    parser.add_argument(
        "--seconds", type=float, help="Defaults to the clip's selected duration"
    )
    parser.add_argument("--on-height", type=float, default=0.04)
    parser.add_argument("--off-height", type=float, default=0.06)
    parser.add_argument("--on-speed", type=float, default=0.5)
    parser.add_argument("--off-speed", type=float, default=1.0)
    parser.add_argument("--bridge-samples", type=int, default=2)
    parser.add_argument("--minimum-support-samples", type=int, default=3)
    args = parser.parse_args()
    checksum, title, sequence_id, seconds = CLIPS[args.clip]
    csv_path = f"dance/{args.clip}/retarget/{args.clip}.csv"
    source_url = f"{DATASET_URL}/resolve/{REVISION}/{csv_path}"
    if args.seconds is None:
        args.seconds = seconds
    if not (
        0 <= args.on_height <= args.off_height
        and 0 <= args.on_speed <= args.off_speed
        and args.seconds > 0
        and args.start_seconds >= 0
        and args.bridge_samples >= 0
        and args.minimum_support_samples >= 1
    ):
        parser.error("Invalid duration or contact-detector settings")
    source = args.cache / f"{args.clip}-{REVISION}.csv"
    download(source, source_url, checksum)
    all_rows = np.loadtxt(source, delimiter=",")
    if all_rows.ndim != 2 or all_rows.shape[1] != 36 or not np.isfinite(all_rows).all():
        raise ValueError(
            "Expected finite G1 root XYZ, quaternion XYZW and 29-joint CSV rows"
        )
    first = round(args.start_seconds * FPS)
    last = first + round(args.seconds * FPS)
    if first < 0 or last > len(all_rows):
        raise ValueError(
            "Requested window exceeds the original continuous capture; no looping or padding is permitted"
        )
    qpos = all_rows[first:last].copy()
    qpos[:, 3:7] = qpos[:, [6, 3, 4, 5]]
    qpos[:, 3:7] /= np.linalg.norm(qpos[:, 3:7], axis=1)[:, None]
    model_description = json.loads(args.template.read_text())["model"]
    model = (
        mujoco.MjModel.from_binary_path(str(args.model))
        if args.model.suffix == ".mjb"
        else mujoco.MjModel.from_xml_path(str(args.model))
    )
    prefix = (
        "robot/"
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot/pelvis") >= 0
        else ""
    )
    bodies = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix + n)
        for n in model_description["bodyNames"]
    ]
    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i).removeprefix("robot/")
        for i in range(model.njnt)
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    if min(bodies) < 0 or joint_names != model_description["jointNames"]:
        raise ValueError(
            "The compiled model does not match the exported G1 body and joint order"
        )
    feet = {
        side: bodies[model_description["footBodies"][side]]
        for side in ("left", "right")
    }
    local, world = mujoco.MjData(model), mujoco.MjData(model)
    sole = np.array(model_description["soleOutline"])
    samples, reference, world_soles = [], [], []
    reference_feet = {side: [] for side in feet}
    for i, row in enumerate(qpos):
        local.qpos[:] = row
        local.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        mujoco.mj_kinematics(model, local)
        world.qpos[:] = row
        mujoco.mj_kinematics(model, world)
        world_soles.append(
            [
                sole @ world.xmat[b].reshape(3, 3).T + world.xpos[b]
                for b in feet.values()
            ]
        )
        samples.append(
            {
                "index": i,
                "sourceIndex": i + first,
                "t": round(i / FPS, 8),
                "jointAngles": rounded(row[7:]),
                "bodyPoints": rounded(local.xpos[bodies]),
                "feet": {
                    side: {"p": rounded(local.xpos[b]), "q": rounded(local.xquat[b], 9)}
                    for side, b in feet.items()
                },
            }
        )
        reference.append({"p": rounded(row[:3], 9), "q": rounded(row[3:7], 9)})
        for side, body in feet.items():
            reference_feet[side].append(
                (world.xpos[body].copy(), world.xquat[body].copy())
            )
    world_soles = np.array(world_soles)
    heights = world_soles[:, :, :, 2].min(axis=2)
    # A foot can rock around a slow toe or heel while its center moves. Select
    # the slowest sole corner, not the moving foot center or centroid.
    corner_speeds = np.linalg.norm(
        np.gradient(world_soles, 1 / FPS, axis=0)[:, :, :, :2], axis=3
    )
    speeds = corner_speeds.min(axis=2)
    floor = float(np.percentile(heights.min(axis=1), 5))
    raw, support, contacts, handovers = detect_support(
        heights,
        speeds,
        floor,
        args.on_height,
        args.off_height,
        args.on_speed,
        args.off_speed,
        args.bridge_samples,
        args.minimum_support_samples,
    )
    for i, sample in enumerate(samples):
        sample["support"] = {side: bool(support[i, j]) for j, side in enumerate(feet)}
        sample["contactSignals"] = {
            side: {
                "soleHeightAboveFloorMeters": round(float(heights[i, j] - floor), 6),
                "minimumSoleSpeedMetersPerSecond": round(float(speeds[i, j]), 6),
            }
            for j, side in enumerate(feet)
        }
    supported = np.flatnonzero(support.any(axis=1))
    if not len(supported):
        raise ValueError("Heuristic found no candidate support")
    frames, frame_sampling = sample_frames(samples, contacts)
    rigidity = []
    for contact in contacts:
        (p0, q0), (p1, q1) = (
            reference_feet[contact["foot"]][i]
            for i in (contact["start"], contact["end"])
        )
        rigidity.append(
            {
                "id": contact["id"],
                "endpointTranslationMeters": round(float(np.linalg.norm(p1 - p0)), 9),
                "endpointRotationDegrees": round(
                    float(np.degrees(2 * np.arccos(np.clip(abs(q0 @ q1), -1, 1)))), 6
                ),
            }
        )
    stats = {
        "episodeCount": len(contacts),
        "frameCount": len(frames),
        "leftEpisodes": sum(c["foot"] == "left" for c in contacts),
        "rightEpisodes": sum(c["foot"] == "right" for c in contacts),
        "singleSupportSamples": int(np.sum(support.sum(axis=1) == 1)),
        "doubleSupportSamples": int(np.sum(support.all(axis=1))),
        "unsupportedSamples": int(np.sum(~support.any(axis=1))),
        "graphComponents": component_count(frames),
        "observationCount": sum(len(f["contacts"]) for f in frames),
        "referenceDisplacementMeters": rounded(
            qpos[-1, :3] - qpos[int(supported[0]), :3], 9
        ),
    }
    metadata = {
        "title": f"G1 Moves · captured {title} · {len(samples) / FPS:g} s",
        "sequenceId": sequence_id,
        "motionKind": f"retargeted human {title} dance capture",
        "contactEvidence": "kinematic",
        "sourceUrl": source_url,
        "datasetUrl": DATASET_URL,
        "sourceRevision": REVISION,
        "sourceFile": csv_path,
        "sourceSha256": checksum,
        "recordingSha256": checksum,
        "modelSha256": digest(args.model),
        "sourceSampleCount": len(all_rows),
        "sourceSampleRateHz": FPS,
        "sourceDurationSeconds": len(all_rows) / FPS,
        "sourceWindow": {
            "startSample": first,
            "endExclusiveSample": last,
            "startSeconds": first / FPS,
            "endExclusiveSeconds": last / FPS,
            "description": "One continuous original capture window; no concatenation, repetition, interpolation, or synthesized motion",
        },
        "sampleCount": len(samples),
        "sampleRateHz": FPS,
        "samplePeriodSeconds": 1 / FPS,
        "recordedDurationSeconds": len(samples) / FPS,
        "timestampSpanSeconds": (len(samples) - 1) / FPS,
        "timestampConvention": "t=0 denotes first selected source sample; sourceIndex preserves the original CSV row",
        "mujocoVersion": mujoco.__version__,
        "rootQuaternionSourceOrder": "xyzw; normalized and exported as wxyz",
        "detector": {
            "type": "kinematic",
            "source": "Heuristic from root-referenced FK sole height and minimum horizontal sole-corner speed; no measured or simulated contact forces",
            "floorHeightMeters": floor,
            "floorEstimate": "5th percentile of the minimum of both foot sole heights over the selected continuous window",
            "onHeightMeters": args.on_height,
            "offHeightMeters": args.off_height,
            "onSpeedMetersPerSecond": args.on_speed,
            "offSpeedMetersPerSecond": args.off_speed,
            "bridgeSamples": args.bridge_samples,
            "minimumSupportSamples": args.minimum_support_samples,
            "bridgedSampleCount": int(np.sum(support & ~raw)) - len(handovers),
            "sampleResolutionSeconds": 1 / FPS,
            "handoverLookbackSamples": 1,
            "handoverCorrections": handovers,
            "handoverRule": "One-sample lookback at confirmed direct support transfers, with incoming height-on and speed-hold evidence; no extension across unsupported intervals",
            "endpointConvention": "Support states [start,endExclusive); graph endpoints start and end=endExclusive-1 are supported samples",
            "limitation": "A kinematic candidate-contact schedule, not force evidence. Both feet can slide or appear above the floor after retargeting. Threshold rejections may disconnect the graph; no motion edges fill those gaps. Recorded root is used only for labeling and held-out evaluation, never as a pose measurement in the solver.",
        },
        "graphSampleRange": [int(supported[0]), int(supported[-1])],
        "frameSampling": frame_sampling,
        "excludedBoundarySamples": [
            i for i in range(len(samples)) if i < supported[0] or i > supported[-1]
        ],
        "stats": stats,
        "licenseNotes": [
            {
                "name": "G1 Moves capture and retargeted motion",
                "license": "CC-BY-4.0",
                "url": f"{DATASET_URL}/blob/{REVISION}/README.md#license",
                "licenseUrl": "https://creativecommons.org/licenses/by/4.0/",
                "creator": "Mitch Chaiet / Experiential Technologies",
                "performer": "Jasmine Coro",
                "changes": f"Selected source rows {first}..{last - 1}, converted quaternion order, computed local FK, and inferred candidate contact episodes; no frames repeated",
                "localNotice": "G1_MOVES_ATTRIBUTION.txt",
            },
            (
                {
                    "name": "MJLab G1 model source",
                    "license": "Apache-2.0",
                    "url": "https://github.com/mujocolab/mjlab/blob/main/LICENSE",
                    "localNotice": "THIRD_PARTY_LICENSES.txt",
                }
                if args.model.suffix == ".mjb"
                else {
                    "name": "Unitree G1 model from MuJoCo Menagerie",
                    "license": "BSD-3-Clause",
                    "url": "https://github.com/google-deepmind/mujoco_menagerie/blob/a03e87bf13502b0b48ebbf2808928fd96ebf9cf3/unitree_g1/LICENSE",
                    "localNotice": "assets/unitree_g1/LICENSE",
                }
            ),
        ],
    }
    output = {
        "schemaVersion": 1,
        "metadata": metadata,
        "model": model_description,
        "samples": samples,
        "contacts": contacts,
        "frames": frames,
        "reference": {
            "description": "Original retargeted motion root poses; held-out overlay/evaluation only after contact schedule annotation. No reference root pose is a graph input.",
            "poses": reference,
            "rigidity": {
                "description": "Held-out ankle endpoint motion for each heuristic support episode; sliding/rocking violates stationary-patch assumptions",
                "medianEndpointTranslationMeters": float(
                    np.median([r["endpointTranslationMeters"] for r in rigidity])
                ),
                "maxEndpointTranslationMeters": max(
                    r["endpointTranslationMeters"] for r in rigidity
                ),
                "medianEndpointRotationDegrees": float(
                    np.median([r["endpointRotationDegrees"] for r in rigidity])
                ),
                "maxEndpointRotationDegrees": max(
                    r["endpointRotationDegrees"] for r in rigidity
                ),
                "episodes": rigidity,
            },
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, separators=(",", ":"), allow_nan=False) + "\n"
    )
    if args.recording_output is not None:
        directory = args.recording_output
        directory.mkdir(parents=True, exist_ok=True)
        mujoco.mj_saveModel(model, str(directory / "model.mjb"))
        np.savez_compressed(directory / "walk.npz", qpos=qpos)
        # The legacy IK path still requires a constant forward command. A dance
        # returning near its start has zero commanded speed, not reference poses.
        recording_metadata = {
            "policy_dt": 1 / FPS,
            "command_speed_m_s": 0.0,
            "motionSource": metadata,
        }
        (directory / "walk.json").write_text(
            json.dumps(recording_metadata, indent=2) + "\n"
        )
    print(json.dumps(stats, indent=2))
    print(f"Wrote {args.output} ({args.output.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
