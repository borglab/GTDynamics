#!/usr/bin/env python3
"""Download a physical G1 recording and prepare a walking excerpt for Kinematics.

Keep the original sensor files in the ignored cache. Resample poses and encoders
for the kinematic solve; retain the pelvis IMU at its original timestamps.
No IMU samples or derived body streams are used as optimization factors here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any
import urllib.request

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation, Slerp
from scipy.ndimage import percentile_filter

REPO = Path(__file__).resolve().parents[3]
APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
sys.path.insert(0, str(REPO / "apps/contact-initialization/scripts"))
from mujoco_io import load_model
from export_g1_moves import detect_support
from contact_discovery import component_count, sample_frames

CLIP = "10871_humanoid_20260520_223451"
REVISION = "db621c57833e08a812b2ec8491aa0ca81f0ecb33"
DATASET = "https://huggingface.co/datasets/Open-IO/humanoid-g1-odometry"
CHECKSUMS = {
    "imus.npy": "6aa3b901a081346cb6874c4ea31f885cac94820431ff51b2d7f03c67120764be",
    "joints.json": "9a0bd4014d941da9efa21aec442874921149683f22b495857c5544c88570b904",
    "joints.npy": "2f0784ebea10c8a9cc2b0827e4bd80015bb2b235fc14ec34c289053bd237a154",
    "pose.npy": "8bea8cc0bce5e460c98e1be382ac202cfb5b2056291c1d1aa09637db663f6170",
    "spec.json": "8fcb27d07c0f6d5eb511bb0109ea535d973b40e4bea0a142aa0d3294b0b74de2",
    "time_ns.npy": "612be31f2e452d7328ddb2fcbcf41c01defd3edd8dc5b6c3409bac71d3edcb83",
}
# Unitree SDK JointIndex, checked against the dataset's derived foot poses below.
# https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/dds_wrapper/robots/g1/defines.h
JOINT_NAMES = (
    [f"{s}_{j}_joint" for s in ("left", "right")
     for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
    + [f"waist_{j}_joint" for j in ("yaw", "roll", "pitch")]
    + [f"{s}_{j}_joint" for s in ("left", "right")
       for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                 "wrist_roll", "wrist_pitch", "wrist_yaw")]
)


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, separators=(",", ":"), allow_nan=False) + "\n")


def download(directory: Path) -> None:
    """Fetch pinned files atomically; validate existing cached files as well."""
    directory.mkdir(parents=True, exist_ok=True)
    for name, checksum in CHECKSUMS.items():
        target = directory / name
        if not target.exists():
            print(f"Downloading {CLIP}/{name}", flush=True)
            with urllib.request.urlopen(
                f"{DATASET}/resolve/{REVISION}/{CLIP}/{name}", timeout=120
            ) as response:
                content = response.read()
            if hashlib.sha256(content).hexdigest() != checksum:
                raise ValueError(f"Checksum mismatch: {name}")
            temporary = target.with_suffix(target.suffix + ".tmp")
            temporary.write_bytes(content)
            temporary.replace(target)
        if hashlib.sha256(target.read_bytes()).hexdigest() != checksum:
            raise ValueError(f"Checksum mismatch: {target}")


def root_positions(
    imu_positions: NDArray[np.float64], rotations: Rotation,
    imu_offset: NDArray[np.float64],
) -> NDArray[np.float64]:
    """The dataset pelvis pose is at the IMU site, not the MuJoCo root origin."""
    return imu_positions - rotations.apply(imu_offset)


def validate_mapping(
    model: mujoco.MjModel, poses: NDArray[Any], joints: NDArray[Any],
    part_names: list[str], addresses: list[int], imu_offset: NDArray[np.float64],
) -> dict[str, float]:
    """Check joint order AND the IMU-to-root transform against both recorded feet."""
    data = mujoco.MjData(model)
    max_position, max_rotation = 0.0, 0.0
    for k in np.linspace(0, len(joints) - 1, 257, dtype=int):
        rotation = Rotation.from_quat(poses[0, k, 3:])
        data.qpos[:3] = root_positions(poses[0, k, :3], rotation, imu_offset)
        data.qpos[3:7] = rotation.as_quat()[[3, 0, 1, 2]]
        data.qpos[addresses] = joints[k]
        mujoco.mj_kinematics(model, data)
        for side in ("left", "right"):
            body = model.body(f"robot/{side}_ankle_roll_link").id
            reference = poses[part_names.index(f"{side}_foot"), k]
            max_position = max(max_position, float(np.linalg.norm(data.xpos[body] - reference[:3])))
            actual = Rotation.from_matrix(data.xmat[body].reshape(3, 3))
            max_rotation = max(max_rotation, float((actual.inv() * Rotation.from_quat(reference[3:])).magnitude()))
    if max_position > 1e-4 or max_rotation > 1e-4:
        raise ValueError(f"Dataset/model mapping mismatch: {max_position=} {max_rotation=}")
    return {"maxFootPositionErrorMeters": max_position, "maxFootRotationErrorRadians": max_rotation}


def resample_qpos(
    model: mujoco.MjModel, times: NDArray[np.float64], target: NDArray[np.float64],
    poses: NDArray[Any], angles: NDArray[Any], addresses: list[int],
) -> NDArray[np.float64]:
    rotation = Slerp(times, Rotation.from_quat(poses[0, :, 3:]))(target)
    position = np.column_stack([np.interp(target, times, poses[0, :, i]) for i in range(3)])
    qpos = np.tile(model.qpos0, (len(target), 1))
    qpos[:, :3] = root_positions(position, rotation, model.site("robot/imu_in_pelvis").pos)
    qpos[:, 3:7] = rotation.as_quat()[:, [3, 0, 1, 2]]
    qpos[:, addresses] = np.column_stack([np.interp(target, times, a) for a in angles.T])
    return qpos


def support_intervals(support: NDArray[np.bool_]) -> list[dict[str, Any]]:
    """Express a sampled mask as inclusive stance intervals, without filling gaps."""
    contacts = []
    for side, foot in enumerate(("left", "right")):
        edges = np.diff(np.r_[False, support[:, side], False].astype(int))
        for ordinal, (first, last) in enumerate(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))):
            contacts.append(
                {
                    "id": f"{foot[0].upper()}{ordinal:03d}",
                    "foot": foot,
                    "start": int(first),
                    "end": int(last - 1),
                    "endExclusive": int(last),
                }
            )
    return sorted(contacts, key=lambda c: (c["start"], c["foot"]))


def stationary_imu_calibration(
    times: NDArray[np.float64], imus: NDArray[Any]
) -> dict[str, Any]:
    """Average the physical pelvis IMU over the verified initial rest, 0–25 s.

    The same calibration accompanies every excerpt from this clip, including
    the turn at 204 s. Gyroscope measurements at rest directly observe bias.
    Accelerometer means include gravity and are recorded only as diagnostics.
    """
    stationary = (times >= 0.0) & (times < 25.0)
    gyro = imus[0, stationary, 3:6].astype(float)
    acceleration = imus[0, stationary, :3].astype(float)
    return {
        "sourceWindowSeconds": [0.0, 25.0],
        "sampleCount": int(stationary.sum()),
        "frame": "pelvis IMU measurement axes",
        "gyroMeanRadiansPerSecond": gyro.mean(axis=0).tolist(),
        "gyroSampleStdRadiansPerSecond": gyro.std(axis=0, ddof=1).tolist(),
        "gyroPriorSigmaRadiansPerSecond": 0.0001,
        "meanAccelerationMetersPerSecondSquared": acceleration.mean(axis=0).tolist(),
        "description": "Manually verified initial stationary interval; raw gyroscope sample mean. Accelerometer mean includes gravity and is not an accelerometer bias estimate.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=APP / ".cache/openio/source")
    parser.add_argument("--output", type=Path, default=APP / ".cache/openio")
    parser.add_argument("--model-directory", type=Path, default=REPO / "examples/g1_mjlab/results")
    parser.add_argument("--start-seconds", type=float, default=204.0)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--full-trajectory", action="store_true", help="Use the entire source duration; pass --fps 5 for the overview")
    args = parser.parse_args()
    if not (args.start_seconds >= 0 and args.seconds > 0 and 0 < args.fps <= 200):
        parser.error("Require nonnegative start, positive duration, and 0 < fps <= 200")
    source = args.cache / CLIP
    download(source)
    arrays = {name: np.load(source / f"{name}.npy", allow_pickle=False)
              for name in ("time_ns", "imus", "pose", "joints")}
    spec = json.loads((source / "spec.json").read_text())
    channels = json.loads((source / "joints.json").read_text())["channels"]
    stamps = arrays["time_ns"]
    times = (stamps - stamps[0]).astype(np.float64) * 1e-9
    if not all(np.isfinite(a).all() for a in arrays.values()) or not (np.diff(times) > 0).all():
        raise ValueError("Expected finite measurements with increasing timestamps")
    if spec["names"][0] != "pelvis" or arrays["joints"].shape != (len(times), 29, 3):
        raise ValueError("Unexpected pelvis index or joint shape")
    if args.full_trajectory:
        args.start_seconds = 0.0
        args.seconds = float(times[-1])
    end = args.start_seconds + args.seconds
    if end > times[-1] or args.seconds * args.fps < 2:
        raise ValueError("Requested window exceeds recording or has fewer than two samples")
    target = args.start_seconds + np.arange(0, args.seconds, 1 / args.fps)
    model = load_model(args.model_directory)
    addresses = [int(model.jnt_qposadr[model.joint(f"robot/{name}").id]) for name in JOINT_NAMES]
    imu_offset = model.site("robot/imu_in_pelvis").pos.copy()
    angles = arrays["joints"][:, :, channels["q"][0]]
    validation = validate_mapping(model, arrays["pose"], angles, spec["names"], addresses, imu_offset)
    qpos = resample_qpos(model, times, target, arrays["pose"], angles, addresses)

    description = json.loads((REPO / "apps/contact-initialization/public/g1-walk.json").read_text())["model"]
    bodies = [model.body("robot/" + name).id for name in description["bodyNames"]]
    feet = {side: model.body(f"robot/{side}_ankle_roll_link").id for side in ("left", "right")}
    sole = np.asarray(description["soleOutline"])
    world, local = mujoco.MjData(model), mujoco.MjData(model)
    world_soles = []
    samples = []
    for k, row in enumerate(qpos):
        world.qpos[:] = row
        mujoco.mj_kinematics(model, world)
        world_soles.append([sole @ world.xmat[b].reshape(3, 3).T + world.xpos[b] for b in feet.values()])
        local.qpos[:] = row
        local.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        mujoco.mj_kinematics(model, local)
        samples.append({
            "index": k, "t": k / args.fps,
            "jointAngles": [float(row[model.jnt_qposadr[model.joint('robot/' + name).id]]) for name in description["jointNames"]],
            "bodyPoints": local.xpos[bodies].tolist(),
            "feet": {side: {"p": local.xpos[b].tolist(), "q": local.xquat[b].tolist()} for side, b in feet.items()},
        })
    soles = np.asarray(world_soles)
    heights = soles[:, :, :, 2].min(axis=2)
    speeds = np.linalg.norm(np.gradient(soles, 1 / args.fps, axis=0)[:, :, :, :2], axis=3).min(axis=2)
    floor = float(np.percentile(heights.min(axis=1), 5))
    _, support, contacts, handovers = detect_support(heights, speeds, floor, .04, .06, .5, 1., 2, 3)
    if args.full_trajectory:
        # Detect at 50 Hz: 5 Hz is too coarse for sole velocities and touchdown.
        # A local floor tracks reference vertical drift for LABELING only; it
        # never modifies the reference trajectory or the solver's ground plane.
        detection_times = np.arange(0, times[-1], .02)
        detection_qpos = resample_qpos(model, times, detection_times, arrays["pose"], angles, addresses)
        dense_soles = []
        for row in detection_qpos:
            world.qpos[:] = row
            mujoco.mj_kinematics(model, world)
            dense_soles.append([sole @ world.xmat[b].reshape(3, 3).T + world.xpos[b] for b in feet.values()])
        dense_soles = np.asarray(dense_soles)
        dense_heights = dense_soles[:, :, :, 2].min(axis=2)
        dense_speeds = np.linalg.norm(np.gradient(dense_soles, .02, axis=0)[:, :, :, :2], axis=3).min(axis=2)
        local_floor = percentile_filter(dense_heights.min(axis=1), percentile=5, size=51, mode="nearest")
        _, dense_support, _, handovers = detect_support(dense_heights - local_floor[:, None], dense_speeds, 0., .04, .06, .5, 1., 2, 3)
        support = dense_support[np.minimum(np.rint(target * 50).astype(int), len(dense_support) - 1)]
        contacts = support_intervals(support)
        floor = float(local_floor[0])
    for k, sample in enumerate(samples):
        sample["support"] = {side: bool(support[k, i]) for i, side in enumerate(feet)}
    frames, sampling = sample_frames(samples, contacts)
    components = component_count(frames)
    if not args.full_trajectory and (not support.any(axis=1).all() or components != 1):
        raise ValueError("Choose a walking window with continuous connected support")
    translation = -np.array([qpos[0, 0], qpos[0, 1], floor])
    qpos[:, :3] += translation
    first, last = np.searchsorted(times, [args.start_seconds, end])
    if args.full_trajectory:
        last = len(times)
    changes = (
        f"Selected seconds {args.start_seconds:g}–{end:g}; resampled encoders/poses to {args.fps:g} Hz "
        "using linear interpolation and quaternion SLERP; converted the pelvis IMU origin to the robot root; "
        "applied one constant world translation to place the start near the origin and estimated floor at z=0; "
        "inferred contact phases from FK sole height/speed; computed initialization and smoothing iterations. "
        "Pelvis accelerometer and gyro samples retain their original values and timestamps."
    )
    metadata = {
        "title": f"G1 · {CLIP} · {args.start_seconds:g}–{end:g} s",
        "sequenceId": CLIP, "datasetName": "Open-IO G1 Inertial Odometry",
        "motionKind": "physical robot recording", "contactEvidence": "kinematic",
        "sourceUrl": f"{DATASET}/tree/{REVISION}/{CLIP}", "datasetUrl": DATASET,
        "sourceRevision": REVISION, "sourceChecksums": CHECKSUMS,
        "sourceSampleCount": len(times), "sourceDurationSeconds": float(times[-1]),
        "sourceSampleRateHz": 200, "sampleCount": len(target), "sampleRateHz": args.fps,
        "stationaryImuCalibration": stationary_imu_calibration(times, arrays["imus"]),
        "samplePeriodSeconds": 1 / args.fps,
        "fullTrajectory": args.full_trajectory,
        "contactInitialization": {"addBetweenFactors": components > 1,
                                  "betweenTranslationSigma": 10.0, "betweenRotationSigma": float(np.pi)},
        "sourceWindow": {"startSeconds": args.start_seconds, "endExclusiveSeconds": end,
                         "startSample": int(first), "endExclusiveSample": int(last)},
        "referenceLabel": "SuperOdom reference",
        "referenceDescription": "SuperOdom LiDAR-inertial reference root trajectory with measured-joint FK; not independent motion-capture ground truth.",
        "worldTranslationMeters": translation.tolist(), "pelvisImuOffsetMeters": imu_offset.tolist(),
        "jointNames": JOINT_NAMES, "mappingValidation": validation,
        "detector": {"floorHeightMeters": floor, "onHeightMeters": .04, "offHeightMeters": .06,
                     "onSpeedMetersPerSecond": .5, "offSpeedMetersPerSecond": 1.,
                     "bridgeSamples": 2, "minimumSupportSamples": 3, "handoverCorrections": handovers,
                     "detectionRateHz": 50 if args.full_trajectory else args.fps,
                     "floorEstimate": "rolling 1.02 s fifth percentile for labels only" if args.full_trajectory else "window fifth percentile",
                     "unsupportedSamples": int((~support.any(axis=1)).sum()), "contactComponents": components},
        "imu": {"file": "imu.json", "samples": int(last - first), "physicalPart": "pelvis",
                "frame": "pelvis", "nominalRateHz": 200, "usedBySmoother": False,
                "description": "Measured specific force (m/s²) and angular rate (rad/s), without bias correction. Other body-part IMU-format streams are derived and excluded."},
        "licenseNotes": [{"name": "Open-IO G1 Inertial Odometry", "license": "CC-BY-4.0",
                          "url": DATASET, "licenseUrl": "https://creativecommons.org/licenses/by/4.0/",
                          "creator": "Yuheng Qiu and the SSIO authors / Open-IO",
                          "changes": changes, "localNotice": "OPENIO_ATTRIBUTION.txt"}],
    }
    args.output.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveModel(model, str(args.output / "model.mjb"))
    # Include bracketing measurements for exact interval integration, while the
    # public IMU download still contains only samples inside the selected window.
    imu_first = max(0, int(np.searchsorted(times, target[0], side="right")) - 1)
    imu_last = min(len(times), int(np.searchsorted(times, target[-1], side="left")) + 1)
    np.savez_compressed(
        args.output / "walk.npz",
        qpos=qpos,
        imu_time=times[imu_first:imu_last] - args.start_seconds,
        imu_acceleration=arrays["imus"][0, imu_first:imu_last, :3].astype(float),
        imu_angular_rate=arrays["imus"][0, imu_first:imu_last, 3:6].astype(float),
    )
    write_json(args.output / "walk.json", {"policy_dt": 1 / args.fps, "command_speed_m_s": 0.0, "motionSource": metadata})
    write_json(args.output / "contacts.json", {
        "schemaVersion": 1, "metadata": metadata, "model": description, "samples": samples,
        "contacts": contacts, "frames": frames, "frameSampling": sampling,
        "reference": {"poses": [{"p": row[:3].tolist(), "q": row[3:7].tolist()} for row in qpos]},
    })
    write_json(args.output / "imu.json", {
        "clip": CLIP, "sourceRevision": REVISION, "frame": "pelvis",
        "timeOriginNs": str(int(stamps[first])),
        "timeOffsetNs": (stamps[first:last] - stamps[first]).tolist(),
        "timeSeconds": (times[first:last] - args.start_seconds).tolist(),
        "accelerometerMetersPerSecondSquared": arrays["imus"][0, first:last, :3].astype(float).tolist(),
        "gyroscopeRadiansPerSecond": arrays["imus"][0, first:last, 3:6].astype(float).tolist(),
    })
    (args.output / "OPENIO_ATTRIBUTION.txt").write_text(
        f"Unitree G1 Inertial Odometry Dataset\n{DATASET}\nClip: {CLIP}\nRevision: {REVISION}\n\n"
        "Credit: Yuheng Qiu, Yutian Chen, Carlton Cort, Weison Huang, Shibo Zhao, Pieter Abbeel, "
        "Rocky Duan, Koushil Sreenath, C. Karen Liu, Carmelo Sferrazza, Junyi Geng, Sebastian Scherer, "
        "and Guanya Shi. SSIO: Self-Supervised Inertial Odometry (2026).\n"
        "https://arxiv.org/abs/2610.25988\nhttps://ssio-imu.github.io/\n\n"
        "License: Creative Commons Attribution 4.0 International (CC BY 4.0)\n"
        "https://creativecommons.org/licenses/by/4.0/\n\n"
        f"Modifications: {changes}\n\nReference: SuperOdom LiDAR-inertial odometry. "
        "Contacts are heuristic labels, not force measurements. Original IMU samples are retained; recording metadata identifies whether IMU aiding was enabled.\n\n"
        "Source SHA-256 checksums:\n" + "\n".join(f"{value}  {name}" for name, value in CHECKSUMS.items()) + "\n"
    )
    print(json.dumps({"samples": len(target), "imuSamples": int(last - first), "contacts": len(contacts),
                      "sourceDurationSeconds": float(times[-1]), "mappingValidation": validation}, indent=2))


if __name__ == "__main__":
    main()
