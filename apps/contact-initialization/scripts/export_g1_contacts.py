#!/usr/bin/env python3
"""Export the real G1 PR walking recording for the contact initialization app.

Requires Python with numpy and mujoco, not MJLab or a GPU. The original recorder
is examples/g1_mjlab/record_walk.py in PR #446. Example, from repository root:

  /path/to/mjlab/.venv/bin/python apps/contact-initialization/scripts/export_g1_contacts.py \
      --data examples/g1_mjlab/results --metadata /path/to/original/walk.json

No reference floating-base pose enters local FK: the free joint is explicitly
replaced by the identity before mj_kinematics. Recorded world states are used
only to label support through replayed normal forces and to export a separate
evaluation reference. The browser graph never reads that reference to solve.
"""

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


SOURCE_REVISION = "dba5c5eaea5d9bd19f0fa91191bbbbc9a4291941"
SOURCE_PR = "https://github.com/borglab/GTDynamics/pull/446"
CHECKPOINT_SHA256 = "b45a629ce3b6c279201be60362408cb0586fac077e577964bd66a50c32fd4763"
POLICY_URL = "https://huggingface.co/robomotic/mjlab-policies/tree/f277ca8/g1_velocity/2026-04-26_20-11-37"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rounded(values, decimals=6):
    return np.asarray(values).round(decimals).tolist()


def support_episodes(loads, on, off, bridge_samples):
    """Hysteresis, then bridge bounded short low-load runs; no edge padding."""
    raw = np.zeros(loads.shape, dtype=bool)
    for side in range(2):
        active = False
        for i, force in enumerate(loads[:, side]):
            active = force >= (off if active else on)
            raw[i, side] = active
    clean = raw.copy()
    for side in range(2):
        edges = np.diff(np.r_[True, raw[:, side], True].astype(int))
        for start, end in zip(np.flatnonzero(edges == -1), np.flatnonzero(edges == 1)):
            if start > 0 and end < len(raw) and end - start <= bridge_samples:
                clean[start:end, side] = True
    episodes = []
    for side, name in enumerate(("left", "right")):
        edges = np.diff(np.r_[False, clean[:, side], False].astype(int))
        for ordinal, (start, end) in enumerate(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))):
            episodes.append({
                "id": f"{name[0].upper()}{ordinal:02d}", "foot": name,
                "start": int(start), "end": int(end - 1), "endExclusive": int(end),
                "clippedStart": bool(start == 0), "clippedEnd": bool(end == len(raw)),
            })
    return raw, clean, sorted(episodes, key=lambda c: (c["start"], c["foot"]))


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


def main():
    app = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=app.parents[1] / "examples/g1_mjlab/results")
    parser.add_argument("--metadata", type=Path, help="Original recorder walk.json, if separate from --data")
    parser.add_argument("--output", type=Path, default=app / "public/g1-walk.json")
    parser.add_argument("--dt", type=float, default=0.01, help="Original policy sample period, seconds")
    parser.add_argument("--on", type=float, default=20.0, help="Normal-load contact onset threshold, N")
    parser.add_argument("--off", type=float, default=10.0, help="Normal-load contact release threshold, N")
    parser.add_argument("--bridge-samples", type=int, default=1)
    args = parser.parse_args()
    if args.dt <= 0 or args.off > args.on or args.off < 0 or args.bridge_samples < 0:
        parser.error("Require dt > 0, 0 <= off <= on, and bridge-samples >= 0")

    model_path, recording_path = args.data / "model.mjb", args.data / "walk.npz"
    metadata_path = args.metadata or args.data / "walk.json"
    recorder_metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    model = mujoco.MjModel.from_binary_path(str(model_path))
    record = np.load(recording_path)
    qpos, qvel, ctrl = (record[name] for name in ("qpos", "qvel", "ctrl"))
    if qpos.shape[1] != model.nq or qvel.shape[1] != model.nv or ctrl.shape[1] != model.nu:
        raise ValueError("Recording dimensions do not match model")
    if not np.isfinite(qpos).all() or len(qpos) < 2:
        raise ValueError("Invalid recorded poses")
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot/pelvis")
    feet = {side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"robot/{side}_ankle_roll_link") for side in ("left", "right")}
    if root < 0 or min(feet.values()) < 0:
        raise ValueError("Expected the G1 PR model with robot/pelvis and ankle-roll bodies")
    bodies = [i for i in range(model.nbody) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").startswith("robot/")]
    body_map = {body: i for i, body in enumerate(bodies)}
    side_for_body = {body: i for i, body in enumerate(feet.values())}
    local, world = mujoco.MjData(model), mujoco.MjData(model)
    force = np.empty(6)
    loads = np.zeros((len(qpos), 2))
    samples, references = [], []
    reference_feet = {side: [] for side in feet}

    # Lower collision-envelope rectangle of the seven left sole capsules.
    # These are fixed material landmarks, not collision contact centroids.
    bounds = []
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] == feet["left"] and model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_CAPSULE:
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
            offset = rotation.reshape(3, 3)[:, 2] * model.geom_size[geom, 1]
            center, radius = model.geom_pos[geom], model.geom_size[geom, 0]
            bounds.extend((center - offset - radius, center - offset + radius,
                           center + offset - radius, center + offset + radius))
    lo, hi = np.min(bounds, axis=0), np.max(bounds, axis=0)
    outline = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
                        [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]]])

    for index in range(len(qpos)):
        local.qpos[:] = qpos[index]
        local.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        mujoco.mj_kinematics(model, local)
        world.qpos[:], world.qvel[:], world.ctrl[:] = qpos[index], qvel[index], ctrl[index]
        mujoco.mj_forward(model, world)
        for j in range(world.ncon):
            contact = world.contact[j]
            a, b = int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])
            side = side_for_body.get(a, side_for_body.get(b))
            # Terrain/world against a foot only: exclude any robot self-contact.
            if side is not None and (a not in body_map or b not in body_map):
                mujoco.mj_contactForce(model, world, j, force)
                loads[index, side] += max(0.0, float(force[0]))
        samples.append({
            "index": index, "t": round(index * args.dt, 8),
            "jointAngles": rounded(qpos[index, 7:]),
            "bodyPoints": rounded(local.xpos[bodies]),
            "feet": {side: {"p": rounded(local.xpos[body]), "q": rounded(local.xquat[body], 9)} for side, body in feet.items()},
            "normalLoads": {side: round(float(loads[index, j]), 4) for j, side in enumerate(feet)},
        })
        references.append({"p": rounded(qpos[index, :3], 9), "q": rounded(qpos[index, 3:7], 9)})
        for side, body in feet.items():
            reference_feet[side].append((world.xpos[body].copy(), world.xquat[body].copy()))

    raw, support, contacts = support_episodes(loads, args.on, args.off, args.bridge_samples)
    supported = np.flatnonzero(support.any(axis=1))
    if not len(supported):
        raise ValueError("Detector found no supported samples")
    first, last = int(supported[0]), int(supported[-1])
    event_indices = sorted({first, last, *(c["start"] for c in contacts), *(c["end"] for c in contacts)})
    frames = [{
        "index": i, "sampleIndex": i, "t": round(i * args.dt, 8),
        "starts": [c["id"] for c in contacts if c["start"] == i],
        "ends": [c["id"] for c in contacts if c["end"] == i],
        "contacts": [c["id"] for c in contacts if c["start"] <= i < c["endExclusive"]],
    } for i in event_indices]
    for i, sample in enumerate(samples):
        sample["support"] = {side: bool(support[i, j]) for j, side in enumerate(feet)}

    # The no-slip rigid-foot assumption is a hypothesis, not a simulator fact.
    # Keep its reference-only diagnostic beside the reference, outside inputs.
    rigidity = []
    for contact in contacts:
        (p0, q0), (p1, q1) = (reference_feet[contact["foot"]][i] for i in (contact["start"], contact["end"]))
        rigidity.append({"id": contact["id"],
                         "endpointTranslationMeters": round(float(np.linalg.norm(p1 - p0)), 9),
                         "endpointRotationDegrees": round(float(np.degrees(2 * np.arccos(np.clip(abs(q0 @ q1), -1, 1)))), 6)})
    reference_rigidity = {
        "description": "Held-out reference ankle motion between stance endpoints. A perfectly rigid no-slip foot would have zero translation and rotation; this compliant gait rocks its feet.",
        "medianEndpointTranslationMeters": float(np.median([r["endpointTranslationMeters"] for r in rigidity])),
        "maxEndpointTranslationMeters": max(r["endpointTranslationMeters"] for r in rigidity),
        "medianEndpointRotationDegrees": float(np.median([r["endpointRotationDegrees"] for r in rigidity])),
        "maxEndpointRotationDegrees": max(r["endpointRotationDegrees"] for r in rigidity),
        "episodes": rigidity,
    }

    metadata = {
        "title": "G1 pretrained MJLab walking sequence",
        "sourcePr": SOURCE_PR, "sourceRevision": SOURCE_REVISION,
        "recorder": f"https://github.com/borglab/GTDynamics/blob/{SOURCE_REVISION}/examples/g1_mjlab/record_walk.py",
        "policyUrl": POLICY_URL, "checkpointSha256": recorder_metadata.get("checkpoint_sha256", CHECKPOINT_SHA256),
        "recordingSha256": digest(recording_path), "modelSha256": digest(model_path),
        "sampleCount": len(samples), "sampleRateHz": 1 / args.dt,
        "samplePeriodSeconds": args.dt, "recordedDurationSeconds": len(samples) * args.dt,
        "timestampSpanSeconds": (len(samples) - 1) * args.dt,
        "timestampConvention": "t=0 denotes first saved post-policy-step sample; original recorder saves after env.step",
        "commandSpeedMetersPerSecond": recorder_metadata.get("command_speed_m_s", 0.5),
        "simulationTimestepSeconds": float(model.opt.timestep), "mujocoVersion": mujoco.__version__,
        "randomization": "Original recorder disables startup domain randomization, observation noise and random pushes; seed 42",
        "detector": {"source": "Native MuJoCo mj_forward replay at recorded qpos/qvel/ctrl; sum nonnegative mj_contactForce normal components for foot-terrain contacts",
                     "onNewtons": args.on, "offNewtons": args.off, "bridgeSamples": args.bridge_samples,
                     "bridgedSampleCount": int(np.sum(support & ~raw)),
                     "endpointConvention": "Support states are [start,endExclusive); end=endExclusive-1 is the last supported sample. Graph endpoints use start and end, so every observation is a supported state.",
                     "sampleResolutionSeconds": args.dt,
                     "limitation": "Support labels use simulator normal loads; contact detection from encoders alone requires an additional heuristic or sensor."},
        "graphSampleRange": [first, last],
        "excludedBoundarySamples": [i for i in range(len(samples)) if i < first or i > last],
        "excludedBoundaryReason": "No detected support at the initial saved sample; an unconstrained pose would be an isolated graph component.",
        "stats": {"episodeCount": len(contacts), "frameCount": len(frames),
                  "leftEpisodes": sum(c["foot"] == "left" for c in contacts),
                  "rightEpisodes": sum(c["foot"] == "right" for c in contacts),
                  "singleSupportSamples": int(np.sum(support.sum(axis=1) == 1)),
                  "doubleSupportSamples": int(np.sum(support.all(axis=1))),
                  "unsupportedSamples": int(np.sum(~support.any(axis=1))),
                  "graphComponents": component_count(frames),
                  "observationCount": sum(len(f["contacts"]) for f in frames),
                  "referenceDisplacementMeters": rounded(qpos[-1, :3] - qpos[first, :3], 9)},
        "licenseNotes": [
            {"name": "GTDynamics recorder and app", "license": "BSD-2-Clause", "url": "https://github.com/borglab/GTDynamics/blob/develop/LICENSE"},
            {"name": "MJLab G1 model source", "license": "Apache-2.0", "url": "https://github.com/mujocolab/mjlab/blob/main/LICENSE", "localNotice": "THIRD_PARTY_LICENSES.txt"},
            {"name": "Policy checkpoint", "note": "Used only by the original recorder; checkpoint and mesh assets are not distributed in this app", "url": POLICY_URL},
        ],
    }
    dataset = {
        "schemaVersion": 1, "metadata": metadata,
        "model": {"name": "Unitree G1 (29 actuated joints)", "upAxis": "z", "lengthUnit": "meter", "quaternionOrder": "wxyz",
                  "bodyNames": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i).removeprefix("robot/") for i in bodies],
                  "bodyParents": [body_map.get(int(model.body_parentid[i]), -1) for i in bodies],
                  "jointNames": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i).removeprefix("robot/") for i in range(model.njnt) if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE],
                  "footBodies": {side: body_map[body] for side, body in feet.items()},
                  "soleCorners": rounded(outline[:3]), "soleOutline": rounded(outline),
                  "soleCenter": rounded(np.mean(outline, axis=0)),
                  "soleDefinition": "Three fixed material points on a rectangular approximation of the capsule collision sole lower envelope, expressed in ankle-roll body coordinates; not changing collision centroids."},
        "samples": samples, "contacts": contacts, "frames": frames,
        "reference": {"description": "Recorded floating-base world poses for evaluation and optional overlay only; excluded from graph construction and optimization.",
                      "poses": references, "rigidity": reference_rigidity},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dataset, separators=(",", ":"), allow_nan=False) + "\n")
    print(json.dumps(metadata["stats"], indent=2))
    print(f"Wrote {args.output} ({args.output.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
