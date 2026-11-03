#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file export_recording.py
# @brief Serialize the kinodynamic example's numerical result for offline replay.
# @author GTDynamics contributors
"""Export the saved numerical example; this script performs no optimization.

First run examples/g1_mjlab/kinodynamic_walk.py --output /tmp/kinodynamics.
Then run this script --result /tmp/kinodynamics. MuJoCo FK fills display-only
bodies; estimated massive-link poses are copied from the actual solution.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any

import gtsam
import mujoco
import numpy as np
from numpy.typing import NDArray

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
from browser_geometry import browser_geometry
from g1_model import load_g1_model, prepare_model, VERSIONS
from mujoco_io import pose
from sole_support import support_rectangle


def write_gzip(path: Path, data: dict[str, Any]) -> None:
    """Save deterministic JSON with finite numbers only."""
    path.write_bytes(
        gzip.compress(
            json.dumps(data, separators=(",", ":"), allow_nan=False).encode(), mtime=0
        )
    )


def encode_pose(matrix: NDArray[np.float64]) -> list[float]:
    """Encode world position in metres followed by xyzw quaternion."""
    value = gtsam.Pose3(matrix)
    q = value.rotation().toQuaternion()
    return [*value.translation().tolist(), q.x(), q.y(), q.z(), q.w()]


def body_poses(
    model: mujoco.MjModel, arrays: dict[str, Any], initial: bool
) -> list[list[list[float]]]:
    """Replay actual estimated COM frames; FK only supplies non-estimated bodies."""
    matrices = arrays["initial_poses" if initial else "poses"]
    angles = arrays["initial_joint_angles" if initial else "joint_angles"]
    ids = arrays["link_ids"].tolist()
    root = int(arrays["root_id"])
    root_column = ids.index(root)
    body_to_com = pose(model.body_ipos[root], model.body_iquat[root])
    addresses = [
        model.joint("robot/" + name).qposadr[0] for name in arrays["joint_names"]
    ]
    data = mujoco.MjData(model)
    result = []
    for com_poses, joints in zip(matrices, angles):
        world_body = gtsam.Pose3(com_poses[root_column]) * body_to_com.inverse()
        q = world_body.rotation().toQuaternion()
        data.qpos[:] = model.qpos0
        data.qpos[:7] = [*world_body.translation(), q.w(), q.x(), q.y(), q.z()]
        data.qpos[addresses] = joints + model.qpos0[addresses]
        mujoco.mj_kinematics(model, data)
        sample = np.column_stack([data.xpos, data.xquat[:, [1, 2, 3, 0]]]).tolist()
        for identifier, matrix in zip(ids, com_poses):
            body_com = pose(model.body_ipos[identifier], model.body_iquat[identifier])
            sample[identifier] = encode_pose(
                (gtsam.Pose3(matrix) * body_com.inverse()).matrix()
            )
        result.append(sample)
    return result


def export(result_path: Path, output: Path) -> None:
    """Package model geometry, initial/final replay, forces, velocities and timing."""
    arrays = dict(np.load(result_path / "kinodynamics.npz", allow_pickle=False))
    report = json.loads((result_path / "summary.json").read_text())
    validation = result_path / "validation.json"
    if validation.exists():
        report["validation"] = json.loads(validation.read_text())
    model = load_g1_model()
    output.mkdir(parents=True, exist_ok=True)
    ids = arrays["link_ids"].tolist()
    contact_columns = [ids.index(i) for i in arrays["contact_link_ids"]]
    poses = arrays["poses"][:, contact_columns] @ arrays["contact_transforms"]
    initial_poses = (
        arrays["initial_poses"][:, contact_columns] @ arrays["contact_transforms"]
    )
    wrenches = arrays["wrenches"]
    world_forces = np.einsum("nfij,nfj->nfi", poses[:, :, :3, :3], wrenches[:, :, 3:])
    world_moments = np.einsum("nfij,nfj->nfi", poses[:, :, :3, :3], wrenches[:, :, :3])
    root = int(arrays["root_id"])
    root_column = ids.index(root)
    twists = arrays["body_twists"]
    sole_maps = np.array(
        [
            gtsam.Pose3(matrix).inverse().AdjointMap()
            for matrix in arrays["contact_transforms"]
        ]
    )
    foot_twists = np.einsum("fij,nfj->nfi", sole_maps, twists[:, contact_columns])
    stance_speeds = np.linalg.norm(foot_twists[:, :, 3:], axis=2)[arrays["contact"]]
    effort_by_name = {
        model.joint(int(model.actuator_trnid[a, 0])).name.split("/")[-1]: float(
            np.max(np.abs(model.actuator_forcerange[a]))
        )
        for a in range(model.nu)
    }
    effort_limits = np.array([effort_by_name[name] for name in arrays["joint_names"]])
    report["stanceFootSpeedRmsMetersPerSecond"] = float(
        np.sqrt(np.mean(stance_speeds**2))
    )
    report["maxJointEffortLimitRatio"] = float(
        np.max(np.abs(arrays["joint_torques"]) / effort_limits)
    )
    world_velocity = np.einsum(
        "nij,nj->ni",
        arrays["poses"][:, root_column, :3, :3],
        twists[:, root_column, 3:],
    )
    link_accelerations = np.zeros((len(poses), model.nbody, 6))
    link_accelerations[:, ids] = arrays["body_twist_accelerations"]
    solver_provenance_path = result_path / "solver-provenance.json"
    solver_provenance = (
        json.loads(solver_provenance_path.read_text())
        if solver_provenance_path.exists()
        else {}
    )
    provenance = json.loads((prepare_model() / "provenance.json").read_text())
    final_errors = report["errorHistory"][-1]
    factor_counts = [
        29 * len(poses),
        int(arrays["contact"].sum()),
        len(poses) + 1,
        30 * (len(poses) - 1),
        30 * (len(poses) - 1),
        int(arrays["contact"].sum()),
        29 * len(poses),
        2 * len(poses),
        29 * (len(poses) - 1),
        int(np.logical_and(arrays["contact"][1:], arrays["contact"][:-1]).sum()),
        30 * len(poses),
        2,  # Translation and wrapped-yaw gauge costs.
    ]
    if len(factor_counts) != len(report["errorNames"]):
        raise ValueError("Update factor counts to match the example error groups")
    record = {
        "schemaVersion": 1,
        "title": "SSIO G1 mid-clip turns · full 50 Hz kinodynamics",
        "times": arrays["times"].tolist(),
        "bodyNames": [model.body(i).name for i in range(model.nbody)],
        "rootBody": root,
        "bodyPoses": body_poses(model, arrays, False),
        "contactPoses": [[encode_pose(p) for p in sample] for sample in poses],
        "contact": arrays["contact"].tolist(),
        "wrenches": wrenches.tolist(),
        "worldForces": world_forces.tolist(),
        "worldMoments": world_moments.tolist(),
        "worldVelocities": world_velocity.tolist(),
        "baseTwists": twists[:, root_column].tolist(),
        "baseTwistAccelerations": arrays["body_twist_accelerations"][
            :, root_column
        ].tolist(),
        "baseAccelerationPriors": np.zeros((len(poses), 6)).tolist(),
        "linkTwistAccelerations": link_accelerations.tolist(),
        "jointNames": arrays["joint_names"].tolist(),
        "jointAngles": arrays["joint_angles"].tolist(),
        "jointVelocities": arrays["joint_velocities"].tolist(),
        "jointAccelerations": arrays["joint_accelerations"].tolist(),
        "jointAccelerationPriors": np.zeros_like(
            arrays["joint_accelerations"]
        ).tolist(),
        "jointTorques": arrays["joint_torques"].tolist(),
        "jointEffortLimits": effort_limits.tolist(),
        "initial": {
            "bodyPoses": body_poses(model, arrays, True),
            "contactPoses": [
                [encode_pose(p) for p in sample] for sample in initial_poses
            ],
        },
        "referenceRootPositions": arrays["reference_root_positions"].tolist(),
        "diagnostics": {
            **report,
            "solver": {
                "name": "CMC-Opt · IELM · articulated FK / affine AL retraction",
                "iterations": report["iterations"],
                "elapsedSeconds": report["coupledSolveSeconds"],
            },
            "status": (
                "Exact articulated mechanics and sole-wrench cones satisfied"
                if report["feasible"]
                else "Constraint violations remain; inspect residuals"
            ),
            "factors": [
                {"type": n, "error": e, "count": c}
                for n, e, c in zip(report["errorNames"], final_errors, factor_counts)
            ],
        },
        "settings": {
            "sampleRateHz": 50,
            "retractionThreads": report.get("retractionThreads", 1),
            "kinematicsEstimated": True,
            "accelerationsEstimated": True,
            "friction": 0.8,
            "torqueLimitsEnforced": False,
            "supportRectangles": [
                list(support_rectangle(model, side)) for side in ("left", "right")
            ],
            "encoderSigmaDegrees": 0.5,
            "contactRotationSigmaDegrees": 90.0,
            "contactTranslationSigma": 0.01,
            "effortSigmaNm": 100.0,
            "wrenchSigmaN": 500.0,
            "torqueRateSigmaNmPerSqrtSecond": 100.0,
            "wrenchRateSigmaNPerSqrtSecond": 500.0,
            "baseAngularAccelerationSigmaDegrees": 1000.0,
            "baseLinearAccelerationSigma": 30.0,
            "jointAccelerationSigmaDegrees": 2000.0,
            "collocation": report["collocation"],
            "accelerationDifferenceStepSeconds": 0.02,
        },
        "source": {
            "dataset": "SSIO",
            "clip": [float(arrays["times"][0]), float(arrays["times"][-1])],
            "sourceIndices": arrays["source_indices"].tolist(),
            "numericResultSha256": hashlib.sha256(
                (result_path / "kinodynamics.npz").read_bytes()
            ).hexdigest(),
            "labels": report["labels"],
            "referenceUsage": report["referenceUsage"],
        },
        "provenance": {"model": provenance, "solver": solver_provenance},
        "attribution": [
            {
                "label": "Open-IO G1 / SSIO dataset",
                "url": "https://huggingface.co/datasets/Open-IO/humanoid-g1-odometry",
                "text": "Sequence 10871_humanoid_20260520_223451; CC BY 4.0",
            },
            {
                "label": "MuJoCo Menagerie Unitree G1",
                "url": "https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1",
                "text": "Canonical g1_mjlab model; BSD 3-Clause",
            },
        ],
    }
    write_gzip(output / "recording.json.gz", record)
    write_gzip(output / "model.json.gz", browser_geometry(model))
    # Preserve authors and licenses, then document this derived recording.
    for notice in ("SSIO_ATTRIBUTION.txt", "MODEL_LICENSE.txt"):
        shutil.copyfile(REPO / "apps/dynamics/public" / notice, output / notice)
    notice = output / "SSIO_ATTRIBUTION.txt"
    text = notice.read_text().split("Changes:")[0]
    notice.write_text(
        text
        + "Changes: selected the 78–98 s mid-clip turns at the original 50 Hz. Used saved max-product support labels derived from SuperOdom LiDAR-inertial foot motion and measured joint velocities. Contact-initialized configurations, then jointly estimated configurations, velocities, accelerations, sole/reaction wrenches and joint torques with native CMC-Opt and trapezoidal collocation. All dynamic variables and the shared IMU bias started at zero. Later reference poses are evaluation data. Forces and torques are model-based estimates. The browser replays stored results.\n"
    )
    print(f"Exported {len(poses)} slices to {output}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, default=REPO / "apps/kinodynamics/public"
    )
    args = parser.parse_args()
    export(args.result, args.output)


if __name__ == "__main__":
    main()
