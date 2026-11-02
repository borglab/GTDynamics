#!/usr/bin/env python3
"""Solve fixed-kinematics G1 dynamics offline; publish only browser replay data."""

from __future__ import annotations

import argparse
from dataclasses import replace
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from numpy.typing import NDArray

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
sys.path.insert(0, str(REPO / "apps/kinematics-inspector/scripts"))
from fixed_kinematics import prepare_fixed_kinematics
from browser_geometry import browser_geometry
from g1_model import sole_frame, VERSIONS
from interpolate_walk import fit_walk, sample_walk


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def planar_hull(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return counterclockwise vertices of a planar convex hull, in metres.

    Andrew's monotone chain removes duplicates and interior collinear points.
    The support-patch calculation needs only this two-dimensional boundary.
    """
    ordered = np.unique(points, axis=0)
    boundary = []
    for sequence in (ordered, ordered[::-1]):
        half = []
        for point in sequence:
            while len(half) >= 2:
                first, second = half[-1] - half[-2], point - half[-1]
                if first[0] * second[1] - first[1] * second[0] > 0:
                    break
                half.pop()
            half.append(point)
        boundary.extend(half[:-1])
    if len(boundary) < 3:
        raise ValueError("The support footprint must have nonzero planar area")
    return np.asarray(boundary)


def support_rectangle(model: mujoco.MjModel, side: str) -> tuple[float, float]:
    """Inscribed symmetric rectangle in the projected capsule support hull.

    A 5% margin avoids replacing rounded capsule footprints by an optimistic
    bounding box. This is a planar conservative support-patch assumption.
    """
    body = model.body(f"robot/{side}_ankle_roll_link").id
    origin = sole_frame(model, side)[:2, 3]
    points = []
    angles = np.linspace(0, 2 * np.pi, 128, endpoint=False)
    circle = np.column_stack([np.cos(angles), np.sin(angles)])
    for geom in range(model.ngeom):
        if (
            model.geom_bodyid[geom] != body
            or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_CAPSULE
        ):
            continue
        axis = gtsam.Rot3.Quaternion(*model.geom_quat[geom]).matrix()[:, 2]
        for sign in (-1, 1):
            center = model.geom_pos[geom] + sign * axis * model.geom_size[geom, 1]
            points.extend(center[:2] - origin + circle * model.geom_size[geom, 0])
    points = np.asarray(points)
    half = np.minimum(-points.min(axis=0), points.max(axis=0))
    hull = planar_hull(points)
    edges = np.roll(hull, -1, axis=0) - hull
    normals = np.column_stack([edges[:, 1], -edges[:, 0]])
    # Each CCW hull edge gives an outward normal n and the bound n.x <= n.p.
    # A symmetric rectangle's support along n is abs(n) @ half.
    offsets = np.sum(normals * hull, axis=1)
    if np.any(offsets <= 0) or np.any(half <= 0):
        raise ValueError("The sole origin must lie inside the support footprint")
    scale = np.min(offsets / (np.abs(normals) @ half))
    return tuple((half * min(1.0, scale) * 0.95).tolist())


def export(args: argparse.Namespace) -> None:
    """Fit GP velocities, estimate accelerations and wrenches, and save replay."""
    gp_settings = {
        "joint_process_sigma_degrees": 100.0,
        "joint_measurement_sigma_degrees": 0.5,
        "base_angular_process_sigma_degrees": 90.0,
        "base_linear_process_sigma": 3.0,
        "base_velocity_sigma": 0.5,
    }
    walk = fit_walk(args.kinematics, args.dense_data, **gp_settings)
    model, record = walk.model, walk.record
    metadata = args.kinematics / "variants" / walk.variant / "final.json.gz"
    binary = metadata.with_name("final.bin.gz")
    fixed = prepare_fixed_kinematics(
        walk,
        args.start,
        args.end,
        args.acceleration_step_ms / 1000,
    )
    samples = sample_walk(replace(walk, dense_times=fixed.times))
    j = len(record["jointNames"])
    print("Prepared fixed kinematics:", fixed.diagnostics, flush=True)
    rectangles = [support_rectangle(model, side) for side in ("left", "right")]
    contacts = [
        gtd.fixed_dynamics.Contact(fixed.contacts[side], *rectangle, args.friction)
        for side, rectangle in zip(("left", "right"), rectangles)
    ]
    options = gtd.fixed_dynamics.Options()
    options.baseAngularAccelerationSigmaDegrees = (
        args.base_angular_acceleration_sigma_degrees
    )
    options.baseLinearAccelerationSigma = args.base_linear_acceleration_sigma
    options.jointAccelerationSigmaDegrees = args.joint_acceleration_sigma_degrees
    options.maxIterations = args.max_iterations
    options.softDynamics = args.soft_base_balance
    options.balanceForceSigma = args.base_force_sigma
    options.balanceMomentSigma = args.base_moment_sigma
    start = time.perf_counter()
    result = gtd.solveFixedKinematicsDynamics(
        fixed.robot,
        fixed.trajectory,
        fixed.slices,
        fixed.times.tolist(),
        fixed.values,
        fixed.acceleration_priors,
        contacts,
        fixed.armature,
        options,
    )
    elapsed = time.perf_counter() - start
    print(
        "Solved:",
        {
            "seconds": elapsed,
            "iterations": result.iterations,
            "feasible": result.feasible,
            "maxForceResidual": result.maxForceResidual,
            "maxMomentResidual": result.maxMomentResidual,
        },
        flush=True,
    )
    count = len(fixed.times)
    wrenches = np.array(
        [result.values.atVector(key) for key in result.contactKeys]
    ).reshape(count, 2, 6)
    torques = np.array(
        [result.values.atDouble(key) for key in result.torqueKeys]
    ).reshape(count, -1)
    joint_by_name = {joint.name(): joint.id() for joint in fixed.robot.joints()}
    joint_ids = [joint_by_name[name.split("/")[-1]] for name in record["jointNames"]]
    columns = [list(result.jointIds).index(identifier) for identifier in joint_ids]
    interpolation_columns = [
        walk.interpolation.jointNames().index(name.split("/")[-1])
        for name in record["jointNames"]
    ]
    link_ids = {link.id() for link in fixed.robot.links()}
    contact_poses, forces, moments = [], [], []
    for index, slice_ in enumerate(fixed.slices):
        k = slice_.k
        feet = [
            fixed.contacts[side].predict(fixed.values, k) for side in ("left", "right")
        ]
        contact_poses.append(samples["contactPoses"][index].tolist())
        forces.append(
            [p.rotation().rotate(w[3:]).tolist() for p, w in zip(feet, wrenches[index])]
        )
        moments.append(
            [p.rotation().rotate(w[:3]).tolist() for p, w in zip(feet, wrenches[index])]
        )
    factor_rows = [
        {"type": "Motor effort", "count": count * j, "error": result.effortError},
        {
            "type": "Contact wrench regularization",
            "count": count * 2,
            "error": result.wrenchError,
        },
        {
            "type": "Torque rate",
            "count": (count - 1) * j,
            "error": result.torqueSmoothError,
        },
        {
            "type": "Wrench rate",
            "count": result.wrenchSmoothFactors,
            "error": result.wrenchSmoothError,
        },
        {
            "type": "All exact dynamics constraints",
            "count": count
            * (len(fixed.robot.links()) - int(options.softDynamics) + 2 + 3 * j)
            + sum(not flag for sample in fixed.active for flag in sample),
            "error": 0,
            "maxViolation": result.equalityL2,
        },
        {
            "type": "Contact wrench inequalities",
            "count": sum(sum(sample) for sample in fixed.active) * 17,
            "error": 0,
            "maxViolation": result.inequalityL2,
        },
    ]
    factor_rows = [row for row in factor_rows if row["count"]]
    factor_rows.append(
        {
            "type": "GP velocity-difference acceleration priors",
            "count": count * (j + 1),
            "error": result.accelerationError,
        }
    )
    if options.softDynamics:
        factor_rows.append(
            {
                "type": "Soft floating-base balance",
                "count": count,
                "error": result.balanceError,
            }
        )
    diagnostics = {
        name: getattr(result, name)
        for name in (
            "objectiveInitial",
            "objectiveFinal",
            "equalityL2",
            "inequalityL2",
            "maxForceResidual",
            "maxMomentResidual",
            "assemblySeconds",
            "initializationSeconds",
            "manifoldSeconds",
            "optimizationSeconds",
            "iterations",
            "variableDimension",
            "equalityDimension",
            "inequalityDimension",
            "reducedDimension",
            "numALSolves",
            "numFeasibleTrials",
            "numFeasibleProjections",
            "feasible",
            "softDynamics",
        )
    }
    diagnostics.update(fixed.diagnostics)
    stance_speeds = np.linalg.norm(samples["footTwists"][:, :, 3:], axis=2)
    diagnostics["stanceFootSpeedRmsMetersPerSecond"] = float(
        np.sqrt(np.mean(stance_speeds[np.asarray(fixed.active)] ** 2))
    )
    diagnostics["exactConstraintFactorCounts"] = {
        "linkNewtonEulerBalance": count
        * (len(fixed.robot.links()) - int(options.softDynamics)),
        "soleToComWrenchMapping": count * 2,
        "jointWrenchEquivalence": count * j,
        "motorTorque": count * j,
        "articulatedAccelerationKinematics": count * j,
        "zeroSwingWrench": sum(not flag for sample in fixed.active for flag in sample),
    }
    if hasattr(result, "stationary"):
        diagnostics["stationary"] = bool(result.stationary)
        diagnostics["projectedGradientNorm"] = float(result.projectedGradientNorm)
    diagnostics.update(
        {
            "solver": {
                "name": "CMC-Opt · IELM · affine ReducedALRetractor",
                "elapsedSeconds": elapsed,
                "iterations": result.iterations,
            },
            "status": (
                "Soft base balance: unmodeled floating-root wrench remains; contact constraints are enforced"
                if options.softDynamics
                else "Exact dynamics: equality and contact wrench constraints satisfied"
            ),
            "accelerationMethod": "Root and joint accelerations estimated with soft [-1, 0, 1] GP velocity-difference priors at retained knots and exact articulated acceleration constraints",
            "factors": factor_rows,
            "objectiveHistory": list(result.objectiveHistory),
            "baseResidualsRootCom": [v.tolist() for v in result.baseResiduals],
            "maxEffortLimitRatio": float(
                np.max(np.abs(torques[:, columns]) / fixed.effort_limits[joint_ids])
            ),
        }
    )
    if not result.feasible:
        diagnostics["status"] = (
            "Solver returned constraint violations; inspect the reported residuals"
        )
    output = {
        "schemaVersion": 1,
        "title": "SSIO G1 · mid-clip turns · GP-constrained dynamics",
        "source": {
            "dataset": "SSIO",
            "clip": [args.start, args.end],
            "variant": walk.variant,
            "kinematicsVariant": walk.variant,
            "sourceIndices": np.asarray(record["sampling"]["sourceIndices"])[
                fixed.indices
            ].tolist(),
            "retainedFullTrajectoryIndices": fixed.indices.tolist(),
            "kinematicFullRecordingSha256": digest(metadata),
        },
        "times": fixed.times.tolist(),
        "bodyNames": record["bodyNames"],
        "rootBody": record["rootBody"],
        "bodyPoses": samples["bodyPoses"].tolist(),
        "footNames": ["left", "right"],
        "contactPoses": contact_poses,
        "contact": fixed.active,
        "wrenches": wrenches.tolist(),
        "worldForces": forces,
        "worldMoments": moments,
        "jointNames": record["jointNames"],
        "jointTorques": torques[:, columns].tolist(),
        "jointAngles": samples["jointAngles"][:, interpolation_columns].tolist(),
        "jointVelocities": samples["jointVelocities"][
            :, interpolation_columns
        ].tolist(),
        "baseTwists": samples["baseTwists"].tolist(),
        "baseTwistAccelerations": [
            gtd.TwistAccel(
                result.values, fixed.robot.link("pelvis").id(), slice_.k
            ).tolist()
            for slice_ in fixed.slices
        ],
        "baseAccelerationPriors": [
            gtd.TwistAccel(
                fixed.acceleration_priors, fixed.robot.link("pelvis").id(), slice_.k
            ).tolist()
            for slice_ in fixed.slices
        ],
        "jointAccelerations": [
            [
                gtd.JointAccel(result.values, joint_id, slice_.k)
                for joint_id in joint_ids
            ]
            for slice_ in fixed.slices
        ],
        "jointAccelerationPriors": [
            [
                gtd.JointAccel(fixed.acceleration_priors, joint_id, slice_.k)
                for joint_id in joint_ids
            ]
            for slice_ in fixed.slices
        ],
        "linkTwistAccelerations": [
            [
                (
                    gtd.TwistAccel(result.values, body, slice_.k).tolist()
                    if body in link_ids
                    else [0.0] * 6
                )
                for body in range(model.nbody)
            ]
            for slice_ in fixed.slices
        ],
        "jointEffortLimits": fixed.effort_limits[joint_ids].tolist(),
        "settings": {
            "maxIterations": args.max_iterations,
            "friction": args.friction,
            "frictionPyramidCoefficient": args.friction / np.sqrt(2),
            "halfLength": rectangles[0][0],
            "halfWidth": rectangles[0][1],
            "supportRectangles": rectangles,
            "gravity": [0, 0, -9.81],
            "effortSigmaNm": options.effortSigma,
            "wrenchSigmaN": options.wrenchSigma,
            "torqueRateSigmaNmPerSqrtSecond": options.torqueSmoothSigma,
            "wrenchRateSigmaNPerSqrtSecond": options.wrenchSmoothSigma,
            "balanceMomentSigmaNm": options.balanceMomentSigma,
            "balanceForceSigmaN": options.balanceForceSigma,
            "torqueLimitsEnforced": False,
            "kinematicsFixed": True,
            "interpolation": gp_settings,
            "accelerationsEstimated": True,
            "baseAngularAccelerationSigmaDegreesPerSecondSquared": options.baseAngularAccelerationSigmaDegrees,
            "baseLinearAccelerationSigmaMetersPerSecondSquared": options.baseLinearAccelerationSigma,
            "jointAccelerationSigmaDegreesPerSecondSquared": options.jointAccelerationSigmaDegrees,
            "accelerationDifferenceStepSeconds": args.acceleration_step_ms / 1000,
            "sampling": "Exactly the saved double-support kinematic slices; no interpolated solve states",
            "wrenchOrder": "[moment; force], sole-local, N m and N; world moments are about sole origin",
            "massKg": float(model.body_mass.sum()),
            "armatureIncluded": True,
        },
        "diagnostics": diagnostics,
        "provenance": {
            "packages": VERSIONS,
            "modelFingerprint": record.get("modelFingerprint"),
            "kinematicsMetadataSha256": digest(metadata),
            "kinematicsBinarySha256": digest(binary),
            "exporterSha256": digest(Path(__file__)),
            "nativeSolverSha256": digest(
                REPO / "gtdynamics/dynamics/FixedKinematicsDynamics.cpp"
            ),
            "sourceSha256": {
                name: digest(REPO / name)
                for name in (
                    "examples/g1_mjlab/fixed_kinematics.py",
                    "examples/g1_mjlab/interpolate_walk.py",
                    "gtdynamics/utils/FloatingBaseInterpolator.cpp",
                    "gtdynamics.i",
                    "gtdynamics/dynamics/FixedKinematicsDynamics.cpp",
                    "gtdynamics/dynamics/ContactWrenchFactor.h",
                    "gtdynamics/utils/ContactWrench.cpp",
                    "gtdynamics/cmcopt/ReducedALRetractor.cpp",
                    "gtdynamics/cmcopt/RobustQp.cpp",
                )
            },
        },
        "attribution": [
            {
                "label": "SSIO / Open-IO G1 dataset and SuperOdom LiDAR-inertial reference (CC BY 4.0)",
                "url": "https://huggingface.co/datasets/Open-IO/humanoid-g1-odometry",
            },
            {
                "label": "Caron, Pham and Nakamura: Stability of Surface Contacts for Humanoid Robots (2015)",
                "url": "https://arxiv.org/abs/1501.04719",
            },
            {
                "label": "MuJoCo Menagerie Unitree G1 model",
                "url": "https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1",
            },
        ],
    }
    base_correction = np.asarray(output["baseTwistAccelerations"]) - np.asarray(
        output["baseAccelerationPriors"]
    )
    joint_correction = np.asarray(output["jointAccelerations"]) - np.asarray(
        output["jointAccelerationPriors"]
    )
    diagnostics["accelerationCorrections"] = {
        "rootAngularRmsDegreesPerSecondSquared": float(
            np.rad2deg(np.sqrt(np.mean(base_correction[:, :3] ** 2)))
        ),
        "rootLinearRmsMetersPerSecondSquared": float(
            np.sqrt(np.mean(base_correction[:, 3:] ** 2))
        ),
        "jointRmsDegreesPerSecondSquared": float(
            np.rad2deg(np.sqrt(np.mean(joint_correction**2)))
        ),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(output, separators=(",", ":"), allow_nan=False).encode()
    (args.output / "recording.json.gz").write_bytes(gzip.compress(encoded, mtime=0))
    geometry = json.dumps(browser_geometry(model), separators=(",", ":")).encode()
    (args.output / "model.json.gz").write_bytes(gzip.compress(geometry, mtime=0))
    print(
        f"Wrote {count} slices, {j} torque traces, {len(encoded)} uncompressed bytes",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--kinematics",
        type=Path,
        default=REPO / "apps/kinematics-inspector/public",
    )
    parser.add_argument(
        "--dense-data",
        type=Path,
        default=REPO / "apps/kinematics-inspector/.cache/openio-contact-source",
    )
    parser.add_argument("--acceleration-step-ms", type=float, default=20.0)
    parser.add_argument(
        "--base-angular-acceleration-sigma-degrees", type=float, default=90.0
    )
    parser.add_argument("--base-linear-acceleration-sigma", type=float, default=3.0)
    parser.add_argument("--joint-acceleration-sigma-degrees", type=float, default=100.0)
    parser.add_argument("--output", type=Path, default=REPO / "apps/dynamics/public")
    parser.add_argument("--start", type=float, default=78.0)
    parser.add_argument("--end", type=float, default=98.0)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument(
        "--base-force-sigma",
        type=float,
        default=5.0,
        help="Soft-mode base force balance standard deviation, N",
    )
    parser.add_argument(
        "--base-moment-sigma",
        type=float,
        default=1.0,
        help="Soft-mode base moment balance standard deviation, N m",
    )
    parser.add_argument(
        "--soft-base-balance",
        action="store_true",
        help="Explicitly soften only floating-root Newton–Euler balance and publish residuals",
    )
    export(parser.parse_args())


if __name__ == "__main__":
    main()
