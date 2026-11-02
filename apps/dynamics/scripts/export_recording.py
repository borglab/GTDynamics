#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file export_recording.py
# @brief Serialize the shared dynamics example for offline browser replay.
# @author GTDynamics contributors

"""Run the shared G1 dynamics example and serialize an offline browser replay."""

from __future__ import annotations

import argparse
from dataclasses import replace
import gzip
import json
from pathlib import Path
import sys

import gtdynamics as gtd
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
import dynamics_walk as example
from browser_geometry import browser_geometry
from g1_model import VERSIONS
from interpolate_walk import checksum, sample_walk


def export(args: argparse.Namespace) -> None:
    """Call the dynamics example and serialize its solution for browser replay."""
    solution = example.solve_walk(
        args.kinematics,
        args.dense_data,
        start=args.start,
        end=args.end,
        friction=args.friction,
        acceleration_step_seconds=args.acceleration_step_ms / 1000,
        options=example.options_from_arguments(args),
    )
    walk, fixed, result = solution.walk, solution.fixed, solution.result
    options, rectangles = solution.options, solution.support_rectangles
    elapsed = solution.elapsed_seconds
    model, record = walk.model, walk.record
    metadata = args.kinematics / "variants" / walk.variant / "final.json.gz"
    binary = metadata.with_name("final.bin.gz")
    samples = sample_walk(replace(walk, dense_times=fixed.times))
    arrays = example.solution_arrays(solution)
    print(json.dumps(example.summarize(solution), indent=2), flush=True)
    count, j = len(fixed.times), len(record["jointNames"])
    wrenches, torques = arrays["wrenches"], arrays["joint_torques"]
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
            "kinematicFullRecordingSha256": checksum(metadata),
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
        "baseTwistAccelerations": arrays["base_twist_accelerations"].tolist(),
        "baseAccelerationPriors": arrays["base_twist_acceleration_priors"].tolist(),
        "jointAccelerations": arrays["joint_accelerations"][:, columns].tolist(),
        "jointAccelerationPriors": arrays["joint_acceleration_priors"][
            :, columns
        ].tolist(),
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
            "interpolation": example.GP_SETTINGS,
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
            "kinematicsMetadataSha256": checksum(metadata),
            "kinematicsBinarySha256": checksum(binary),
            "exporterSha256": checksum(Path(__file__)),
            "nativeSolverSha256": checksum(
                REPO / "gtdynamics/dynamics/FixedKinematicsDynamics.cpp"
            ),
            "sourceSha256": {
                name: checksum(REPO / name)
                for name in (
                    "examples/g1_mjlab/dynamics_walk.py",
                    "examples/g1_mjlab/sole_support.py",
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
    """Export the example's solution in the static browser recording format."""
    parser = example.argument_parser()
    parser.description = __doc__
    parser.add_argument("--output", type=Path, default=REPO / "apps/dynamics/public")
    export(parser.parse_args())


if __name__ == "__main__":
    main()
