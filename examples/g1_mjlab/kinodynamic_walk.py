#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_walk.py
# @brief Solve dense G1 kinodynamics from contact initialization and zero dynamics.
# @author GTDynamics contributors
"""Solve the SSIO mid-clip turns jointly at 50 Hz using native CMC-Opt.

The saved Contact Labels max-product schedule supplies stance phases. Encoder
measurements and one initial root gauge initialize the pose-contact graph. All
velocities, accelerations, contact/reaction wrenches, torques and IMU bias start
at zero in the coupled graph. Native feasibility restoration is timed separately
and included in the solve time. No saved smoothing or GP solution is loaded.

Run: python examples/g1_mjlab/kinodynamic_walk.py --output /tmp/kinodynamics
Use --end 79 --max-iterations 2 for a small development run. Computation lives
here and in the library; the app exporter only serializes these results.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import gzip
import json
from pathlib import Path
import time
from typing import Any

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from contact_phases import contact_slices, initialization_slices, stance_observations
from g1_model import load_g1_model
from imu_aiding import BIAS_KEY, ImuAiding, build_imu_aiding
from kinematic_initialization import align_contact_ground, seed_contact_landmarks
from mujoco_io import (
    KinematicRecording,
    contact_frames,
    import_robot,
    load_recording,
    model_bodies,
    pose,
)
from sole_support import support_rectangle

REPO = Path(__file__).resolve().parents[2]


@dataclass
class KinodynamicSolution:
    """Full dense solution, initial pose estimate, physical model and timings."""

    walk: KinematicRecording
    result: gtd.kinodynamics.Result
    contacts: list[gtd.fixed_dynamics.Contact]
    initial: gtsam.Values
    metadata: dict[str, Any]


def prepare_walk(
    directory: Path,
    labels_path: Path,
    start: float,
    end: float,
) -> tuple[KinematicRecording, dict[str, Any]]:
    """Crop the original 50 Hz encoders and saved support labels by timestamp.

    LiDAR reference poses only provide the first selected root gauge and
    evaluation overlays. Privileged LiDAR motion also supplied contact labels.
    Joint indices are mapped by name through the canonical compiled model.
    """
    arrays, metadata = load_recording(directory)
    labels = json.loads(gzip.decompress(labels_path.read_bytes()))
    all_times = np.asarray(labels["times"])
    selected = (all_times >= start - 1e-8) & (all_times <= end + 1e-8)
    times = all_times[selected]
    if len(times) < 2 or not np.allclose(np.diff(times), 0.02, atol=1e-8):
        raise ValueError("Requested clip requires consecutive 50 Hz label samples")
    indices = np.rint(times * 50).astype(np.int64)
    states = np.asarray(labels["supportGraph"]["states"])[selected]
    model = load_g1_model()
    robot: gtd.Robot = import_robot(model_bodies(model))
    root: gtd.Link = robot.link("pelvis")
    frames: dict[str, gtd.PoseOnLink] = contact_frames(model, robot)
    measured = gtsam.Values()
    for joint in robot.joints():
        address = model.jnt_qposadr[model.joint("robot/" + joint.name()).id]
        for k, source in enumerate(indices):
            measured.insert(
                gtd.JointAngleKey(joint.id(), k),
                float(arrays["qpos"][source, address] - model.qpos0[address]),
            )
    phases: list[gtd.Phase] = []
    first = 0
    for stop in range(1, len(times) + 1):
        if stop < len(times) and states[stop] == states[first]:
            continue
        active = [
            side
            for side, valid in (
                ("left", states[first] in (0, 2)),
                ("right", states[first] in (1, 2)),
            )
            if valid
        ]
        phases.append(
            gtd.Phase(
                first,
                stop - 1,
                gtd.PoseContactConstraintSpec([frames[s] for s in active]),
            )
        )
        first = stop
    trajectory: gtd.Trajectory = gtd.Trajectory(gtd.WalkCycle(phases), 1)
    retained: list[gtd.Slice] = initialization_slices(trajectory, 0.02)
    if not retained:
        raise ValueError("Contact initialization requires double support")
    body_to_com: gtsam.Pose3 = pose(
        model.body_ipos[root.id()], model.body_iquat[root.id()]
    )
    references = [pose(q[:3], q[3:7]) * body_to_com for q in arrays["qpos"][indices]]
    walk = KinematicRecording(
        robot,
        root,
        measured,
        references[retained[0].k],
        trajectory,
        len(times),
        0.02,
        0.0,
        np.array([p.translation() for p in references]),
        {},
        times,
        indices,
    )
    walk, shift = align_contact_ground(walk, retained[0].k)
    return walk, {
        "source": metadata,
        "groundTranslation": shift.tolist(),
        "initializationSlices": [s.k for s in retained],
        "referenceUsage": "Privileged support labels and first selected root gauge only; later poses are evaluation data",
        "labels": (
            str(labels_path.relative_to(REPO))
            if labels_path.is_relative_to(REPO)
            else str(labels_path)
        ),
    }


def solve_walk(
    directory: Path,
    labels_path: Path,
    *,
    start: float = 78.0,
    end: float = 98.0,
    max_iterations: int = 5,
    retraction_threads: int = 8,
) -> KinodynamicSolution:
    """Run contact initialization and a coupled solve on original 50 Hz samples.

    start/end crop dataset seconds. Configurations use COM-to-world poses and
    measured joint angles; all velocities, accelerations, wrenches, torques and
    the shared IMU bias start at zero. The result owns its numerical Values.
    Timings include contact initialization, measurement setup and the native
    solve separately; output-array extraction and serialization are excluded.
    """
    prepared = time.perf_counter()
    walk, metadata = prepare_walk(directory, labels_path, start, end)
    metadata["preparationSeconds"] = time.perf_counter() - prepared
    settings: gtd.pose_contact_init.Options = gtd.pose_contact_init.Options()
    settings.selectedSlices = [gtd.Slice(k) for k in metadata["initializationSlices"]]
    settings.interpolateOutput = True
    settings.recordHistory = False
    settings.initializeUpright = True
    contact_rotation_sigma_degrees = 90.0
    upright_sigma_degrees = 15.0
    motion_rotation_sigma_degrees = 180.0
    settings.rotationSigma = float(np.deg2rad(contact_rotation_sigma_degrees))
    settings.addUprightFactors = True
    settings.uprightSigma = float(np.deg2rad(upright_sigma_degrees))
    settings.addBetweenFactors = True
    settings.betweenRotationSigma = float(np.deg2rad(motion_rotation_sigma_degrees))
    settings.betweenTranslationSigma = 10.0
    settings.addGroundFactors = True
    settings.addNormalFactors = False
    settings.groundHeight = 0.0
    settings.groundSigma = 0.1
    diagnostics: gtd.pose_contact_init.Result = gtd.pose_contact_init.Result()
    contact_started = time.perf_counter()
    started = contact_started
    initial: gtsam.Values = gtd.Initializer().InitializeSolutionContactPoses(
        walk.robot,
        walk.root.name(),
        walk.anchor,
        walk.measured,
        walk.trajectory,
        walk.dt,
        settings,
        diagnostics,
        0,
    )
    metadata["contactInitializationSeconds"] = time.perf_counter() - started
    metadata["contactInitializationIterations"] = diagnostics.iterations
    measurement_started = time.perf_counter()
    # Interpolation only constructs starting poses between sparse initializer
    # knots. Every original slice is subsequently an independent optimized state.
    initial = seed_contact_landmarks(walk, initial)
    encoders, contacts = gtsam.NonlinearFactorGraph(), gtsam.NonlinearFactorGraph()
    encoder_sigma_degrees = 0.5
    encoder_noise = gtsam.noiseModel.Isotropic.Sigma(
        1, np.deg2rad(encoder_sigma_degrees)
    )
    for k in range(walk.count):
        for joint in walk.robot.joints():
            key = gtd.JointAngleKey(joint.id(), k)
            encoders.add(
                gtsam.PriorFactorDouble(key, walk.measured.atDouble(key), encoder_noise)
            )
    contact_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([np.deg2rad(contact_rotation_sigma_degrees)] * 3 + [0.01] * 3)
    )
    for k, contact, key in stance_observations(walk):
        contacts.add(
            gtd.ContactPoseFactor(
                gtd.PoseKey(contact.link.id(), k),
                key,
                contact_noise,
                contact.comTcontact,
            )
        )
    imu: ImuAiding = build_imu_aiding(walk, directory)
    # Bias is also zero at the raw start. Its calibrated prior is a measurement,
    # not a warm start; the native solver creates all body/world v/a entries.
    initial.insert(BIAS_KEY, gtsam.imuBias.ConstantBias())
    model = load_g1_model()
    frames: dict[str, gtd.PoseOnLink] = contact_frames(model, walk.robot)
    patches: list[gtd.fixed_dynamics.Contact] = [
        gtd.fixed_dynamics.Contact(frames[side], *support_rectangle(model, side), 0.8)
        for side in ("left", "right")
    ]
    armature = np.zeros(model.njnt)
    for actuator in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator, 0])
        armature[joint_id] = model.dof_armature[model.jnt_dofadr[joint_id]]
    options: gtd.kinodynamics.Options = gtd.kinodynamics.Options()
    # The sparse initializer's gauge belongs to its first selected DS slice.
    # Dense output holds that pose before the selection starts; those earlier
    # configurations are guesses and must remain free in the coupled solve.
    options.gaugeSlice = metadata["initializationSlices"][0]
    metadata["gaugeSlice"] = options.gaugeSlice
    metadata["gaugeTimeSeconds"] = float(walk.times[options.gaugeSlice])
    options.maxIterations = max_iterations
    metadata["maxIterations"] = max_iterations
    options.retractionThreads = retraction_threads
    metadata["retractionThreads"] = retraction_threads
    metadata["measurementPreparationSeconds"] = (
        time.perf_counter() - measurement_started
    )
    started = time.perf_counter()
    result: gtd.kinodynamics.Result = gtd.solveKinodynamics(
        walk.robot,
        walk.trajectory,
        [gtd.Slice(k) for k in range(walk.count)],
        walk.times.tolist(),
        initial,
        [encoders, contacts, imu.factors],
        ["Encoders", "Contact poses", "Gal3 IMU and bias"],
        patches,
        armature,
        options,
    )
    metadata["coupledSolveSeconds"] = time.perf_counter() - started
    metadata["totalFromContactInitializationSeconds"] = (
        time.perf_counter() - contact_started
    )
    metadata["imu"] = imu.metadata
    metadata["collocation"] = {
        name: getattr(options, name)
        for name in (
            "poseRotationSigmaDegrees",
            "poseTranslationSigma",
            "twistAngularSigmaDegrees",
            "twistLinearSigma",
            "jointPositionSigmaDegrees",
            "jointVelocitySigmaDegrees",
            "stanceVelocitySigma",
        )
    }
    return KinodynamicSolution(walk, result, patches, initial, metadata)


def summary(solution: KinodynamicSolution) -> dict[str, Any]:
    """Portable timing and convergence report with explicit initial conditions."""
    result = solution.result
    return {
        **solution.metadata,
        "samples": solution.walk.count,
        "sampleRateHz": 50,
        "initialization": "Contact poses/joint angles; zero v/a/wrenches/torques/bias",
        "estimatedBias": result.values.atConstantBias(BIAS_KEY).vector().tolist(),
        **{
            name: getattr(result, name)
            for name in (
                "iterations",
                "assemblySeconds",
                "initializationSeconds",
                "optimizationSeconds",
                "objectiveInitial",
                "objectiveFinal",
                "initialEqualityL2",
                "equalityL2",
                "inequalityL2",
                "variableDimension",
                "reducedDimension",
                "equalityDimension",
                "inequalityDimension",
                "feasible",
            )
        },
        "errorNames": list(result.errorNames),
        "errorHistory": [v.tolist() for v in result.errorHistory],
        "wallSeconds": list(result.wallSeconds),
    }


def solution_arrays(solution: KinodynamicSolution) -> dict[str, NDArray[Any]]:
    """Extract slice-major arrays for offline replay, without changing the solve.

    Poses have shape (N, L, 4, 4), mapping each link COM into world coordinates.
    Link twists and their ordinary body-coordinate derivatives have shape
    (N, L, 6), angular first, in rad/s and m/s (or their time derivatives).
    Joint arrays have shape (N, J), in radians, rad/s, rad/s² and N m. Sole
    wrenches have shape (N, 2, 6), moment then force, in sole axes, N m and N.
    Contact and joint axis order follows contact_link_ids and joint_ids.
    """
    walk, result = solution.walk, solution.result
    joints = list(result.jointIds)
    return {
        "times": walk.times,
        "source_indices": walk.source_indices,
        "link_ids": np.array([link.id() for link in walk.robot.links()]),
        "joint_ids": np.array(joints),
        "poses": np.array(
            [
                [
                    result.values.atPose3(gtd.PoseKey(link.id(), k)).matrix()
                    for link in walk.robot.links()
                ]
                for k in range(walk.count)
            ]
        ),
        "initial_poses": np.array(
            [
                [
                    solution.initial.atPose3(gtd.PoseKey(link.id(), k)).matrix()
                    for link in walk.robot.links()
                ]
                for k in range(walk.count)
            ]
        ),
        "joint_names": np.array(
            [
                {joint.id(): joint.name() for joint in walk.robot.joints()}[j]
                for j in joints
            ]
        ),
        "initial_joint_angles": np.array(
            [
                [walk.measured.atDouble(gtd.JointAngleKey(j, k)) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "contact_transforms": np.array(
            [c.pose.comTcontact.matrix() for c in solution.contacts]
        ),
        "contact_link_ids": np.array([c.pose.link.id() for c in solution.contacts]),
        "contact": np.array(
            [
                [
                    any(c.link.id() == patch.pose.link.id() for c in active)
                    for patch in solution.contacts
                ]
                for _, active, _ in contact_slices(walk.trajectory)
            ]
        ),
        "reference_root_positions": walk.reference_root_positions,
        "root_id": np.array(walk.root.id()),
        "body_twists": np.array(
            [
                [gtd.Twist(result.values, link.id(), k) for link in walk.robot.links()]
                for k in range(walk.count)
            ]
        ),
        "body_twist_accelerations": np.array(
            [
                [
                    gtd.TwistAccel(result.values, link.id(), k)
                    for link in walk.robot.links()
                ]
                for k in range(walk.count)
            ]
        ),
        "joint_angles": np.array(
            [
                [gtd.JointAngle(result.values, j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "joint_velocities": np.array(
            [
                [gtd.JointVel(result.values, j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "joint_accelerations": np.array(
            [
                [gtd.JointAccel(result.values, j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "wrenches": np.array(
            [result.values.atVector(key) for key in result.contactKeys]
        ).reshape(walk.count, 2, 6),
        "joint_torques": np.array(
            [result.values.atDouble(key) for key in result.torqueKeys]
        ).reshape(walk.count, len(joints)),
    }


def argument_parser() -> argparse.ArgumentParser:
    """Shared command-line inputs for computation and browser serialization."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data",
        type=Path,
        default=REPO / "apps/kinematics-inspector/.cache/openio-contact-source",
    )
    parser.add_argument(
        "--labels",
        type=Path,
        default=REPO / "apps/contact-labels/public/snippets/middle.json.gz",
    )
    parser.add_argument("--start", type=float, default=78.0)
    parser.add_argument("--end", type=float, default=98.0)
    parser.add_argument("--max-iterations", type=int, default=5)
    parser.add_argument("--retraction-threads", type=int, default=8)
    return parser


def main() -> None:
    """Save a reproducible numerical result without browser dependencies."""
    parser = argument_parser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    solution = solve_walk(
        args.data,
        args.labels,
        start=args.start,
        end=args.end,
        max_iterations=args.max_iterations,
        retraction_threads=args.retraction_threads,
    )
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output / "kinodynamics.npz", **solution_arrays(solution))
    report = summary(solution)
    (args.output / "summary.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
