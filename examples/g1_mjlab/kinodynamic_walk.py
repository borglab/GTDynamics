#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_walk.py
# @brief Solve G1 kinodynamics on selected single- and double-support slices.
# @author GTDynamics contributors
"""Solve the SSIO mid-clip turns on sparse support slices using native CMC-Opt.

The saved Contact Labels max-product schedule supplies stance phases. The default
runs the native contact -> GP -> dynamics pipeline. Use --initialization contact
for contact-pose fitting with zero motion and forces, or fk for measured-joint
FK at a fixed upright pelvis with gravity-support forces.
The shared IMU bias starts at zero. Native
feasibility restoration is timed separately for either seed.

Run: python examples/g1_mjlab/kinodynamic_walk.py --output /tmp/kinodynamics
Use --end 79 --max-iterations 2 for a small development run. Computation lives
here and in the private example helper; the app exporter serializes the results.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from contact_phases import contact_slices
from imu_aiding import BIAS_KEY
from kinodynamic_initialization import initialize_walk
from kinodynamic_preparation import prepare_walk
from kinodynamic_problem import (
    dynamics_model,
    prepare_measurements,
    solve_coupled_problem,
)
from kinodynamic_settings import configure_solve
from mujoco_io import KinematicRecording

REPO = Path(__file__).resolve().parents[2]


@dataclass
class KinodynamicSolution:
    """Support-slice solution, initial pose estimate, physical model and timings."""

    walk: KinematicRecording
    result: dict[str, Any]
    contacts: list[gtd.fixed_dynamics.Contact]
    initial: gtsam.Values
    metadata: dict[str, Any]


def solve_walk(
    directory: Path,
    labels_path: Path,
    *,
    start: float = 78.0,
    end: float = 98.0,
    max_iterations: int = 5,
    retraction_threads: int = 8,
    initialization: str = "contact-gp-dynamics",
) -> KinodynamicSolution:
    """Initialize and jointly optimize G1 poses, motion, and forces at support knots.

    Args:
        directory: Recording directory containing encoders, reference, and IMU.
        labels_path: Saved max-product support labels used to construct phases.
        start: First dataset time in seconds (inclusive).
        end: Last dataset time in seconds (inclusive).
        max_iterations: Maximum coupled CMC-Opt iterations after restoration.
        retraction_threads: Number of workers for component retractions.
        initialization: "fk" uses fixed-pelvis FK and gravity-support forces;
            "contact" fits contact poses with zero motion and forces;
            "contact-gp-dynamics" adds GP interpolation and a linear dynamics seed (default).

    Returns:
        The prepared clip, initial COM-to-world poses, native optimized values,
        sole patches, and timing metadata. The contact-gp-dynamics strategy retains fitted
        velocities, accelerations, torques and wrenches. Only the selected
        single- and double-support slices are optimized. Preparation, initialization,
        measurement assembly, and native solve are timed separately; array
        extraction and serialization are excluded. Total initialization/solve
        time starts after clip preparation and includes measurement assembly.
    """
    # Load the measured clip, support phases, and root coordinate gauge.
    prepared = time.perf_counter()
    walk, metadata = prepare_walk(directory, labels_path, start, end, initialization)
    metadata["preparationSeconds"] = time.perf_counter() - prepared

    # Define physical inputs, then run the complete native initialization.
    patches, armature = dynamics_model(walk)
    initialization_started = time.perf_counter()
    initial, initialization_metadata = initialize_walk(
        walk, metadata["initializationSlices"], initialization, patches, armature
    )
    metadata["configurationInitializationSeconds"] = (
        time.perf_counter() - initialization_started
    )
    metadata.update(initialization_metadata)
    metadata["initializationStrategy"] = initialization

    # Add observations, shared stance landmarks, and canonical contact patches.
    measurement_started = time.perf_counter()
    initial, measurements, imu_metadata = prepare_measurements(walk, directory, initial)
    gauge_slice = metadata["initializationSlices"][0]
    options, solver_metadata = configure_solve(
        initialization,
        gauge_slice,
        float(walk.times[gauge_slice]),
        max_iterations,
        retraction_threads,
    )
    metadata.update(solver_metadata)
    metadata["measurementPreparationSeconds"] = (
        time.perf_counter() - measurement_started
    )

    # Restore feasibility, then jointly optimize the same sparse support knots.
    started = time.perf_counter()
    result = solve_coupled_problem(
        walk, initial, measurements, patches, armature, options
    )
    metadata["coupledSolveSeconds"] = time.perf_counter() - started
    metadata["totalInitializationAndSolveSeconds"] = (
        time.perf_counter() - initialization_started
    )
    metadata["imu"] = imu_metadata
    return KinodynamicSolution(walk, result, patches, initial, metadata)


def summary(solution: KinodynamicSolution) -> dict[str, Any]:
    """Portable timing and convergence report with explicit initial conditions."""
    result = solution.result
    return {
        **solution.metadata,
        "samples": solution.walk.count,
        "sampleRateHz": None,
        "sourceSampleRateHz": 50,
        "initialization": (
            "Measured-joint FK at a fixed upright pelvis; gravity-support forces; "
            "zero velocities, accelerations, joint torques and bias"
            if solution.metadata["initializationStrategy"] == "fk"
            else (
                "Contact poses/joint angles; zero v/a/wrenches/torques/bias"
                if solution.metadata["initializationStrategy"] == "contact"
                else "Contact poses, GP q/v, affine linear dynamics seed; zero IMU bias"
            )
        ),
        "estimatedBias": result["values"].atConstantBias(BIAS_KEY).vector().tolist(),
        **{
            name: result[name]
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
        "errorNames": list(result["errorNames"]),
        "errorHistory": [v.tolist() for v in result["errorHistory"]],
        "wallSeconds": list(result["wallSeconds"]),
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
    joints = list(result["jointIds"])
    return {
        "times": walk.times,
        "source_indices": walk.source_indices,
        "link_ids": np.array([link.id() for link in walk.robot.links()]),
        "joint_ids": np.array(joints),
        "poses": np.array(
            [
                [
                    result["values"].atPose3(gtd.PoseKey(link.id(), k)).matrix()
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
                [gtd.JointAngle(solution.initial, j, k) for j in joints]
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
                [
                    gtd.Twist(result["values"], link.id(), k)
                    for link in walk.robot.links()
                ]
                for k in range(walk.count)
            ]
        ),
        "body_twist_accelerations": np.array(
            [
                [
                    gtd.TwistAccel(result["values"], link.id(), k)
                    for link in walk.robot.links()
                ]
                for k in range(walk.count)
            ]
        ),
        "joint_angles": np.array(
            [
                [gtd.JointAngle(result["values"], j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "joint_velocities": np.array(
            [
                [gtd.JointVel(result["values"], j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "joint_accelerations": np.array(
            [
                [gtd.JointAccel(result["values"], j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "initial_wrenches": np.array(
            [result["initialValues"].atVector(key) for key in result["contactKeys"]]
        ).reshape(walk.count, 2, 6),
        "wrenches": np.array(
            [result["values"].atVector(key) for key in result["contactKeys"]]
        ).reshape(walk.count, 2, 6),
        "initial_joint_velocities": np.array(
            [
                [gtd.JointVel(result["initialValues"], j, k) for j in joints]
                for k in range(walk.count)
            ]
        ),
        "initial_body_twists": np.array(
            [
                [
                    gtd.Twist(result["initialValues"], link.id(), k)
                    for link in walk.robot.links()
                ]
                for k in range(walk.count)
            ]
        ),
        "initial_joint_torques": np.array(
            [result["initialValues"].atDouble(key) for key in result["torqueKeys"]]
        ).reshape(walk.count, len(joints)),
        "joint_torques": np.array(
            [result["values"].atDouble(key) for key in result["torqueKeys"]]
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
    parser.add_argument(
        "--initialization",
        choices=("fk", "contact", "contact-gp-dynamics"),
        default="contact-gp-dynamics",
        help="contact-gp-dynamics (default): contact -> GP -> linear dynamics seed; contact: contact poses and zero motion/forces; fk: fixed-pelvis FK with gravity support",
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
        initialization=args.initialization,
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
