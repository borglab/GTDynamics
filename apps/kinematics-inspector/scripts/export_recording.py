#!/usr/bin/env python3
"""Record the G1 example's actual initializer and CM-Opt iterates for static replay.

All FK, optimization, residual evaluation, and COM/body conversion happen here.
The browser reads float32 body poses and joint angles; it runs no robot solver.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from time import perf_counter
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from numpy.typing import NDArray

REPO = Path(__file__).resolve().parents[3]
APP = Path(__file__).resolve().parents[1]
FACTOR_TYPES = {
    "joint": ("Joint measurements", "PriorFactor<double>", "0.03 rad"),
    "height": ("Stance landmark height", "ContactHeightFactor", "0.1 m; world Z=0"),
    "contact": ("Stance pose observations", "ContactPoseFactor", "10 mm / 90°"),
    "upright": ("Root upright", "UprightFactor", "15°; yaw free"),
    "anchor": ("Initial root pose", "PriorFactor<Pose3>", "0.000573° / 1e-5 m"),
    "imu": (
        "IMU preintegration",
        "GalileanImuFactor",
        "propagated preintegration covariance",
    ),
    "bias": (
        "Shared IMU bias",
        "PriorFactor<ConstantBias>",
        "accel 0.5 m/s²; gyro 0.0001 rad/s",
    ),
    "velocity": ("Initial COM velocity", "PriorFactor<Vector3>", "3 m/s"),
    "closure": ("Return to start", "PoseTranslationPrior3D", "XY 0.30 m; Z 0.05 m"),
}
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
import kinematic_walk as example
from contact_phases import contact_slices, initialization_slices, stance_observations
from kinematic_initialization import align_contact_ground
from g1_model import load_g1_model
from mujoco_io import KinematicRecording, load_recording, pose
from imu_aiding import BIAS_KEY, build_imu_aiding
from pack_recordings import write_final_bundle, write_replay_bundles

# Use precisely the same calibrated prior preset as Contact Initialization.
sys.path.insert(0, str(REPO / "apps/contact-initialization/scripts"))
from export_native_recordings import prior_configurations


def compressed_json(path: Path, value: Any) -> None:
    path.write_bytes(
        gzip.compress(
            json.dumps(value, separators=(",", ":"), allow_nan=False).encode(), mtime=0
        )
    )


def loop_closure_factor(walk: KinematicRecording) -> gtsam.PoseTranslationPrior3D:
    """Prior the final root COM position to the anchored start, in world axes.

    Sigmas are 30 cm in world X/Y and 5 cm in Z, with no orientation cost.
    The first pose already has a tight prior to walk.anchor. This unary factor
    preserves the graph's chain structure. It is a manually supplied return to
    that anchor, not a measurement from the final LIO pose.
    """
    return gtsam.PoseTranslationPrior3D(
        gtd.PoseKey(walk.root.id(), walk.count - 1),
        walk.anchor.translation(),
        gtsam.noiseModel.Diagonal.Sigmas(np.array([0.30, 0.30, 0.05])),
    )


class Recorder:
    """Serialize exact link poses, including infeasible IK seeds, without rerunning FK."""

    def __init__(self, walk: KinematicRecording, model: mujoco.MjModel) -> None:
        self.walk = walk
        self.model = model
        self.joints = list(walk.robot.joints())
        self.link_ids = {link.id() for link in walk.robot.links()}
        self.com_to_body = [
            pose(model.body_ipos[i], model.body_iquat[i]).inverse()
            for i in range(model.nbody)
        ]
        self.parent_to_body = [
            pose(model.body_pos[i], model.body_quat[i]) for i in range(model.nbody)
        ]
        self.chunks: list[bytes] = []
        self.offset = 0
        self.factor_groups: dict[str, gtsam.NonlinearFactorGraph] = {}

    def append(self, array: NDArray[Any]) -> int:
        data = np.asarray(array, dtype="<f4")
        if not np.isfinite(data).all():
            raise ValueError("Cannot export nonfinite state")
        offset = self.offset
        self.chunks.append(data.tobytes())
        self.offset += data.size
        return offset

    def encoded_states(self) -> bytes:
        """Delta-code equal-shaped snapshots, then shuffle bytes for gzip.

        Render coordinates are rounded to 1e-6 (at most 0.5 micrometre for
        positions). Residuals and objectives remain full-precision JSON values.
        """
        previous: dict[int, NDArray[np.int32]] = {}
        encoded = []
        for chunk in self.chunks:
            data = np.frombuffer(chunk, dtype="<f4").astype(np.float64)
            if np.max(np.abs(data)) * 1e6 >= 2**30:
                raise ValueError("Recording exceeds the fixed-point codec range")
            quantized = np.rint(data * 1e6).astype("<i4")
            encoded.append(quantized - previous.get(len(data), 0))
            previous[len(data)] = quantized
        shuffled = np.concatenate(encoded).view(np.uint8).reshape(-1, 4).T.tobytes()
        return gzip.compress(shuffled, mtime=0)

    def body_poses(self, values: gtsam.Values, sample: int) -> NDArray[np.float64]:
        bodies = [gtsam.Pose3()]
        for body in range(1, self.model.nbody):
            if body in self.link_ids:
                world = (
                    values.atPose3(gtd.PoseKey(body, sample)) * self.com_to_body[body]
                )
            else:
                world = (
                    bodies[int(self.model.body_parentid[body])]
                    * self.parent_to_body[body]
                )
            bodies.append(world)
        return np.asarray(
            [
                list(p.translation()) + list(p.rotation().toQuaternion().coeffs())
                for p in bodies
            ]
        )

    def state(self, values: gtsam.Values, samples: list[int]) -> dict[str, Any]:
        transforms = np.stack([self.body_poses(values, k) for k in samples])
        angles = np.array(
            [
                [values.atDouble(gtd.JointAngleKey(j.id(), k)) for j in self.joints]
                for k in samples
            ]
        )
        state = {
            "poseOffset": self.append(transforms),
            "jointOffset": self.append(angles),
            "frames": len(samples),
        }
        if values.exists(BIAS_KEY):
            state["velocityOffset"] = self.append(
                np.array(
                    [
                        values.atVector(gtd.LinearVelocityKey(self.walk.root.id(), k))
                        for k in samples
                    ]
                )
            )
            bias: gtsam.imuBias.ConstantBias = values.atConstantBias(BIAS_KEY)
            state["bias"] = bias.vector().tolist()
        return state

    def trajectory(self, values: gtsam.Values, **metadata: Any) -> dict[str, Any]:
        print(f"Recording {metadata.get('label', '')}", flush=True)
        state = self.state(values, list(range(self.walk.count)))
        state.update(metadata)
        state["metrics"] = example.evaluate(
            self.walk, self.costs, self.constraints, values
        )
        state["frameMetrics"] = self.frame_metrics(values)
        if self.factor_groups:
            state["factorErrors"] = {
                name: graph.error(values) for name, graph in self.factor_groups.items()
            }
            np.testing.assert_allclose(
                sum(state["factorErrors"].values()), state["metrics"]["cost"], rtol=1e-9
            )
            state["contactSlipOffset"] = self.append(self.contact_slip(values))
        state["endpointDisplacementWorld"] = (
            values.atPose3(
                gtd.PoseKey(self.walk.root.id(), self.walk.count - 1)
            ).translation()
            - values.atPose3(gtd.PoseKey(self.walk.root.id(), 0)).translation()
        ).tolist()
        state["closurePositionErrorWorld"] = (
            values.atPose3(
                gtd.PoseKey(self.walk.root.id(), self.walk.count - 1)
            ).translation()
            - self.walk.anchor.translation()
        ).tolist()
        return state

    def contact_slip(self, values: gtsam.Values) -> NDArray[np.float64]:
        """Record the continuing contact-frame origin displacement per foot and slice.

        Each row stores previous XYZ, current XYZ, and displacement in metres.
        A displacement of -1 means no continuing contact. Frames are world axes;
        no contact geometry or forward kinematics is recomputed in the browser.
        """
        slips = np.zeros((self.walk.count, 2, 7))
        slips[:, :, 6] = -1
        for k, _, continuing in contact_slices(
            self.walk.trajectory, self.walk.contact_breaks
        ):
            for point in continuing:
                foot = 0 if point.link.name().startswith("left") else 1
                previous = point.predict(values, k - 1).translation()
                current = point.predict(values, k).translation()
                distance = float(np.linalg.norm(current - previous))
                if distance > slips[k, foot, 6]:
                    slips[k, foot] = np.r_[previous, current, distance]
        return slips

    def frame_metrics(self, values: gtsam.Values) -> dict[str, list[float]]:
        height, slip, joint, root = [], [], [], []
        for k, points, continuing in contact_slices(
            self.walk.trajectory, self.walk.contact_breaks
        ):
            heights = [float(p.predict(values, k).translation()[2]) for p in points]
            slips = [
                float(
                    np.linalg.norm(
                        p.predict(values, k).translation()
                        - p.predict(values, k - 1).translation()
                    )
                )
                for p in continuing
            ]
            height.append(
                float(np.sqrt(np.mean(np.square(heights)))) if heights else 0.0
            )
            slip.append(float(np.sqrt(np.mean(np.square(slips)))) if slips else 0.0)
            errors = [
                values.atDouble(gtd.JointAngleKey(j.id(), k))
                - self.walk.measured.atDouble(gtd.JointAngleKey(j.id(), k))
                for j in self.joints
            ]
            joint.append(float(np.sqrt(np.mean(np.square(errors)))))
            root.append(
                float(
                    np.linalg.norm(
                        values.atPose3(
                            gtd.PoseKey(self.walk.root.id(), k)
                        ).translation()
                        - self.walk.reference_root_positions[k]
                    )
                )
            )
        return {"height": height, "slip": slip, "joint": joint, "root": root}


def contact_iteration_values(
    walk: KinematicRecording,
    seed: gtsam.Values,
    poses: list[gtsam.Pose3],
) -> gtsam.Values:
    """Replay root poses at exactly the selected slices, using measured-joint FK."""
    if len(poses) != walk.count:
        raise ValueError("Contact iterations must use the retained solve slices")
    result = gtsam.Values(seed)
    for k in range(walk.count):
        root_pose = poses[k]
        correction = root_pose * seed.atPose3(gtd.PoseKey(walk.root.id(), k)).inverse()
        for link in walk.robot.links():
            key = gtd.PoseKey(link.id(), k)
            result.update(key, correction * seed.atPose3(key))
    return result


def export_model(model: mujoco.MjModel, output: Path) -> None:
    # Same compiled-model contract consumed by apps/shared/mujocoRobot.js.
    fields = (
        "geom_group",
        "geom_type",
        "geom_size",
        "geom_dataid",
        "geom_matid",
        "geom_rgba",
        "geom_pos",
        "geom_quat",
        "geom_bodyid",
        "mat_rgba",
        "mesh_vertadr",
        "mesh_vertnum",
        "mesh_faceadr",
        "mesh_facenum",
        "mesh_vert",
        "mesh_face",
    )
    data: dict[str, Any] = {"nbody": model.nbody, "ngeom": model.ngeom}
    data.update(
        {key: np.asarray(getattr(model, key)).ravel().tolist() for key in fields}
    )
    compressed_json(output / "model.json.gz", data)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=APP / ".cache/openio")
    parser.add_argument(
        "--methods", nargs="+", choices=["contact", "ik"], default=["contact", "ik"]
    )
    parser.add_argument(
        "--smoothing-only",
        action="store_true",
        help="Do not capture initializer histories",
    )
    parser.add_argument(
        "--final-only", action="store_true", help="Keep only final results"
    )
    parser.add_argument(
        "--double-support-slices",
        action="store_true",
        default=True,
        help="Infer binary contact phases and solve only representative double-stance slices",
    )
    parser.add_argument(
        "--contact-slices",
        action="store_true",
        help="Estimate only the contact initializer's slices, with a one-source-slice stance buffer",
    )
    parser.add_argument(
        "--imu",
        action="store_true",
        help="Add Gal3 IMU factors, world COM velocities, and one shared bias",
    )
    parser.add_argument(
        "--loop-closure",
        action="store_true",
        help="Close full-trajectory root COM positions: world XYZ sigmas 0.30, 0.30, 0.05 m",
    )
    parser.add_argument(
        "--contact-translation-sigma",
        type=float,
        default=0.01,
        help="Contact observation translation sigma in metres (default: 0.01)",
    )
    parser.add_argument(
        "--contacts",
        type=Path,
        default=APP / ".cache/openio/contacts.json",
    )
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).resolve().parents[1] / "public"
    )
    parser.add_argument(
        "--max-frames", type=int, help="Diagnostic prefix; omit for the entire snippet"
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=10,
        help="CM-Opt iteration limit for each initialization",
    )
    parser.add_argument("--upright-prior", action="store_true")
    parser.add_argument("--height-prior", action="store_true")
    parser.add_argument("--max-slice-interval-ms", type=float, default=200.0)
    args = parser.parse_args()
    if args.double_support_slices:
        args.contact_slices = True
    if args.final_only:
        args.methods = ["contact"]
        args.smoothing_only = True
    if args.max_iterations < 1 or (args.max_frames is not None and args.max_frames < 2):
        parser.error("Use positive iterations and at least two frames")
    walk = example.load_walk(args.data, args.contacts, args.max_frames)
    input_arrays, input_metadata = load_recording(args.data)
    if args.loop_closure and (
        not input_metadata.get("motionSource", {}).get("fullTrajectory")
        or args.max_frames is not None
    ):
        parser.error("Loop closure requires the entire recording, without --max-frames")
    _, between, ground, contact_options = next(
        preset
        for preset in prior_configurations()
        if preset[0] == "between-ground-upright"
    )
    contact_options.addBetweenFactors = between
    contact_options.addGroundFactors = ground
    contact_options.maxIterations = 200
    contact_options.groundHeight = 0.0
    phase_info = None
    if args.double_support_slices:
        walk, phase_info = example.infer_contact_phases(walk, args.data)
    model = load_g1_model()
    anchor_slice = 0
    if args.double_support_slices:
        selected = initialization_slices(
            walk.trajectory, walk.dt, args.max_slice_interval_ms
        )
        if not selected:
            raise ValueError("No double-support slice for the initial gauge")
        anchor_slice = selected[0].k
        qpos = input_arrays["qpos"][anchor_slice]
        body = model.body("robot/pelvis").id
        anchor = pose(qpos[:3], qpos[3:7]) * pose(
            model.body_ipos[body], model.body_iquat[body]
        )
        walk = replace(walk, anchor=anchor)
    walk, world_translation = align_contact_ground(walk, anchor_slice)
    initialization_frame = {
        "anchorSourceIndex": anchor_slice,
        "worldTranslationMeters": world_translation.tolist(),
        "inputWorldTranslationMeters": list(
            input_metadata["motionSource"]["worldTranslationMeters"]
        ),
        "groundSource": "Mean active sole height from measured FK at the first initialized slice",
    }
    selected_seed = None
    selection_info = None
    selected_diagnostics = (
        gtd.pose_contact_init.Result() if not args.smoothing_only else None
    )
    if args.contact_slices:
        walk, selected_seed, selection_info = example.initialize_contact_slices(
            walk,
            contact_options,
            contact_buffer=1,
            double_support_only=args.double_support_slices,
            max_slice_interval_ms=args.max_slice_interval_ms,
            contact_diagnostics=selected_diagnostics,
        )
    recorder = Recorder(walk, model)
    groups: dict[str, gtsam.NonlinearFactorGraph] = {}
    costs, constraints, keys = example.build_problem(
        walk,
        args.contact_translation_sigma,
        groups,
        upright_prior=args.upright_prior,
        height_prior=args.height_prior,
    )
    kinematic_costs = gtsam.NonlinearFactorGraph(costs)
    aiding = build_imu_aiding(walk, args.data) if args.imu else None
    if aiding is not None:
        costs.push_back(aiding.factors)
        for name, start, end in (
            ("imu", 0, walk.count - 1),
            ("bias", walk.count - 1, walk.count),
            ("velocity", walk.count, walk.count + 1),
        ):
            group = gtsam.NonlinearFactorGraph()
            for index in range(start, end):
                group.add(aiding.factors.at(index))
            groups[name] = group
    if args.loop_closure:
        closure = gtsam.NonlinearFactorGraph()
        closure.add(loop_closure_factor(walk))
        costs.push_back(closure)
        groups["closure"] = closure
    recorder.constraints = constraints
    runs = {}
    for method in args.methods:
        # Initializer histories solve the same kinematic problem in both modes.
        recorder.costs = kinematic_costs
        recorder.factor_groups = {}
        contact_result = selected_diagnostics if method == "contact" else None
        ik_frames: list[list[dict[str, Any]]] = [[] for _ in range(walk.count)]

        def ik_iteration(
            sample: int, iteration: int, error: float, values: gtsam.Values
        ) -> None:
            state = recorder.state(values, [sample])
            state.update(
                iteration=iteration, error=error, label=f"IK iteration {iteration}"
            )
            ik_frames[sample].append(state)

        if selected_seed is not None and method == "contact":
            seed = example.seed_contact_landmarks(walk, selected_seed).extract(keys)
            info = selection_info
        else:
            seed, info = example.initialize(
                walk,
                method,
                keys,
                contact_result,
                ik_iteration if method == "ik" and not args.smoothing_only else None,
                contact_options=contact_options,
            )
        initializer = []
        sparse_samples: list[int] = []
        if contact_result is not None:
            sparse_samples = list(range(walk.count))
            for snapshot in contact_result.history:
                dense = contact_iteration_values(walk, seed, snapshot.poses)
                state = recorder.trajectory(
                    dense,
                    iteration=snapshot.iteration,
                    label=f"Contact iteration {snapshot.iteration}",
                    error=snapshot.objective / 2,
                    contactRms=snapshot.rms,
                )
                state["landmarks"] = [
                    p.translation().tolist() for p in snapshot.contactPoses
                ]
                initializer.append(state)
            if not dense.equals(seed, 1e-7):
                raise RuntimeError(
                    "Contact history differs from the selected native solution"
                )
            assert len(initializer) == contact_result.iterations + 1
        elif method == "ik" and not args.smoothing_only:
            assert all(
                history and history[0]["iteration"] == 0 for history in ik_frames
            )
            buffer = b"".join(recorder.chunks)
            for k, history in enumerate(ik_frames):
                expected = recorder.body_poses(seed, k)
                recorded = np.frombuffer(
                    buffer,
                    dtype="<f4",
                    count=model.nbody * 7,
                    offset=history[-1]["poseOffset"] * 4,
                ).reshape(model.nbody, 7)
                np.testing.assert_allclose(recorded, expected, atol=2e-6)

        if aiding is not None:
            seed = aiding.initialize(walk, seed)
        recorder.costs = costs
        recorder.factor_groups = groups
        smoothing = [
            recorder.trajectory(seed, iteration=None, label="Initializer output")
        ]

        error_history: list[dict[str, Any]] = []
        started = perf_counter()

        def cm_iteration(iteration: int, values: gtsam.Values) -> None:
            # Retain scalar objective histories even when intermediate robot
            # poses are omitted. All groups are evaluated at the same Values.
            errors = {name: graph.error(values) for name, graph in groups.items()}
            error_history.append(
                {
                    "iteration": iteration,
                    "elapsedSeconds": perf_counter() - started,
                    "factorErrors": errors,
                    "total": sum(errors.values()),
                }
            )
            if not args.final_only:
                smoothing.append(
                    recorder.trajectory(
                        values,
                        iteration=iteration,
                        label=(
                            "CM-Opt feasible start"
                            if iteration == 0
                            else f"CM-Opt iteration {iteration}"
                        ),
                    )
                )

        solution, solver = example.solve_cm(
            costs,
            constraints,
            seed,
            walk.root.id(),
            args.max_iterations,
            cm_iteration,
        )
        elapsed = perf_counter() - started
        solver["elapsedSeconds"] = elapsed
        solver["timingScope"] = (
            "CM-Opt manifold setup, optimization, solution extraction, and "
            "per-iteration factor-error evaluation; excludes initialization "
            "and graph construction"
            + (
                " and robot replay serialization"
                if args.final_only
                else "; includes intermediate replay serialization"
            )
        )
        assert len(error_history) == solver["iterations"] + 1
        if args.final_only:
            smoothing.append(
                recorder.trajectory(
                    solution,
                    iteration=solver["iterations"],
                    label=f"CM-Opt iteration {solver['iterations']}",
                )
            )
        else:
            assert (
                len(smoothing) == solver["iterations"] + 2
            ), "Missing CM-Opt iterations"
        np.testing.assert_allclose(
            error_history[-1]["total"], costs.error(solution), rtol=1e-9
        )
        final_metrics = example.evaluate(walk, costs, constraints, solution)
        if final_metrics["kinematic_equality_l2"] >= 1e-5:
            raise RuntimeError(f"{method}: infeasible final state")
        for name, value in final_metrics.items():
            np.testing.assert_allclose(
                smoothing[-1]["metrics"][name], value, rtol=1e-8, atol=1e-10
            )
        runs[method] = {
            "initialization": info,
            "solver": solver,
            "errorHistory": error_history,
            "smoothing": smoothing,
            "initializer": initializer,
            "ikFrames": ik_frames if method == "ik" and not args.smoothing_only else [],
            "sparseSamples": sparse_samples,
        }

    # Reference poses are an overlay, not pose measurements in the smoother.
    # The optional privileged contact graph has already used them for labels.
    arrays, metadata = input_arrays, input_metadata
    if phase_info is not None:
        motion = metadata["motionSource"]
        motion["detector"] = {
            "method": phase_info["method"],
            "details": "contactPhaseInference",
            "privilegedReference": True,
        }
        for credit in motion.get("licenseNotes", []):
            credit["changes"] += (
                " For this replay, replaced the imported height/speed schedule "
                "with binary-foot max-product labels and retained only "
                "double-stance slices for initialization and smoothing."
            )
    metadata["motionSource"]["contactInitialization"] = {
        "addBetweenFactors": True,
        "addUprightFactors": True,
        "addGroundFactors": True,
        "addNormalFactors": False,
        "initializeUpright": True,
        "frame": initialization_frame,
    }
    metadata["motionSource"]["worldTranslationMeters"] = (
        np.asarray(metadata["motionSource"]["worldTranslationMeters"])
        + world_translation
    ).tolist()
    for credit in metadata["motionSource"].get("licenseNotes", []):
        credit[
            "changes"
        ] += " Applied one constant vertical shift from the first initialized sole frames to both estimation and reference replay."
    if metadata.get("motionSource", {}).get("imu"):
        metadata["motionSource"]["imu"]["usedBySmoother"] = args.imu
        metadata["motionSource"]["imu"].pop("file", None)
    data = mujoco.MjData(model)
    transforms = []
    source_indices = (
        np.arange(walk.count) if walk.source_indices is None else walk.source_indices
    )
    for qpos in arrays["qpos"][source_indices]:
        data.qpos[:] = qpos
        data.qpos[:3] += world_translation
        mujoco.mj_forward(model, data)
        transforms.append(
            np.concatenate([data.xpos, data.xquat[:, [1, 2, 3, 0]]], axis=1)
        )
    reference = {
        "poseOffset": recorder.append(np.asarray(transforms)),
        "frames": walk.count,
    }
    measured = np.array(
        [
            [
                walk.measured.atDouble(gtd.JointAngleKey(j.id(), k))
                for j in recorder.joints
            ]
            for k in range(walk.count)
        ]
    )
    reference["jointOffset"] = recorder.append(measured)
    phases = [
        {
            "start": p.k_start,
            "end": p.k_end,
            "contacts": sorted(
                {
                    point.link.name().replace("_ankle_roll_link", "")
                    for point in p.constraintSpec().contactPoses()
                }
            ),
        }
        for p in walk.trajectory.phases()
    ]
    source_files = [
        Path(__file__),
        Path(example.__file__),
        REPO / "examples/g1_mjlab/contact_phases.py",
        REPO / "examples/g1_mjlab/kinematic_initialization.py",
        REPO / "examples/g1_mjlab/kinematic_smoothing.py",
        REPO / "apps/contact-initialization/scripts/export_native_recordings.py",
        REPO / "examples/g1_mjlab/mujoco_io.py",
        REPO / "gtdynamics.i",
        REPO / "gtdynamics/utils/Initializer.cpp",
        REPO / "gtdynamics/utils/PoseContactInitializer.cpp",
        REPO / "gtdynamics/utils/PoseContactInitializer.h",
        REPO / "gtdynamics/utils/ContactMotion.cpp",
        REPO / "examples/g1_mjlab/g1_model.py",
        REPO / "gtdynamics/utils/ContactInitializer.cpp",
        REPO / "gtdynamics/kinematics/KinematicsTrajectory.cpp",
        REPO / "gtdynamics/kinematics/Kinematics.h",
        REPO / "gtdynamics/utils/values.h",
        REPO / "examples/g1_mjlab/imu_aiding.py",
    ]
    if "motionSource" in metadata:
        source_files.extend(
            [
                REPO / "apps/contact-initialization/scripts/export_g1_moves.py",
                REPO / "apps/contact-initialization/scripts/contact_discovery.py",
            ]
        )
    if metadata.get("motionSource", {}).get("imu"):
        source_files.append(APP / "scripts/import_openio.py")
    equality_graph = constraints.penaltyGraph()
    equality_dimension = sum(
        equality_graph.at(i).dim() for i in range(equality_graph.size())
    )
    record = {
        "schemaVersion": 1,
        "availableStages": (
            ["smoothing"] if args.smoothing_only else ["smoothing", "initializer"]
        ),
        "title": "G1 walking · kinematics smoothing",
        "motionSource": metadata.get("motionSource"),
        "contactPhaseInference": phase_info,
        "contactInitialization": (
            runs["contact"]["initialization"] if "contact" in runs else None
        ),
        "initializationFrame": initialization_frame,
        "imuAiding": aiding.metadata if aiding is not None else {"enabled": False},
        "loopClosure": {
            "enabled": args.loop_closure,
            "factor": "PoseTranslationPrior3D",
            "sample": walk.count - 1,
            "point": "root link COM",
            "residual": "p_finish - anchored_start_position, world XYZ",
            "targetWorldMeters": walk.anchor.translation().tolist(),
            "positionSigmasMeters": [0.30, 0.30, 0.05],
            "orientationConstraint": "none",
            "source": "manually supplied return-to-start observation; not inferred from LIO",
        },
        "samples": walk.count,
        "maxIterations": args.max_iterations,
        "dt": walk.dt,
        "timestamps": walk.times.tolist(),
        "sampling": {
            "mode": (
                "double-support"
                if args.double_support_slices
                else "contact-initializer" if args.contact_slices else "uniform"
            ),
            "longPhaseSeconds": (
                min(0.2, args.max_slice_interval_ms / 1000.0)
                if args.double_support_slices
                else None
            ),
            "interpolatedSolveStates": False,
            "sourceIndices": source_indices.tolist(),
            "sourcePeriodSeconds": walk.dt,
            "contactBuffer": 1 if args.contact_slices else 0,
            "maxSliceIntervalMs": (
                args.max_slice_interval_ms if args.double_support_slices else None
            ),
            "contactBreaks": sorted(walk.contact_breaks),
            "intervalSeconds": np.diff(walk.times).tolist(),
        },
        "bodies": model.nbody,
        "bodyNames": [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            for i in range(model.nbody)
        ],
        "rootBody": walk.root.id(),
        "jointNames": [j.name() for j in recorder.joints],
        "phases": phases,
        "reference": reference,
        "runs": runs,
        "graph": {
            "costFactors": costs.size(),
            "contactTranslationSigmaMeters": args.contact_translation_sigma,
            "contactRotationSigmaDegrees": example.CONTACT_ROTATION_SIGMA_DEGREES,
            "stanceLandmarks": len({key for _, _, key in stance_observations(walk)}),
            "uprightPrior": args.upright_prior,
            "heightPrior": args.height_prior,
            "motionPrior": False,
            "equalities": constraints.size(),
            "inequalities": 0,
            "factorGroups": [
                {
                    "id": name,
                    "label": FACTOR_TYPES[name][0],
                    "factorType": FACTOR_TYPES[name][1],
                    "noise": (
                        f"translation {args.contact_translation_sigma:g} m; "
                        f"rotation {example.CONTACT_ROTATION_SIGMA_DEGREES:g}° per observation"
                        if name == "contact"
                        else FACTOR_TYPES[name][2]
                    ),
                    "count": graph.size(),
                    "dimension": graph.at(0).dim() if graph.size() else 0,
                    "residualDimension": sum(
                        graph.at(i).dim() for i in range(graph.size())
                    ),
                }
                for name, graph in groups.items()
            ],
            "equalityDimension": equality_dimension,
        },
        "slipVisualization": {
            "feet": ["left", "right"],
            "stride": 7,
            "layout": "previous XYZ, current XYZ, displacement metres; -1 means inactive",
            "frame": "world",
            "measurement": "largest displacement among continuing sole corners, per adjacent slice interval",
        },
        "provenance": {
            "revision": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=REPO, text=True
            ).strip(),
            "inputs": walk.provenance,
            "sources": {
                str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in source_files
            },
            "module": hashlib.sha256(
                Path(gtd.gtdynamics.__file__).read_bytes()
            ).hexdigest(),
        },
        "storage": {
            "codec": "delta-shuffle-int32-v1",
            "scale": 1000000,
            "blocks": [len(chunk) // 4 for chunk in recorder.chunks],
            "poseOrder": "tx ty tz qx qy qz qw",
            "offsetUnits": "floats",
            "states": "states.bin.gz",
            "model": "model.json.gz",
        },
        "notes": "Every completed outer LM iterate is saved, including iteration zero. Internal damping trials and CM-Opt feasibility retraction subiterations are not separate optimizer iterates. IK histories are per sample; contact initialization jointly solves sparse frames, with SE(3) interpolation for display. Export runs collect history and are not timing benchmarks.",
    }
    args.output.mkdir(parents=True, exist_ok=True)
    motion_source = metadata.get("motionSource", {})
    if motion_source.get("imu"):
        shutil.copyfile(
            args.data / "OPENIO_ATTRIBUTION.txt", args.output / "OPENIO_ATTRIBUTION.txt"
        )
        record["provenance"]["inputs"]["imu.json"] = hashlib.sha256(
            (args.data / "imu.json").read_bytes()
        ).hexdigest()
    export_model(model, args.output)
    args.output.joinpath("states.bin.gz").write_bytes(recorder.encoded_states())
    args.output.joinpath("recording.json").write_text(
        json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
    )
    if args.final_only:
        write_final_bundle(args.output)
    else:
        write_replay_bundles(args.output)
    print(
        f"Exported {walk.count} samples, {len(phases)} phases to {args.output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
