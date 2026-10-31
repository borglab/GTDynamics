#!/usr/bin/env python3
"""Record the G1 example's actual initializer and CM-Opt iterates for static replay.

All FK, optimization, residual evaluation, and COM/body conversion happen here.
The browser reads float32 body poses and joint angles; it runs no robot solver.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from numpy.typing import NDArray

REPO = Path(__file__).resolve().parents[3]
APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
import kinematic_walk as example
from mujoco_io import KinematicRecording, load_model, load_recording, pose


def compressed_json(path: Path, value: Any) -> None:
    path.write_bytes(
        gzip.compress(
            json.dumps(value, separators=(",", ":"), allow_nan=False).encode(), mtime=0
        )
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
        return {
            "poseOffset": self.append(transforms),
            "jointOffset": self.append(angles),
            "frames": len(samples),
        }

    def trajectory(self, values: gtsam.Values, **metadata: Any) -> dict[str, Any]:
        print(f"Recording {metadata.get('label', '')}", flush=True)
        state = self.state(values, list(range(self.walk.count)))
        state.update(metadata)
        state["metrics"] = example.evaluate(
            self.walk, self.costs, self.constraints, values
        )
        state["frameMetrics"] = self.frame_metrics(values)
        return state

    def frame_metrics(self, values: gtsam.Values) -> dict[str, list[float]]:
        height, slip, joint, root = [], [], [], []
        for k, points, continuing in example.contact_slices(self.walk.trajectory):
            heights = [float(p.predict(values, k)[2]) for p in points]
            slips = [
                float(np.linalg.norm(p.predict(values, k) - p.predict(values, k - 1)))
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


def dense_contact_state(
    walk: KinematicRecording,
    seed: gtsam.Values,
    samples: list[int],
    poses: list[gtsam.Pose3],
) -> gtsam.Values:
    """Apply the adapter's SE(3) interpolation to saved sparse root poses.

    Seed FK provides the fixed root-relative geometry from measured joints.
    Verify the final reconstruction against the adapter before publishing data.
    """
    result = gtsam.Values(seed)
    interval = 0
    for k in range(walk.count):
        while interval + 1 < len(samples) and samples[interval + 1] < k:
            interval += 1
        root_pose = poses[interval]
        if k > samples[0] and interval + 1 < len(samples):
            fraction = (k - samples[interval]) / (
                samples[interval + 1] - samples[interval]
            )
            root_pose = root_pose * gtsam.Pose3.Expmap(
                fraction * gtsam.Pose3.Logmap(root_pose.between(poses[interval + 1]))
            )
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
    parser.add_argument("--data", type=Path, default=APP / ".cache/jazzwalk")
    parser.add_argument(
        "--contacts",
        type=Path,
        default=APP / ".cache/jazzwalk/contacts.json",
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
    args = parser.parse_args()
    if args.max_iterations < 1 or (args.max_frames is not None and args.max_frames < 2):
        parser.error("Use positive iterations and at least two frames")
    walk = example.load_walk(args.data, args.contacts, args.max_frames)
    model = load_model(args.data)
    recorder = Recorder(walk, model)
    costs, constraints, keys = example.build_problem(walk)
    recorder.costs, recorder.constraints = costs, constraints
    runs = {}
    for method in ("contact", "ik"):
        contact_result = gtd.contact_init.Result() if method == "contact" else None
        ik_frames: list[list[dict[str, Any]]] = [[] for _ in range(walk.count)]

        def ik_iteration(
            sample: int, iteration: int, error: float, values: gtsam.Values
        ) -> None:
            state = recorder.state(values, [sample])
            state.update(
                iteration=iteration, error=error, label=f"IK iteration {iteration}"
            )
            ik_frames[sample].append(state)

        seed, info = example.initialize(
            walk, method, keys, contact_result, ik_iteration if method == "ik" else None
        )
        initializer = []
        sparse_samples: list[int] = []
        if contact_result is not None:
            sparse_samples = [
                measurement.slice.k for measurement in contact_result.graph.measurements
            ]
            for snapshot in contact_result.history:
                dense = dense_contact_state(walk, seed, sparse_samples, snapshot.poses)
                state = recorder.trajectory(
                    dense,
                    iteration=snapshot.iteration,
                    label=f"Contact iteration {snapshot.iteration}",
                    error=snapshot.objective / 2,
                    contactRms=snapshot.rms,
                )
                state["landmarks"] = [list(p) for p in snapshot.landmarks]
                initializer.append(state)
            if not dense.equals(seed, 1e-7):
                raise RuntimeError(
                    "Contact history interpolation differs from the native adapter"
                )
            assert len(initializer) == contact_result.iterations + 1
        else:
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

        smoothing = [
            recorder.trajectory(seed, iteration=None, label="Initializer output")
        ]

        def cm_iteration(iteration: int, values: gtsam.Values) -> None:
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
            costs, constraints, seed, walk.root.id(), args.max_iterations, cm_iteration
        )
        assert len(smoothing) == solver["iterations"] + 2, "Missing CM-Opt iterations"
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
            "smoothing": smoothing,
            "initializer": initializer,
            "ikFrames": ik_frames if method == "ik" else [],
            "sparseSamples": sparse_samples,
        }

    # Reference is rendered solely as an optional overlay, never fed to estimation.
    arrays, metadata = load_recording(args.data)
    data = mujoco.MjData(model)
    transforms = []
    for qpos in arrays["qpos"][: walk.count]:
        data.qpos[:] = qpos
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
                    for point in p.constraintSpec().contactPoints()
                }
            ),
        }
        for p in walk.trajectory.phases()
    ]
    source_files = [
        Path(__file__),
        Path(example.__file__),
        REPO / "examples/g1_mjlab/mujoco_io.py",
        REPO / "gtdynamics.i",
        REPO / "gtdynamics/utils/Initializer.cpp",
        REPO / "gtdynamics/utils/ContactTrajectoryInitializer.cpp",
        REPO / "gtdynamics/utils/ContactInitializer.cpp",
        REPO / "gtdynamics/kinematics/KinematicsTrajectory.cpp",
    ]
    if "motionSource" in metadata:
        source_files.extend(
            [
                REPO / "apps/contact-initialization/scripts/export_g1_moves.py",
                REPO / "apps/contact-initialization/scripts/contact_discovery.py",
            ]
        )
    record = {
        "schemaVersion": 1,
        "title": "G1 walking · kinematics smoothing",
        "motionSource": metadata.get("motionSource"),
        "samples": walk.count,
        "maxIterations": args.max_iterations,
        "dt": walk.dt,
        "bodies": model.nbody,
        "rootBody": walk.root.id(),
        "jointNames": [j.name() for j in recorder.joints],
        "phases": phases,
        "reference": reference,
        "runs": runs,
        "graph": {
            "costFactors": costs.size(),
            "equalities": constraints.size(),
            "inequalities": 0,
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
    export_model(model, args.output)
    args.output.joinpath("states.bin.gz").write_bytes(recorder.encoded_states())
    args.output.joinpath("recording.json").write_text(
        json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
    )
    print(
        f"Exported {walk.count} samples, {len(phases)} phases to {args.output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
