#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file export_recording.py
# @brief Plan, execute with simulated disturbances, and serialize a CMRG build for browser replay.
# @author GTDynamics contributors

"""Plan a post-and-lintel build offline and serialize it for the browser.

Each maneuver is planned with GPMP2 (RRT-Connect seed, the paper's factors
including the learned dress cables), executed state by state with a seeded
disturbance and replanned with ISAM2 measurement factors (STEAP), then sampled
into frames of link poses, tool tips, cables and member poses.
"""

from __future__ import annotations

import argparse
import gzip
import json
import pickle
from pathlib import Path
import subprocess
import time

import gtsam
import numpy as np

import cable_display as cd
import cell
import collision as col
import planner as pl
import scene as sc
import sequence as sq
import steap
import structure as st
import urdf_display as ud

TARGET_FRAMES = 36
DWELL_SECONDS = 1.0
APP_DIR = Path(__file__).resolve().parents[1]


class DisplayKinematics:
    """Pure-numpy link poses, tool frames and members for recording frames."""

    def __init__(self):
        self.model = ud.parse_urdf(cell.URDF_FILE)
        self.names = [n for arm in (1, 2) for n in cell.arm_joint_names(arm)]
        self.tcp = {
            arm: cell.tool_tcp_pose(cell.ARM_TOOL[arm]).matrix() for arm in (1, 2)
        }
        moving = {j.child for j in self.model.joints if j.kind != "fixed"}
        self.movable = [
            i for i, link in enumerate(self.model.links) if self._moves(link, moving)
        ]

    def _moves(self, link: str, moving: set[str]) -> bool:
        while link:
            if link in moving:
                return True
            joint = self.model._by_child.get(link)
            link = joint.parent if joint else ""
        return False

    def transforms(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Return world link transforms for a full 18-DoF configuration."""
        return ud.link_transforms(self.model, dict(zip(self.names, q)))

    def poses(self, world: dict[str, np.ndarray]) -> np.ndarray:
        """Return (len(movable), 7) ``[xyz, qxyzw]`` poses of the moving links."""
        out = np.zeros((len(self.movable), 7))
        for row, index in enumerate(self.movable):
            transform = world[self.model.links[index]]
            out[row, :3] = transform[:3, 3]
            out[row, 3:] = ud.matrix_to_quaternion_xyzw(transform[:3, :3])
        return out

    def tool_frames(self, world: dict[str, np.ndarray]) -> list[np.ndarray]:
        """Return each arm's tool working-point transform."""
        return [world[f"robot{arm}_tool0"] @ self.tcp[arm] for arm in (1, 2)]


def pose_row(matrix: np.ndarray) -> list[float]:
    """Return ``[x, y, z, qx, qy, qz, qw]`` of a 4x4 transform, rounded for JSON."""
    quaternion = ud.matrix_to_quaternion_xyzw(matrix[:3, :3])
    return [round(float(v), 4) for v in matrix[:3, 3]] + [
        round(float(v), 5) for v in quaternion
    ]


def hermite(q0, q1, v0, v1, dt, s):
    """Return the state between two support states at fraction ``s``: the constant-velocity GP mean."""
    h00, h10 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s
    h01, h11 = -2 * s**3 + 3 * s**2, s**3 - s**2
    return h00 * q0 + h10 * dt * v0 + h01 * q1 + h11 * dt * v1


class BuildRecorder:
    """Runs a structure's segments and accumulates the recording."""

    def __init__(
        self,
        structure: st.Structure,
        planner: pl.CmrgPlanner,
        disturbance: steap.Disturbance,
        seed: int,
    ):
        self.structure, self.planner, self.disturbance = structure, planner, disturbance
        self.rng = np.random.default_rng(seed)
        self.kin = DisplayKinematics()
        self.cable_display = cd.CableDisplay()
        self.poses = {
            m.name: (m.target.copy() if m.fixed else m.staging.copy())
            for m in structure.members
        }
        self.status = {
            m.name: ("placed" if m.fixed else "staged") for m in structure.members
        }
        self.member_index = {m.name: i for i, m in enumerate(structure.members)}
        self.q = planner.join([cell.park_configuration(1), cell.park_configuration(2)])
        self.time = 0.0
        self.frames = {
            "time": [],
            "q": [],
            "bodyPoses": [],
            "tips": [],
            "cable": [],
            "members": [],
        }
        self.segments: list[dict] = []
        self.field_cache: dict[tuple, object] = {}
        self.lift_configurations: dict[str, np.ndarray] = {}
        self.grasp = np.eye(4)
        self.planner.set_tables(structure.static_spheres())

    # -- world -----------------------------------------------------------
    def obstacles(self) -> list[sc.SphereSet]:
        wood = [
            m.spheres(self.poses[m.name])
            for m in self.structure.members
            if self.status[m.name] == "placed"
        ]
        return wood + self.planner.table_spheres

    def set_obstacles(self) -> None:
        names = tuple(
            sorted(
                m.name
                for m in self.structure.members
                if self.status[m.name] == "placed"
            )
        )
        key = (names, tuple(np.round(self.poses[n][:3, 3], 3).tobytes() for n in names))
        if key not in self.field_cache:
            self.planner.set_obstacles(self.obstacles())
            self.field_cache[key] = (
                self.planner.block_spheres,
                self.planner.block_field,
            )
        self.planner.block_spheres, self.planner.block_field = self.field_cache[key]

    def tool_world(self, q: np.ndarray, arm: int) -> np.ndarray:
        return self.kin.tool_frames(self.kin.transforms(q))[arm - 1]

    # -- goals -------------------------------------------------------------
    def goal_configuration(
        self, segment: sq.Segment, maneuver: pl.Maneuver
    ) -> np.ndarray:
        parts = self.planner.split(self.q)
        goal = [parts[0].copy(), parts[1].copy()]
        for index, target in enumerate(segment.goals):
            if isinstance(target, np.ndarray):
                goal[index] = target
        for index, target in enumerate(segment.goals):
            if isinstance(target, (sq.Blend, np.ndarray)) or target is None:
                continue
            goal[index] = self.planner.solve_ik(
                self.planner.specs[index],
                target,
                parts[index],
                goal[1 - index],
                maneuver,
                warm=True,
            )
        for index, target in enumerate(segment.goals):
            if isinstance(target, sq.Blend):
                goal[index] = self.blend_goal(
                    segment, index, target, parts, goal, maneuver
                )
        if segment.key.endswith(":carry"):
            member = next(m for m in self.structure.members if m.name == segment.member)
            nails = sq.member_nails(member, self.structure)
            if nails:
                # Arm 1 holds this pose through every nail, so it is reseeded until the bridges clear.
                goal[0] = self.planner.solve_hold(
                    self.planner.specs[0],
                    segment.goals[0],
                    self.nail_configurations(nails),
                    member.name,
                )
        return self.planner.join(goal)

    def nail_configurations(self, nails: list[gtsam.Pose3]) -> list[np.ndarray]:
        """Return arm 2's configurations at each nail: a cold first solve, then each warm from the last."""
        spec, arm1 = self.planner.specs[1], self.q[:9]
        configs = [self.planner.solve_ik(spec, nails[0], self.q[9:], arm1, warm=False)]
        for nail in nails[1:]:
            configs.append(
                self.planner.solve_ik(spec, nail, configs[-1], arm1, warm=True)
            )
        return configs

    def prepare_blocks(self, segments: list[sq.Segment]) -> None:
        """Group segments sharing a blended goal and record each block's member segments in order."""
        self.blocks: dict[str, dict] = {}
        for segment in segments:
            for index, target in enumerate(segment.goals):
                if isinstance(target, sq.Blend):
                    self.blocks.setdefault(
                        target.block, {"arm": index, "segments": [], "end": target.end}
                    )["segments"].append(segment)

    def begin_block(self, name: str, maneuver: pl.Maneuver) -> dict:
        """Solve a block's end configuration and fractions along the other arm's tool travel."""
        block = self.blocks[name]
        if "start" in block:
            return block
        arm = block["arm"]
        parts = self.planner.split(self.q)
        block["start"] = parts[arm].copy()
        end = block["end"]
        block["end_q"] = (
            end
            if isinstance(end, np.ndarray)
            else self.planner.solve_ik(
                self.planner.specs[arm],
                end,
                parts[arm],
                parts[1 - arm],
                maneuver,
                warm=False,
            )
        )
        position, distances = self.tool_world(self.q, 2 - arm)[:3, 3], []
        for segment in block["segments"]:
            target = segment.goals[1 - arm]
            if isinstance(target, gtsam.Pose3):
                point = np.asarray(target.translation())
            elif isinstance(target, np.ndarray):
                q = self.q.copy()
                q[9 * (1 - arm) : 9 * (1 - arm) + 9] = target
                point = self.tool_world(q, 2 - arm)[:3, 3]
            else:
                point = position
            distances.append(max(float(np.linalg.norm(point - position)), 0.15))
            position = point
        block["fractions"] = (np.cumsum(distances) / np.sum(distances)).tolist()
        return block

    def blend_goal(self, segment, index, target, parts, goal, maneuver) -> np.ndarray:
        block = self.begin_block(target.block, maneuver)
        slot = next(i for i, s in enumerate(block["segments"]) if s is segment)
        fraction = block["fractions"][slot]
        previous = block["fractions"][slot - 1] if slot else 0.0
        other = goal[1 - index]
        for candidate in np.linspace(fraction, previous, 6):
            q = block["start"] + candidate * (block["end_q"] - block["start"])
            full = self.planner.join([q, other] if index == 0 else [other, q])
            if self.planner.collision_free(
                full, maneuver, pad=False, check_carried=False
            ):
                return q
        return parts[index]

    def via_configurations(
        self, segment, maneuver, goal
    ) -> tuple[np.ndarray, np.ndarray]:
        """Solve arm 1's lifted poses at both ends; arm 2 moves proportionally between its ends."""
        spec = self.planner.specs[0]
        first, last = segment.rrt_via
        lift = self.lift_configurations.get(segment.member)
        arm2_first = self.q[9:] + 0.25 * (goal[9:] - self.q[9:])
        arm2_last = self.q[9:] + 0.75 * (goal[9:] - self.q[9:])
        arm1_first = (
            lift
            if lift is not None
            else self.planner.solve_ik(
                spec, first, self.q[:9], arm2_first, maneuver, warm=True
            )
        )
        return (
            self.planner.join([arm1_first, arm2_first]),
            self.planner.join(
                [
                    self.planner.solve_ik(
                        spec, last, goal[:9], arm2_last, maneuver, warm=True
                    ),
                    arm2_last,
                ]
            ),
        )

    def maneuver_for(self, segment: sq.Segment) -> pl.Maneuver:
        settings = self.planner.settings
        maneuver = pl.Maneuver(
            straight=segment.straight,
            block_clearance=segment.block_clearance,
            use_rrt=segment.use_rrt,
            contact=segment.contact,
            state_count=(
                settings.nail_state_count if segment.nailing else settings.state_count
            ),
            check_interpolated=(
                settings.nail_check_interpolated
                if segment.nailing
                else settings.check_interpolated
            ),
        )
        if segment.carried:
            member = next(m for m in self.structure.members if m.name == segment.member)
            points, radius = col.carried_points(
                self.planner.specs[0], member.size, member.grip_offset
            )
            maneuver.carried_by, maneuver.carried_points, maneuver.carried_radius = (
                1,
                points,
                radius,
            )
            maneuver.carried_other_only = segment.carried_other_only
        return maneuver

    # -- run -------------------------------------------------------------
    def signature(self, segments: list[sq.Segment], count: int) -> str:
        """Identify the first ``count`` segments plus the settings, so a stale checkpoint is not reused."""
        return json.dumps(
            [
                [s.key for s in segments[:count]],
                vars(self.planner.settings),
                vars(self.disturbance),
                self.structure.name,
                cell.CABLE_ANCHORS,
                st.TABLE_TOP_Z,
                self.structure.stock_distance,
                "split-tables-flush-slack-park-lm-margin-via-hover-guard-lintel-16nails-gp-playback-wait-lift-time-still-grasp-no-extra-priors-solve-hold",
            ],
            sort_keys=True,
            default=str,
        )

    def save_checkpoint(
        self, path: Path, segments: list[sq.Segment], count: int
    ) -> None:
        state = {
            "signature": self.signature(segments, count),
            "count": count,
            "q": self.q,
            "time": self.time,
            "poses": self.poses,
            "status": self.status,
            "frames": self.frames,
            "segments": self.segments,
            "rng": self.rng.bit_generator.state,
            "lift": self.lift_configurations,
            "grasp": self.grasp,
            "blocks": {
                k: {f: v for f, v in b.items() if f in ("start", "end_q", "fractions")}
                for k, b in self.blocks.items()
            },
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_bytes(pickle.dumps(state))
        temporary.replace(path)

    def load_checkpoint(self, path: Path, segments: list[sq.Segment]) -> int:
        """Restore a matching checkpoint and return how many segments it already holds (else 0)."""
        if not path.exists():
            return 0
        state = pickle.loads(path.read_bytes())
        if state["signature"] != self.signature(segments, state["count"]):
            print(
                "checkpoint does not match this sequence or settings; starting over",
                flush=True,
            )
            return 0
        self.q, self.time, self.poses, self.status = (
            state["q"],
            state["time"],
            state["poses"],
            state["status"],
        )
        self.frames, self.segments = state["frames"], state["segments"]
        self.rng.bit_generator.state = state["rng"]
        self.lift_configurations, self.grasp = state["lift"], state["grasp"]
        for name, cached in state["blocks"].items():
            self.blocks[name].update(cached)
        print(f"resumed {state['count']} segments from {path}", flush=True)
        return state["count"]

    def run(
        self,
        segments: list[sq.Segment],
        limit: int | None = None,
        checkpoint: Path | None = None,
    ) -> None:
        self.prepare_blocks(segments)
        first = self.load_checkpoint(checkpoint, segments) if checkpoint else 0
        for number, segment in enumerate(segments[:limit]):
            if number < first:
                continue
            started = time.time()
            self.run_segment(number, segment)
            print(
                f"[{number + 1}/{len(segments)}] {segment.label}: {time.time() - started:.1f}s",
                flush=True,
            )
            if checkpoint:
                self.save_checkpoint(checkpoint, segments, number + 1)

    def run_segment(self, number: int, segment: sq.Segment) -> None:
        member = next(m for m in self.structure.members if m.name == segment.member)
        if segment.event == "grip_open" and segment.event_at == "start":
            self.seat(member)
            self.time += DWELL_SECONDS
        carried = member if self.status[member.name] == "carried" else None
        self.set_obstacles()
        maneuver = self.maneuver_for(segment)
        goal = self.goal_configuration(segment, maneuver)
        if segment.rrt_via is not None:
            maneuver.rrt_via = self.via_configurations(segment, maneuver, goal)
        t0 = time.time()
        plan = self.planner.plan(self.q, goal, maneuver)
        plan_seconds = time.time() - t0
        t0 = time.time()
        run = steap.execute(
            self.planner, plan, self.disturbance, self.rng, feedback=True
        )
        steap_seconds = time.time() - t0
        self.record(segment, member, carried, plan, run, plan_seconds, steap_seconds)
        self.q = run.measured[-1].copy()
        if segment.key.endswith(":reach"):
            self.lift_configurations[member.name] = plan.states()[-1][:9].copy()
        if segment.key.endswith(":carry"):
            self.seat(member)
        if segment.event == "grip_close":
            # The gripper closes on the wood where it lies, so tracking error cannot move the wood.
            self.grasp = (
                np.linalg.inv(self.tool_world(self.q, 1)) @ self.poses[member.name]
            )
            self.status[member.name] = "carried"
        if segment.event and segment.event_at == "end":
            self.time += DWELL_SECONDS

    def seat(self, member: st.Member) -> None:
        """Seat a member at its target, flush against its neighbours whatever the tool's tracking error."""
        self.poses[member.name] = member.target.copy()
        self.status[member.name] = "placed"

    # -- recording ---------------------------------------------------------
    def member_poses(
        self, tool_frame: np.ndarray | None, carried: st.Member | None
    ) -> list[list[float]]:
        rows = []
        for m in self.structure.members:
            pose = (
                (tool_frame @ self.grasp)
                if (carried is not None and m.name == carried.name)
                else self.poses[m.name]
            )
            rows.append(pose_row(pose))
        return rows

    def tips_of(self, q: np.ndarray) -> list[list[float]]:
        frames = self.kin.tool_frames(self.kin.transforms(q))
        return [[round(float(v), 3) for v in f[:3, 3]] for f in frames]

    def cable_points(self, q: np.ndarray) -> list[list[list[float]]]:
        world = self.kin.transforms(q)
        return [
            self.cable_display.samples(world, q, arm, pl.CABLE_SAMPLES)
            .round(3)
            .tolist()
            for arm in (1, 2)
        ]

    def record(
        self, segment, member, carried, plan, run, plan_seconds, steap_seconds
    ) -> None:
        n, dt = plan.state_count, plan.delta_t
        sub = max(2, int(np.ceil(TARGET_FRAMES / (n - 1))))
        frame_start = len(self.frames["time"])
        velocities, executed = run.velocities, []
        for k in range(n):
            last = k == n - 1
            for j in range(1 if last else sub):
                s = j / sub
                q = (
                    run.measured[k]
                    if last
                    else hermite(
                        run.measured[k],
                        run.measured[k + 1],
                        velocities[k],
                        velocities[k + 1],
                        dt,
                        s,
                    )
                )
                world = self.kin.transforms(q)
                tools = self.kin.tool_frames(world)
                self.frames["time"].append(round(self.time + dt * (k + s), 3))
                executed.append(q)
                self.frames["q"].append([round(float(v), 5) for v in q])
                self.frames["bodyPoses"].append(self.kin.poses(world).round(4).tolist())
                self.frames["tips"].append(
                    [[round(float(v), 3) for v in f[:3, 3]] for f in tools]
                )
                self.frames["cable"].append(self.cable_points(q))
                self.frames["members"].append(self.member_poses(tools[0], carried))
        events = (
            [{"at": segment.event_at, "type": segment.event}] if segment.event else []
        )
        clearance = [self.planner.clearances(q) for q in executed]
        self.segments.append(
            {
                "key": segment.key,
                "label": segment.label,
                "member": member.name,
                "t0": round(self.time, 3),
                "duration": round(plan.total_time, 3),
                "frameStart": frame_start,
                "frameCount": len(self.frames["time"]) - frame_start,
                "events": events,
                "planned": [self.tips_of(q) for q in run.nominal],
                "replans": [
                    [self.tips_of(q) for q in estimate] for estimate in run.estimates
                ],
                "graphError": round(plan.error, 4),
                "supportStates": n,
                "subFrames": sub,
                "seed": plan.seed_kind,
                "planSeconds": round(plan_seconds, 2),
                "steapSeconds": round(steap_seconds, 2),
                "finalJointError": round(run.final_error, 5),
                "toolError": self.tool_error(run),
                "clearance": {
                    key: round(min(c[key] for c in clearance), 4)
                    for key in ("floor", "wood", "table", "cross_arm")
                },
                "disturbanceMax": round(
                    float(np.max(np.abs(run.measured - run.commanded))), 5
                ),
                "replanShiftMax": round(
                    float(np.max(np.abs(run.commanded - run.nominal))), 4
                ),
            }
        )
        self.time += plan.total_time

    def tool_error(self, run) -> list[float]:
        end_nominal = self.kin.tool_frames(self.kin.transforms(run.nominal[-1]))
        end_measured = self.kin.tool_frames(self.kin.transforms(run.measured[-1]))
        return [
            round(float(np.linalg.norm(a[:3, 3] - b[:3, 3])), 5)
            for a, b in zip(end_nominal, end_measured)
        ]

    def nail_records(self) -> list[dict]:
        """Return each fired nail's head on the wood surface, driving direction and firing frame."""
        fired = {segment["key"]: segment for segment in self.segments}
        nails = []
        for member in self.structure.members:
            for i, goal in enumerate(sq.member_nails(member, self.structure)):
                segment = fired.get(f"{member.name}:nail:{i}")
                if segment is None:
                    continue
                direction = goal.rotation().matrix()[:, 2]
                embed = sq.NAIL_EMBED if member.kind == "lintel" else 0.0
                head = np.asarray(goal.translation()) - embed * direction
                nails.append(
                    {
                        "member": member.name,
                        "frame": segment["frameStart"] + segment["frameCount"] - 1,
                        "head": head.round(4).tolist(),
                        "direction": direction.round(5).tolist(),
                    }
                )
        return nails

    def to_json(self) -> dict:
        s = self.structure
        return {
            "schemaVersion": 1,
            "kind": "steap-cmrg",
            "structure": {
                "name": s.name,
                "spec": s.spec,
                "members": [
                    {
                        "name": m.name,
                        "kind": m.kind,
                        "size": list(m.size),
                        "fixed": m.fixed,
                        "target": pose_row(m.target),
                        "staging": None if m.staging is None else pose_row(m.staging),
                    }
                    for m in s.members
                ],
                "tables": [
                    {
                        "name": k,
                        "center": b.center.round(4).tolist(),
                        "half": b.half.round(4).tolist(),
                    }
                    for k, b in s.tables.items()
                ],
                "stockDistance": s.stock_distance,
            },
            "bodies": {
                "names": self.kin.model.links,
                "movable": self.kin.movable,
                "static": [
                    pose_row(t)
                    for t in (
                        self.kin.transforms(np.zeros(18))[l]
                        for l in self.kin.model.links
                    )
                ],
            },
            "settings": {
                "planner": vars(self.planner.settings),
                "disturbance": vars(self.disturbance),
                "measurementSigma": {
                    "gantryMeters": steap.MEASUREMENT_SIGMA_GANTRY_M,
                    "armDegrees": steap.MEASUREMENT_SIGMA_ARM_DEG,
                },
                "cableRadius": pl.CABLE_RADIUS,
            },
            "segments": self.segments,
            "nails": self.nail_records(),
            "frames": self.frames,
        }


def display_model() -> dict:
    """Return display geometry for the cell with both tools attached to their tool0 links."""
    model = ud.parse_urdf(cell.URDF_FILE)
    extras = {
        f"robot{arm}_tool0": [
            ud.Visual(
                cell.TOOL_MESHES[cell.ARM_TOOL[arm]],
                np.zeros(3),
                np.zeros(3),
                (0.32, 0.34, 0.36, 1.0),
            )
        ]
        for arm in (1, 2)
    }
    return ud.display_geometry(model, extras)


def write_gzip_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode()
    path.write_bytes(gzip.compress(data, mtime=0))
    print(f"wrote {path} ({path.stat().st_size / 1e6:.2f} MB)")


NOTICE = """BAR Lab CMRG model notice

The robot meshes (ABB IRB6620 arms, gantry bridges, timber gripper and nailer) and the learned dress-cable
weights come from the Bruun Automation Research Lab (Georgia Institute of Technology):
https://bruun-automation-research-lab.github.io/BAR-Documentation/index.html
They are used here to visualize and plan the cell; ABB retains any rights in its robot geometry. The primary
source for this work is CAST, Shaji et al., arXiv:2609.24841. STEAP is Mukadam et al., arXiv:1807.10425.
"""


def update_presets(output_dir: Path, preset: str, title: str) -> None:
    """Add or replace this preset in presets.json, which lists the recordings the app can open."""
    path = output_dir / "presets.json"
    entries = json.loads(path.read_text()) if path.exists() else []
    entries = [entry for entry in entries if entry["id"] != preset]
    entries.append(
        {"id": preset, "title": title, "file": f"recording-{preset}.json.gz"}
    )
    path.write_text(
        json.dumps(sorted(entries, key=lambda entry: entry["id"]), indent=2) + "\n"
    )
    (output_dir / "MODEL_NOTICE.txt").write_text(NOTICE)


def revision() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=APP_DIR, text=True
        ).strip()
    except Exception:
        return "unknown"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", default="portal", choices=sorted(st.PRESETS))
    parser.add_argument("--output-dir", type=Path, default=APP_DIR / "public")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--lm-iterations", type=int, default=250)
    parser.add_argument("--gantry-sigma-mm", type=float, default=10.0)
    parser.add_argument("--arm-sigma-deg", type=float, default=0.10)
    parser.add_argument("--max-segments", type=int, default=None)
    parser.add_argument("--model-only", action="store_true")
    parser.add_argument(
        "--no-checkpoint",
        action="store_true",
        help="ignore and do not write the resume checkpoint",
    )
    args = parser.parse_args()
    if args.model_only:
        write_gzip_json(args.output_dir / "model.json.gz", display_model())
        return
    structure = st.build_structure(st.PRESETS[args.preset])
    settings = pl.Settings(lm_iterations=args.lm_iterations)
    planner = pl.CmrgPlanner(cell.load_robot(), settings)
    disturbance = steap.Disturbance(
        args.gantry_sigma_mm / 1000.0, args.arm_sigma_deg, seed=args.seed
    )
    recorder = BuildRecorder(structure, planner, disturbance, args.seed)
    checkpoint = (
        None
        if args.no_checkpoint
        else APP_DIR / ".cache" / f"checkpoint-{args.preset}.pkl"
    )
    recorder.run(sq.build_sequence(structure), args.max_segments, checkpoint)
    payload = recorder.to_json()
    payload["source"] = {"revision": revision(), "preset": args.preset}
    write_gzip_json(args.output_dir / "model.json.gz", display_model())
    write_gzip_json(args.output_dir / f"recording-{args.preset}.json.gz", payload)
    update_presets(args.output_dir, args.preset, structure.name)


if __name__ == "__main__":
    main()
