#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file planner.py
# @brief Two-arm GPMP2 planner for the CMRG with RRT-Connect seeding and NN cable factors.
# @author GTDynamics contributors

"""Offline GPMP2 trajectory planning for the two CMRG arms.

One maneuver is a joint-space trajectory over both arms (18 degrees of freedom:
gantry axes then arm joints, arm 1 then arm 2) represented by ``X(k)``/``V(k)``
at ``state_count`` support states. The graph and tuning follow the BAR Lab
paper planner: Gaussian-process motion priors, joint and velocity limits,
signed-distance factors against the floor and placed wood, cross-arm and bridge
self-collision, the carried plank, and a learned neural-network dress cable per
arm, each also evaluated at interpolated states. Initialization is RRT-Connect.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import gtdynamics as gtd
import gtsam
import numpy as np
from gtsam.symbol_shorthand import V, X
from scipy import optimize

import cell
import collision as col
import rrt
import scene as sc

CABLE_WEIGHTS = {
    1: Path(__file__).resolve().parents[3]
    / "models"
    / "cable"
    / "cable_model_arm1_gripper.txt",
    2: Path(__file__).resolve().parents[3]
    / "models"
    / "cable"
    / "cable_model_arm2_nailer.txt",
}
CABLE_RADIUS = 3.0 * 0.0254 / 2.0
CABLE_SAMPLES = 57
CONVERGENCE_ERROR = 3.3e4
LM_ERROR_TOLERANCE = 1e-10
ENDPOINT_RADIUS = 0.15
# Contact poses: tool spheres overlap a table by up to 6.9 cm where the tool meshes clear it.
TABLE_CONTACT_SLACK = 0.07
# RRT sampling margin per planning attempt, as a fraction of joint range; widens only on failure.
RRT_BOX_MARGINS = (0.2, 0.2, 0.5, 0.5, 1.0, 1.0)


@dataclass
class Settings:
    """Planner tuning, defaulting to the BAR Lab paper values."""

    state_count: int = 10
    nail_state_count: int = 2
    check_interpolated: int = 5
    nail_check_interpolated: int = 5
    lm_iterations: int = 250
    cost_sigma: float = 0.1
    limit_sigma: float = 0.001
    velocity_limit_sigma: float = 0.001
    velocity_limit_margin: float = 0.1
    floor_clearance: float = 0.5
    block_clearance: float = 0.1
    cross_arm_clearance: float = 0.1
    joint_limit_margin_deg: float = 5.0
    min_total_time: float = 0.5
    velocity_time_fraction: float = 0.5
    rrt_step: float = 0.05
    rrt_resolution: float = 0.02
    rrt_iterations: int = 5000
    rrt_seed: int = 0
    gantry_seed_sigma: float = 0.05
    vertical_gantry_sigma: float = 0.0005


@dataclass
class Maneuver:
    """Per-maneuver context: carried plank, straight-pinned arms and wood clearance."""

    carried_by: int | None = None
    carried_points: list | None = None
    carried_radius: float = 0.0
    carried_other_only: bool = False
    straight: tuple[bool, bool] = (False, False)
    block_clearance: float | None = None
    use_rrt: bool = True
    contact: bool = False
    state_count: int | None = None
    check_interpolated: int | None = None
    rrt_via: tuple[np.ndarray, np.ndarray] | None = None


@dataclass
class Plan:
    """A solved maneuver: states and velocities with the graph that produced them."""

    graph: gtsam.NonlinearFactorGraph
    values: gtsam.Values
    error: float
    total_time: float
    delta_t: float
    state_count: int
    seed_kind: str = "rrt"

    def states(self) -> np.ndarray:
        """Return the (state_count, dof) support configurations."""
        return np.array([self.values.atVector(X(k)) for k in range(self.state_count)])

    def velocities(self) -> np.ndarray:
        """Return the (state_count, dof) support velocities."""
        return np.array([self.values.atVector(V(k)) for k in range(self.state_count)])


def read_cable_metadata(path: Path) -> dict[str, list[str]]:
    """Read the header (before ``layers``) of a cable model file."""
    metadata = {}
    for line in path.read_text(encoding="ascii").splitlines():
        words = line.split()
        if not words or words[0].startswith("#"):
            continue
        if words[0] == "layers":
            break
        metadata[words[0]] = words[1:]
    return metadata


def build_cable(
    robot: gtd.Robot, arm: int, joint_names: list[str], block_start: int
) -> gtd.RobotNNCableModel:
    """Build one arm's learned cable spline between the Grasshopper canvas anchors."""
    path = CABLE_WEIGHTS[arm]
    metadata = read_cable_metadata(path)

    def anchor(link: str) -> gtd.PointOnLink:
        link_name = f"robot{arm}_{link}"
        point = cell.link_centroid_offset(link_name).transformTo(
            cell.CABLE_ANCHORS[arm][link]
        )
        return gtd.PointOnLink(robot.link(link_name), np.asarray(point))

    spline = gtd.NNCableSpline(
        gtd.MLP(str(path)), int(metadata["cheb_nodes"][0]), CABLE_SAMPLES
    )
    joints = [robot.joint(name) for name in joint_names]
    inputs = [block_start + i for i in (4, 5, 6, 7, 8)]
    return gtd.RobotNNCableModel(
        robot,
        cell.BASE_LINK,
        joints,
        anchor("link_3"),
        anchor("link_6"),
        robot.link(f"robot{arm}_link_1"),
        spline,
        inputs,
    )


class CmrgPlanner:
    """Plans coordinated two-arm GPMP2 maneuvers in the tabletop scene."""

    def __init__(self, robot: gtd.Robot, settings: Settings | None = None):
        self.robot = robot
        self.settings = settings or Settings()
        self.specs = cell.make_arm_specs(robot)
        self.joint_names = [n for spec in self.specs for n in spec.q_names]
        self.dof = len(self.joint_names)
        self.models = col.build_collision_models(robot, self.joint_names)
        self.cables = [
            build_cable(robot, spec.arm, self.joint_names, 9 * i)
            for i, spec in enumerate(self.specs)
        ]
        self.floor = sc.floor_field()
        self.block_spheres: list[sc.SphereSet] = []
        self.block_field = None
        self.table_spheres: list[sc.SphereSet] = []
        self.table_field = None
        self._configure_limits()

    # -- configuration ---------------------------------------------------
    def _configure_limits(self) -> None:
        s = self.settings
        self.lower = np.array(
            [spec.limits[n][0] for spec in self.specs for n in spec.q_names]
        )
        self.upper = np.array(
            [spec.limits[n][1] for spec in self.specs for n in spec.q_names]
        )
        margin = np.deg2rad(s.joint_limit_margin_deg)
        pad = np.concatenate([[0, 0, 0] + [margin] * 6 for _ in self.specs])
        self.factor_lower = self.lower + pad
        self.factor_upper = np.maximum(self.upper - pad, self.factor_lower)
        self.velocity_limit = np.array(
            [spec.vel_limits[n] for spec in self.specs for n in spec.q_names]
        )
        self.qc_model = gtsam.noiseModel.Isotropic.Sigma(self.dof, 1.0)
        self.endpoint_model = gtsam.noiseModel.Isotropic.Sigma(self.dof, 1e-4)
        self.limit_model = gtsam.noiseModel.Isotropic.Sigma(self.dof, s.limit_sigma)
        self.velocity_model = gtsam.noiseModel.Isotropic.Sigma(
            self.dof, s.velocity_limit_sigma
        )
        self.limit_threshold = np.full(self.dof, 0.01)
        self.joint_id = {joint.name(): joint.id() for joint in self.robot.joints()}

    def set_obstacles(self, obstacles: list[sc.SphereSet]) -> None:
        """Set the placed-wood and table spheres that arms, cables and the carried plank avoid."""
        self.block_spheres = list(obstacles)
        self.block_field = sc.sphere_field(self.block_spheres)

    # -- geometry --------------------------------------------------------
    def split(self, q: np.ndarray) -> list[np.ndarray]:
        """Split a full configuration into per-arm blocks."""
        return [q[:9], q[9:]]

    def join(self, parts: list[np.ndarray]) -> np.ndarray:
        """Join per-arm blocks into one configuration."""
        return np.concatenate(parts)

    def _sphere_gap(self, points: np.ndarray, radii: np.ndarray) -> float:
        if not self.block_spheres or len(points) == 0:
            return 1e9
        return float(
            np.min(sc.sphere_signed_distance(points, self.block_spheres) - radii)
        )

    def set_tables(self, tables: list[sc.SphereSet]) -> None:
        """Set the tables, which stay obstacles in contact moves that exempt the wood."""
        self.table_spheres = list(tables)
        self.table_field = sc.sphere_field(self.table_spheres)

    def table_gap(self, points: np.ndarray, radii: np.ndarray) -> float:
        """Return the least gap between spheres and the tables, m."""
        if not self.table_spheres or len(points) == 0:
            return np.inf
        return float(
            np.min(sc.sphere_signed_distance(points, self.table_spheres) - radii)
        )

    def collision_free(
        self,
        q: np.ndarray,
        maneuver: Maneuver | None = None,
        pad: bool = True,
        check_carried: bool = True,
        only_arm: int | None = None,
    ) -> bool:
        """Whether spheres clear the floor and wood (RRT feasibility); cables and arms are not checked."""
        s, q = self.settings, np.asarray(q, dtype=float)
        floor_points = np.asarray(self.models.floor_model.worldPoints(q)).T
        floor_radii = self.models.floor_radii
        if only_arm is not None:
            keep = (
                slice(None, self.models.floor_split)
                if only_arm == 1
                else slice(self.models.floor_split, None)
            )
            floor_points, floor_radii = floor_points[keep], floor_radii[keep]
        if np.any(
            floor_points[:, 2] - floor_radii < (s.floor_clearance if pad else 0.0)
        ):
            return False
        block_margin = (
            (
                maneuver.block_clearance
                if maneuver and maneuver.block_clearance is not None
                else s.block_clearance
            )
            if pad
            else 0.0
        )
        points = np.asarray(self.models.model.worldPoints(q)).T
        radii = self.models.radii
        if only_arm is not None:
            keep = (
                slice(None, self.models.self_split)
                if only_arm == 1
                else slice(self.models.self_split, None)
            )
            points, radii = points[keep], radii[keep]
        if maneuver and maneuver.contact:
            return self.table_gap(points, radii) >= -TABLE_CONTACT_SLACK
        if self._sphere_gap(points, radii) < block_margin:
            return False
        if (
            check_carried
            and maneuver
            and maneuver.carried_points
            and not maneuver.carried_other_only
            and self.block_spheres
        ):
            carried = col.carried_model(self.robot, self.specs, maneuver.carried_points)
            carried_points = np.asarray(carried.worldPoints(q)).T
            if (
                self._sphere_gap(
                    carried_points,
                    np.full(len(carried_points), maneuver.carried_radius),
                )
                < block_margin
            ):
                return False
        return True

    def clearances(
        self, q: np.ndarray, maneuver: Maneuver | None = None
    ) -> dict[str, float]:
        """Least sphere clearance to the floor, wood and between arms at a configuration, m."""
        q = np.asarray(q, dtype=float)
        floor_points = np.asarray(self.models.floor_model.worldPoints(q)).T
        floor = float(np.min(floor_points[:, 2] - self.models.floor_radii))
        points = np.asarray(self.models.model.worldPoints(q)).T
        blocks = self._sphere_gap(points, self.models.radii)
        split = self.models.self_split
        d = np.linalg.norm(points[:split, None, :] - points[None, split:, :], axis=2)
        radii = self.models.radii[:split, None] + self.models.radii[None, split:]
        return {
            "floor": floor,
            "wood": blocks,
            "table": self.table_gap(points, self.models.radii),
            "cross_arm": float(np.min(d - radii)),
        }

    # -- inverse kinematics ---------------------------------------------
    def _tip(self, spec: cell.ArmSpec, q: np.ndarray) -> gtsam.Pose3:
        return cell.tool_pose(self.robot, spec, q)

    def _residual(self, spec, goal, q) -> np.ndarray:
        tip = self._tip(spec, q)
        position = np.asarray(tip.translation()) - np.asarray(goal.translation())
        rotation = gtsam.Rot3.Logmap(tip.rotation().between(goal.rotation()))
        return np.concatenate([position, rotation])

    def _solve_ik(
        self,
        spec: cell.ArmSpec,
        goal: gtsam.Pose3,
        seed: dict,
        gantry_sigma: float,
        warm: bool,
    ):
        sigmas = {self.robot.joint(n).key(): gantry_sigma for n in spec.q_names[:3]}
        if warm:
            sigmas[self.robot.joint(spec.q_names[3]).key()] = 0.02
        limits = {self.robot.joint(n).key(): b for n, b in spec.limits.items()}
        priors = gtsam.Values()
        for joint in self.robot.joints():
            gtd.InsertJointAngle(priors, joint.id(), 0, seed.get(joint.name(), 0.0))
        params = gtd.KinematicsParameters()
        for key, sigma in sigmas.items():
            params.setJointPriorSigma(key, float(sigma))
        for key, (lower, upper) in limits.items():
            params.setJointLimit(key, float(lower), float(upper))
        result = gtd.Kinematics(params).inverse(
            gtd.Slice(0),
            self.robot,
            {0: gtd.PoseGoal(spec.link, spec.tool_t, goal)},
            priors,
            False,
        )
        return np.array(
            [gtd.JointAngle(result, self.joint_id[n], 0) for n in spec.q_names]
        )

    def _polish(self, spec, goal, q0, bound=0.15) -> np.ndarray:
        # GTDynamics keeps the pose variable only softly coupled to joint-angle FK, so a bounded local
        # least-squares step on real FK removes the residual (SciPy: no GTSAM equivalent for bounds).
        fit = optimize.least_squares(
            lambda q: self._residual(spec, goal, q),
            q0,
            bounds=(q0 - bound, q0 + bound),
            xtol=1e-14,
            ftol=1e-14,
            gtol=1e-14,
        )
        return np.clip(
            fit.x,
            [spec.limits[n][0] for n in spec.q_names],
            [spec.limits[n][1] for n in spec.q_names],
        )

    def ik_seeds(self, spec: cell.ArmSpec, goal: gtsam.Pose3):
        """Yield (seed joint dict, gantry sigma) candidates, most conventional first."""
        vertical = abs(goal.rotation().matrix()[2, 2]) > 0.5
        gx, gy = goal.translation()[0], goal.translation()[1]
        side = -0.6 if spec.arm == 1 else 0.6
        elbows = [
            {2: np.deg2rad(-60.0), 3: 0.0 if vertical else np.deg2rad(60.0)},
            {2: np.deg2rad(-37.0), 3: np.deg2rad(-21.0)},
            {2: np.deg2rad(-90.0), 3: np.deg2rad(30.0)},
            None,
        ]
        for z in (1.15, 0.7, 1.6):
            for dx in (side, -side, 0.0):
                for elbow in elbows:
                    seed = {
                        n: v for n, v in zip(spec.q_names, [gx + dx, gy, z] + [0.0] * 6)
                    }
                    for number, angle in (elbow or {}).items():
                        seed[f"robot{spec.arm}_joint_{number}"] = angle
                    yield seed, (
                        self.settings.vertical_gantry_sigma
                        if vertical
                        else self.settings.gantry_seed_sigma
                    )

    def inside_limit_margin(self, q: np.ndarray, index: int) -> bool:
        """Whether one arm's configuration avoids every active joint-limit factor."""
        block = slice(9 * index, 9 * index + 9)
        return bool(
            np.all(q >= self.factor_lower[block] + self.limit_threshold[block])
            and np.all(q <= self.factor_upper[block] - self.limit_threshold[block])
        )

    def solve_ik(
        self,
        spec: cell.ArmSpec,
        goal: gtsam.Pose3,
        current: np.ndarray,
        other: np.ndarray,
        maneuver: Maneuver | None = None,
        warm: bool = False,
        tolerance: float = 2e-3,
    ) -> np.ndarray:
        """Return the arm configuration that places the tool at ``goal`` without collisions.

        Candidate seeds are tried until the pose error is small and the pair of arms is collision free.
        """
        index = spec.arm - 1
        best, best_error = None, np.inf
        fallback, fallback_error = None, np.inf
        candidates = (
            [({n: v for n, v in zip(spec.q_names, current)}, 0.005, True)]
            if warm
            else []
        )
        candidates += [
            (seed, sigma, False) for seed, sigma in self.ik_seeds(spec, goal)
        ]
        for seed, sigma, is_warm in candidates:
            try:
                q = self._polish(
                    spec, goal, self._solve_ik(spec, goal, seed, sigma, is_warm)
                )
            except RuntimeError:
                continue
            residual = self._residual(spec, goal, q)
            error = float(
                np.linalg.norm(residual[:3]) + 0.1 * np.linalg.norm(residual[3:])
            )
            full = self.join([q, other] if index == 0 else [other, q])
            free = self.collision_free(
                full, maneuver, pad=False, check_carried=False, only_arm=spec.arm
            )
            if error < tolerance and free and self.inside_limit_margin(q, index):
                return q
            if free and error < best_error:
                best, best_error = q, error
            if error < fallback_error:
                fallback, fallback_error = q, error
        if best is not None:
            return best
        if fallback is None:
            raise RuntimeError(
                f"no IK solution for arm {spec.arm} at {np.round(goal.translation(), 3)}"
            )
        print(
            f"  warning: arm {spec.arm} IK at {np.round(goal.translation(), 3)} touches an obstacle; "
            f"using the lowest-error solution (error {fallback_error:.4f})",
            flush=True,
        )
        return fallback

    # -- graph -----------------------------------------------------------
    def total_time_for(self, q_start: np.ndarray, q_goal: np.ndarray) -> float:
        """Return the maneuver duration from the slowest joint at a fraction of its speed limit."""
        s = self.settings
        return float(
            max(
                s.min_total_time,
                np.max(
                    np.abs(q_goal - q_start)
                    / (s.velocity_time_fraction * self.velocity_limit)
                ),
            )
        )

    def _self_models(self, maneuver: Maneuver):
        m = self.models
        if maneuver.carried_by is None or maneuver.contact:
            return m.model, m.radii, m.self_pairs, m.self_split
        other = 2 if maneuver.carried_by == 1 else 1
        points = gtd.PointOnLinks()
        radii = []
        for link, center in m.arm_points[maneuver.carried_by]:
            points.append(gtd.PointOnLink(link, center))
        radii += list(m.arm_radii[maneuver.carried_by])
        for link, center in maneuver.carried_points:
            points.append(gtd.PointOnLink(link, center))
        radii += [maneuver.carried_radius] * len(maneuver.carried_points)
        split = len(radii)
        for link, center in m.arm_points[other]:
            points.append(gtd.PointOnLink(link, center))
        radii += list(m.arm_radii[other])
        joints = [self.robot.joint(n) for n in self.joint_names]
        pairs = gtd.SelfCollisionPairs()
        for a in range(split):
            for b in range(split, len(radii)):
                pairs.append(
                    gtd.SelfCollisionPair(a, b, self.settings.cross_arm_clearance)
                )
        return (
            gtd.RobotQueryPoints(self.robot, cell.BASE_LINK, joints, points),
            np.array(radii),
            pairs,
            split,
        )

    def maneuver_time(self, q_start, q_goal, maneuver: Maneuver) -> float:
        """Return the duration, summed over every leg when the maneuver passes through lifted poses."""
        points = [q_start, *(maneuver.rrt_via or ()), q_goal]
        return sum(self.total_time_for(a, b) for a, b in zip(points, points[1:]))

    def build_graph(
        self, q_start, q_goal, total_time: float, maneuver: Maneuver
    ) -> gtsam.NonlinearFactorGraph:
        """Assemble the GPMP2 graph for one maneuver."""
        s, m = self.settings, self.models
        n = maneuver.state_count or s.state_count
        checks = maneuver.check_interpolated or s.check_interpolated
        dt = total_time / (n - 1)
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.PriorFactorVector(X(0), q_start, self.endpoint_model))
        graph.add(
            gtsam.PriorFactorVector(V(0), np.zeros(self.dof), self.endpoint_model)
        )
        graph.add(gtsam.PriorFactorVector(X(n - 1), q_goal, self.endpoint_model))
        graph.add(
            gtsam.PriorFactorVector(V(n - 1), np.zeros(self.dof), self.endpoint_model)
        )
        self._add_pinned(graph, q_start, q_goal, maneuver)
        self_model, self_radii, self_pairs, _ = self._self_models(maneuver)
        carried = None
        if (
            maneuver.carried_points
            and self.block_field is not None
            and not maneuver.carried_other_only
        ):
            carried = col.carried_model(self.robot, self.specs, maneuver.carried_points)
            carried_radii = np.full(
                len(maneuver.carried_points), maneuver.carried_radius
            )
        block_pad = (
            maneuver.block_clearance
            if maneuver.block_clearance is not None
            else s.block_clearance
        )
        sigma = s.cost_sigma
        for k in range(n - 1):
            graph.add(
                gtd.GPLinearPrior(X(k), V(k), X(k + 1), V(k + 1), dt, self.qc_model)
            )
        for k in range(n):
            graph.add(
                gtd.JointLimitFactorVector(
                    X(k),
                    self.limit_model,
                    self.factor_lower,
                    self.factor_upper,
                    self.limit_threshold,
                )
            )
            graph.add(
                gtd.VelocityLimitFactorVector(
                    V(k),
                    self.velocity_model,
                    self.velocity_limit,
                    s.velocity_limit_margin * self.velocity_limit,
                )
            )
            graph.add(
                gtd.ObstacleSDFFactor(
                    X(k),
                    m.floor_model,
                    self.floor,
                    sigma,
                    s.floor_clearance,
                    m.floor_radii,
                )
            )
            graph.add(
                gtd.SelfCollisionSphereFactor(
                    X(k), self_model, self_pairs, self_radii, sigma
                )
            )
            graph.add(
                gtd.SelfCollisionSphereFactor(
                    X(k), m.bridge_model, m.bridge_pairs, m.bridge_radii, sigma
                )
            )
            if self.table_field is not None and maneuver.contact:
                graph.add(
                    gtd.ObstacleSDFFactor(
                        X(k), m.model, self.table_field, sigma, 0.0, m.radii
                    )
                )
            if self.block_field is not None and not maneuver.contact:
                graph.add(
                    gtd.ObstacleSDFFactor(
                        X(k), m.model, self.block_field, sigma, block_pad, m.radii
                    )
                )
                if carried is not None:
                    graph.add(
                        gtd.ObstacleSDFFactor(
                            X(k),
                            carried,
                            self.block_field,
                            sigma,
                            block_pad,
                            carried_radii,
                        )
                    )
            for cable in self.cables:
                graph.add(
                    gtd.NNCableFactor(
                        X(k), cable, self.floor, sigma, s.floor_clearance, CABLE_RADIUS
                    )
                )
                if self.block_field is not None:
                    graph.add(
                        gtd.NNCableFactor(
                            X(k),
                            cable,
                            self.block_field,
                            sigma,
                            block_pad,
                            CABLE_RADIUS,
                        )
                    )
        for k in range(n - 1):
            for i in range(1, checks + 1):
                tau = i * dt / (checks + 1)
                keys = (X(k), V(k), X(k + 1), V(k + 1))
                graph.add(
                    gtd.ObstacleSDFFactorGP(
                        *keys,
                        m.floor_model,
                        self.floor,
                        sigma,
                        s.floor_clearance,
                        m.floor_radii,
                        self.qc_model,
                        dt,
                        tau,
                    )
                )
                graph.add(
                    gtd.SelfCollisionSphereFactorGP(
                        *keys,
                        self_model,
                        self_pairs,
                        self_radii,
                        sigma,
                        self.qc_model,
                        dt,
                        tau,
                    )
                )
                graph.add(
                    gtd.SelfCollisionSphereFactorGP(
                        *keys,
                        m.bridge_model,
                        m.bridge_pairs,
                        m.bridge_radii,
                        sigma,
                        self.qc_model,
                        dt,
                        tau,
                    )
                )
                if self.table_field is not None and maneuver.contact:
                    graph.add(
                        gtd.ObstacleSDFFactorGP(
                            *keys,
                            m.model,
                            self.table_field,
                            sigma,
                            0.0,
                            m.radii,
                            self.qc_model,
                            dt,
                            tau,
                        )
                    )
                if self.block_field is not None and not maneuver.contact:
                    graph.add(
                        gtd.ObstacleSDFFactorGP(
                            *keys,
                            m.model,
                            self.block_field,
                            sigma,
                            block_pad,
                            m.radii,
                            self.qc_model,
                            dt,
                            tau,
                        )
                    )
                    if carried is not None:
                        graph.add(
                            gtd.ObstacleSDFFactorGP(
                                *keys,
                                carried,
                                self.block_field,
                                sigma,
                                block_pad,
                                carried_radii,
                                self.qc_model,
                                dt,
                                tau,
                            )
                        )
                for cable in self.cables:
                    graph.add(
                        gtd.NNCableFactorGP(
                            *keys,
                            cable,
                            self.floor,
                            sigma,
                            s.floor_clearance,
                            CABLE_RADIUS,
                            self.qc_model,
                            dt,
                            tau,
                        )
                    )
                    if self.block_field is not None:
                        graph.add(
                            gtd.NNCableFactorGP(
                                *keys,
                                cable,
                                self.block_field,
                                sigma,
                                block_pad,
                                CABLE_RADIUS,
                                self.qc_model,
                                dt,
                                tau,
                            )
                        )
        return graph

    def _add_pinned(self, graph, q_start, q_goal, maneuver: Maneuver) -> None:
        """Pin held and straight-flagged degrees of freedom to their linear path at interior states."""
        n = maneuver.state_count or self.settings.state_count
        held = np.abs(q_goal - q_start) < 1e-6
        mask = held.copy()
        for flag, offset in zip(maneuver.straight, (0, 9)):
            if flag:
                mask[offset : offset + 9] = True
        if not mask.any():
            return
        model = gtsam.noiseModel.Diagonal.Sigmas(np.where(mask, 1e-6, 1e6))
        still = gtsam.noiseModel.Diagonal.Sigmas(np.where(held, 1e-3, 1e6))
        for k in range(1, n - 1):
            target = q_start + (k / (n - 1)) * (q_goal - q_start)
            graph.add(gtsam.PriorFactorVector(X(k), target, model))
            if held.any():
                graph.add(gtsam.PriorFactorVector(V(k), np.zeros(self.dof), still))

    # -- solve -----------------------------------------------------------
    def floor_free(self, q: np.ndarray) -> bool:
        """Whether the upper-arm spheres are above the floor, ignoring every clearance."""
        points = np.asarray(
            self.models.floor_model.worldPoints(np.asarray(q, dtype=float))
        ).T
        return not np.any(points[:, 2] - self.models.floor_radii < 0.0)

    def straight_mask(self, q_start, q_goal, maneuver: Maneuver) -> np.ndarray:
        """Return the degrees of freedom held or pinned to their straight path."""
        mask = np.abs(q_goal - q_start) < 1e-6
        for flag, offset in zip(maneuver.straight, (0, 9)):
            if flag:
                mask[offset : offset + 9] = True
        return mask

    def seed(
        self, q_start, q_goal, total_time: float, maneuver: Maneuver, rrt_seed: int = 0
    ):
        """Return (states, velocities, kind) from RRT-Connect, or a straight line when RRT is off.

        Tools and planks touch the wood at contact poses, so only the floor is checked within
        ``ENDPOINT_RADIUS`` of either end, per arm. Pinned degrees of freedom follow their straight path.
        """
        s, n = self.settings, maneuver.state_count or self.settings.state_count
        q_start, q_goal = np.asarray(q_start, dtype=float), np.asarray(
            q_goal, dtype=float
        )
        span = np.where((self.upper - self.lower) > 0, self.upper - self.lower, 1.0)
        mask = self.straight_mask(q_start, q_goal, maneuver)
        straight = [q_start + (k / (n - 1)) * (q_goal - q_start) for k in range(n)]
        velocity = (q_goal - q_start) / total_time
        straight_velocities = (
            [np.zeros(self.dof)] + [velocity] * (n - 2) + [np.zeros(self.dof)]
        )
        if not maneuver.use_rrt or mask.all():
            return straight, straight_velocities, "straight"

        # RRT runs between the lifted poses when a contact end only has a narrow straight exit.
        first, last = maneuver.rrt_via or (q_start, q_goal)

        def arm_near_end(q, offset):
            block = slice(offset, offset + 9)
            return (
                min(
                    np.linalg.norm((q[block] - first[block]) / span[block]),
                    np.linalg.norm((q[block] - last[block]) / span[block]),
                )
                < ENDPOINT_RADIUS
            )

        def free(q):
            if not self.floor_free(q):
                return False
            return all(
                arm_near_end(q, 9 * (arm - 1))
                or self.collision_free(
                    q, maneuver, check_carried=arm == 1, only_arm=arm
                )
                for arm in (1, 2)
            )

        margin = RRT_BOX_MARGINS[min(rrt_seed, len(RRT_BOX_MARGINS) - 1)]
        lower = np.maximum(self.lower, np.minimum(first, last) - margin * span)
        upper = np.minimum(self.upper, np.maximum(first, last) + margin * span)
        lower = np.where(mask, np.minimum(q_start, q_goal), lower)
        upper = np.where(mask, np.maximum(q_start, q_goal), upper)
        path = rrt.rrt_connect(
            first,
            last,
            lower,
            upper,
            free,
            s.rrt_step,
            s.rrt_resolution,
            s.rrt_iterations,
            s.rrt_seed + rrt_seed,
            endpoint_collision_free=self.floor_free,
        )
        path = rrt.shortcut(path, free, span, s.rrt_resolution, s.rrt_seed + rrt_seed)
        if maneuver.rrt_via is None:
            line, velocities = rrt.path_to_states(path, n, total_time, span)
        else:
            # The lifted poses are support states, so resampling cannot cut the corners at contact.
            line = [q_start] + rrt.resample_path(path, n - 2, span) + [q_goal]
            velocities = rrt.central_velocities(line, total_time)
        for k in range(n):
            line[k] = np.where(mask, straight[k], line[k])
            velocities[k] = np.where(mask, straight_velocities[k], velocities[k])
        return line, velocities, "rrt"

    @staticmethod
    def error_breakdown(
        graph: gtsam.NonlinearFactorGraph, values: gtsam.Values
    ) -> dict[str, float]:
        """Return the graph error summed by factor class, largest first (for diagnostics)."""
        totals: dict[str, float] = {}
        for index in range(graph.size()):
            factor = graph.at(index)
            name = type(factor).__name__
            totals[name] = totals.get(name, 0.0) + factor.error(values)
        return {
            name: round(total, 1)
            for name, total in sorted(totals.items(), key=lambda item: -item[1])[:4]
        }

    def plan(self, q_start, q_goal, maneuver: Maneuver | None = None) -> Plan:
        """Solve one maneuver: seed, batch Levenberg-Marquardt, and a convergence check."""
        maneuver = maneuver or Maneuver()
        q_start, q_goal = np.asarray(q_start, dtype=float), np.asarray(
            q_goal, dtype=float
        )
        total_time = self.maneuver_time(q_start, q_goal, maneuver)
        n = maneuver.state_count or self.settings.state_count
        graph = self.build_graph(q_start, q_goal, total_time, maneuver)
        failure = "not attempted"
        for attempt in range(len(RRT_BOX_MARGINS)):
            try:
                line, velocities, kind = self.seed(
                    q_start, q_goal, total_time, maneuver, attempt
                )
            except ValueError as error:
                failure = str(error)
                continue
            initial = gtsam.Values()
            for k in range(n):
                initial.insert(X(k), line[k])
                initial.insert(V(k), velocities[k])
            params = gtsam.LevenbergMarquardtParams()
            params.setMaxIterations(self.settings.lm_iterations)
            # Run to convergence: the cost is nearly flat along large motions, and ISAM2 would finish the solve.
            params.setRelativeErrorTol(LM_ERROR_TOLERANCE)
            params.setAbsoluteErrorTol(LM_ERROR_TOLERANCE)
            values = gtsam.LevenbergMarquardtOptimizer(
                graph, initial, params
            ).optimize()
            error = graph.error(values)
            if np.isfinite(error) and error <= CONVERGENCE_ERROR:
                return Plan(
                    graph,
                    values,
                    float(error),
                    total_time,
                    total_time / (n - 1),
                    n,
                    kind,
                )
            failure = f"graph error {error:.3g}; by factor: {self.error_breakdown(graph, values)}"
            if kind == "straight":
                break
        raise RuntimeError(f"maneuver did not converge: {failure}")
