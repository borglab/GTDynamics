"""
* GTDynamics Copyright 2026, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* See LICENSE for the license information
*
* @file  ik_backend.py
* @brief Robot model, inverse kinematics schemes and failure diagnostics for
*        the interactive URDF IK app.
"""

from __future__ import annotations

import math
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import gtsam
import numpy as np

import gtdynamics as gtd

DEFAULT_URDF_ROOT = Path(gtd.URDF_PATH).resolve()

# Frame names commonly used for a robot's tool frame, most preferred first.
PREFERRED_EE_NAMES = (
    "tool0",
    "ee_link",
    "flange",
    "panda_hand",
    "hand",
    "tool",
    "end_effector",
    "right_palm",
)

MOVABLE_TYPES = ("revolute", "continuous", "prismatic")
PANDA_JOINTS = tuple(f"joint{i}" for i in range(1, 8))


class ModelError(RuntimeError):
    """A URDF that cannot be used for interactive kinematics."""


# ------------------------------------------------------------------- URDF XML


def read_urdf_xml(path: Path) -> ET.Element:
    """Parse a URDF, tolerating undeclared namespace prefixes (xacro:, etc.)."""
    text = Path(path).read_text()
    text = re.sub(r"(</?)([A-Za-z_][\w.-]*):", r"\1\2_", text)
    text = re.sub(r"(\s)([A-Za-z_][\w.-]*):([A-Za-z_][\w.-]*\s*=)", r"\1\2_\3", text)
    return ET.fromstring(text)


def _floats(text: Optional[str], default) -> list:
    return list(default) if text is None else [float(v) for v in text.split()]


@dataclass
class UrdfJoint:
    """The parts of a URDF <joint> that GTDynamics' Python API does not expose."""

    name: str
    type: str
    parent: str
    child: str
    origin: gtsam.Pose3  # parent link frame -> joint (= child link) frame
    axis: np.ndarray  # unit axis in the joint frame
    lower: float
    upper: float


def parse_urdf(path: Path) -> tuple[list[str], dict[str, UrdfJoint]]:
    """Return the link names and joints of a URDF file."""
    root = read_urdf_xml(path)
    links = [el.get("name") for el in root.findall("link")]
    joints = {}
    for el in root.findall("joint"):
        jtype = el.get("type", "fixed")
        origin_el = el.find("origin")
        xyz = _floats(None if origin_el is None else origin_el.get("xyz"), (0, 0, 0))
        rpy = _floats(None if origin_el is None else origin_el.get("rpy"), (0, 0, 0))
        axis_el = el.find("axis")
        axis = np.array(
            _floats(None if axis_el is None else axis_el.get("xyz"), (1, 0, 0))
        )
        norm = np.linalg.norm(axis)
        axis = axis / norm if norm > 0 else np.array([1.0, 0.0, 0.0])

        # Continuous joints, and limits with lower >= upper, are unlimited.
        lower, upper = -math.inf, math.inf
        limit_el = el.find("limit")
        if jtype in ("revolute", "prismatic") and limit_el is not None:
            lo = float(limit_el.get("lower", 0.0))
            hi = float(limit_el.get("upper", 0.0))
            if lo < hi:
                lower, upper = lo, hi

        joints[el.get("name")] = UrdfJoint(
            name=el.get("name"),
            type=jtype,
            parent=el.find("parent").get("link"),
            child=el.find("child").get("link"),
            origin=gtsam.Pose3(gtsam.Rot3.RzRyRx(*rpy), gtsam.Point3(*xyz)),
            axis=axis,
            lower=lower,
            upper=upper,
        )
    return links, joints


def check_loadable(path: Path, timeout: float = 120.0) -> Optional[str]:
    """Parse a URDF with GTDynamics in a subprocess.

    Some malformed models crash the native parser, which would take down the
    server, so models are vetted out of process first.

    Returns:
        None if the model loads, otherwise a human readable error.
    """
    code = (
        "import sys, gtdynamics as gtd\n"
        "r = gtd.CreateRobotFromFile(sys.argv[1])\n"
        "print('NUM_LINKS', r.numLinks())\n"
    )
    try:
        proc = subprocess.run(
            [sys.executable, "-c", code, str(path)],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return f"GTDynamics did not finish parsing within {timeout:.0f} s."
    if proc.returncode < 0:
        name = signal.Signals(-proc.returncode).name
        return f"GTDynamics' URDF parser crashed ({name}) on this file."
    if proc.returncode != 0:
        lines = [
            line.strip()
            for line in (proc.stdout + proc.stderr).splitlines()
            if line.startswith(("ERROR", "RuntimeError", "Error"))
        ]
        detail = " ".join(dict.fromkeys(lines[-4:])) or "unknown error"
        return f"GTDynamics could not load this URDF: {detail}"
    match = re.search(r"NUM_LINKS (\d+)", proc.stdout)
    if match is None or int(match.group(1)) == 0:
        return "GTDynamics loaded the URDF but found no links with inertia."
    return None


# ---------------------------------------------------------------- robot model


@dataclass
class JointInfo:
    name: str
    id: int
    type: str  # revolute, prismatic, helical or fixed
    parent: str
    child: str
    axis: np.ndarray  # unit axis in the child link frame
    lower: float
    upper: float

    @property
    def movable(self) -> bool:
        return self.type in ("revolute", "prismatic")


@dataclass
class LinkInfo:
    name: str
    id: int
    comTlink: gtsam.Pose3  # URDF link frame expressed in the CoM frame


@dataclass
class EndEffector:
    name: str  # frame name shown to the user
    link: str  # GTDynamics link the frame is rigidly attached to
    linkTee: gtsam.Pose3


class RobotModel:
    """A URDF loaded into GTDynamics with its root link fixed in the world."""

    def __init__(self, urdf_root: Path, rel_path: str):
        self.rel_path = rel_path
        self.notes: list[str] = []
        path = Path(urdf_root) / rel_path
        urdf_links, urdf_joints = parse_urdf(path)

        try:
            robot = gtd.CreateRobotFromFile(str(path))
        except RuntimeError as e:
            raise ModelError(f"GTDynamics could not load this URDF: {e}")
        if robot.numLinks() == 0:
            raise ModelError("GTDynamics found no links with inertia.")

        parent_counts: dict[str, int] = {}
        for joint in robot.joints():
            child = joint.child().name()
            parent_counts[child] = parent_counts.get(child, 0) + 1
        loops = sorted(name for name, n in parent_counts.items() if n > 1)
        if loops:
            raise ModelError(
                "The model has a closed kinematic loop (link "
                f"'{loops[0]}' has several parent joints); this app only "
                "handles kinematic trees."
            )
        roots = [l.name() for l in robot.links() if l.name() not in parent_counts]
        if not roots:
            raise ModelError(
                "Every link has a parent joint, so there is no " "base link to fix."
            )

        children: dict[str, list[str]] = {}
        for joint in robot.joints():
            children.setdefault(joint.parent().name(), []).append(joint.child().name())

        def subtree_size(name: str) -> int:
            return 1 + sum(subtree_size(c) for c in children.get(name, []))

        self.base = max(sorted(roots), key=subtree_size)
        if len(roots) > 1:
            self.notes.append(
                f"The model has {len(roots)} disconnected parts; '{self.base}' "
                "is fixed and the others float freely."
            )
        self.robot = robot.fixLink(self.base)

        self.links = {
            l.name(): LinkInfo(l.name(), l.id(), l.bMcom().between(l.bMlink()))
            for l in self.robot.links()
        }
        self.joints: dict[str, JointInfo] = {}
        for joint in self.robot.joints():
            urdf_joint = urdf_joints.get(joint.name())
            jtype = str(joint.type()).split(".")[-1].lower()
            self.joints[joint.name()] = JointInfo(
                name=joint.name(),
                id=joint.id(),
                type=jtype,
                parent=joint.parent().name(),
                child=joint.child().name(),
                axis=(
                    urdf_joint.axis
                    if urdf_joint is not None
                    else np.array([1.0, 0.0, 0.0])
                ),
                lower=urdf_joint.lower if urdf_joint is not None else -math.inf,
                upper=urdf_joint.upper if urdf_joint is not None else math.inf,
            )
        self.parent_joint = {j.child: j for j in self.joints.values()}

        dropped = [
            j.name
            for j in urdf_joints.values()
            if j.type in MOVABLE_TYPES and j.name not in self.joints
        ]
        if dropped:
            self.notes.append(
                "GTDynamics does not model these URDF joints (their links have "
                "no valid inertia), so IK leaves them at zero: " + ", ".join(dropped)
            )

        self.end_effectors = self._find_end_effectors(urdf_links, urdf_joints)
        self.default_ee = self._pick_default_ee()
        self.panda_ikfast = self._detect_panda_ikfast()

    def _detect_panda_ikfast(self) -> bool:
        """Recognize IKFast's seven-joint geometry, independently of file name."""
        if not hasattr(gtd, "PandaIKFast") or self.base != "link0":
            return False
        if "link7" not in self.end_effectors:
            return False
        chain = self.movable_chain("link7")
        if tuple(j.name for j in chain) != PANDA_JOINTS or any(
            j.type != "revolute"
            or not (math.isfinite(j.lower) and math.isfinite(j.upper))
            for j in chain
        ):
            return False
        # Compare several configurations, including an independent perturbation
        # of each joint, so a renamed or geometrically modified URDF is safe.
        seed = np.array([0.2, -0.4, 0.3, -1.6, 0.5, 1.2, -0.2])
        for x in [seed, *(seed + 0.2 * row for row in np.eye(7))]:
            q = self.default_configuration()
            q.update(zip(PANDA_JOINTS, x))
            frames = self.link_frames(q)
            actual = frames[self.base].between(self.ee_pose(frames, "link7"))
            if not actual.equals(gtd.PandaIKFast.forward(x), 1e-6):
                return False
        return True

    def scheme_ids(self, ee: str, orientation: bool = True) -> list[str]:
        """Return solvers supported by this model, frame and goal type."""
        return [
            name
            for name in SCHEMES
            if name != "panda_ikfast"
            or (
                self.panda_ikfast
                and orientation
                and self.end_effectors[ee].link == "link7"
            )
        ]

    def _find_end_effectors(self, urdf_links, urdf_joints):
        """GTD links plus URDF frames rigidly attached to them (e.g. tool0)."""
        effs = {
            name: EndEffector(name, name, gtsam.Pose3()) for name in sorted(self.links)
        }
        by_child = {j.child: j for j in urdf_joints.values()}
        for name in urdf_links:
            if name in effs:
                continue
            linkTee, current = gtsam.Pose3(), name
            while current not in self.links:
                joint = by_child.get(current)
                if joint is None or joint.type != "fixed":
                    break
                linkTee = joint.origin.compose(linkTee)
                current = joint.parent
            else:
                effs[name] = EndEffector(name, current, linkTee)
        return effs

    def _pick_default_ee(self) -> str:
        for name in PREFERRED_EE_NAMES:
            if name in self.end_effectors and self.movable_chain(name):
                return name

        def score(name):
            chain = self.chain(name)
            return (
                len([j for j in chain if j.movable]),
                len(chain),
                name != self.end_effectors[name].link,
            )

        return max(sorted(self.end_effectors), key=score)

    def chain(self, ee: str) -> list[JointInfo]:
        """GTD joints from the base link to the end-effector, base first."""
        if ee not in self.end_effectors:
            raise ModelError(f"Unknown end-effector frame '{ee}'.")
        chain, current = [], self.end_effectors[ee].link
        while current != self.base:
            joint = self.parent_joint.get(current)
            if joint is None:
                raise ModelError(
                    f"Link '{current}' is not connected to the fixed base "
                    f"'{self.base}', so it cannot be moved by the base's "
                    "joints."
                )
            chain.append(joint)
            current = joint.parent
        return chain[::-1]

    def movable_chain(self, ee: str) -> list[JointInfo]:
        return [j for j in self.chain(ee) if j.movable]

    def default_configuration(self) -> dict[str, float]:
        """Zero joint angles, moved inside the joint limits if necessary."""
        q = {}
        for j in self.joints.values():
            if j.lower <= 0.0 <= j.upper:
                q[j.name] = 0.0
            else:
                q[j.name] = 0.5 * (j.lower + j.upper)
        return q

    def values(self, q: dict[str, float]) -> gtsam.Values:
        values = gtsam.Values()
        for j in self.joints.values():
            gtd.InsertJointAngle(values, j.id, 0, float(q.get(j.name, 0.0)))
        return values

    def link_frames(self, q: dict[str, float]) -> dict[str, gtsam.Pose3]:
        """World poses of all URDF link frames (not the CoM frames)."""
        fk = self.robot.forwardKinematics(self.values(q), 0, self.base)
        return {
            name: gtd.Pose(fk, info.id, 0).compose(info.comTlink)
            for name, info in self.links.items()
        }

    def ee_pose(self, frames: dict[str, gtsam.Pose3], ee: str) -> gtsam.Pose3:
        eff = self.end_effectors[ee]
        return frames[eff.link].compose(eff.linkTee)

    def skeleton(self, frames: dict[str, gtsam.Pose3]) -> list[dict]:
        """World link frames and parents for rendering the robot and skeleton."""
        return [
            {
                "link": name,
                "parent": (
                    self.parent_joint[name].parent
                    if name in self.parent_joint
                    else None
                ),
                "position": list(map(float, frames[name].translation())),
                "quat_xyzw": quaternion_xyzw(frames[name].rotation()),
            }
            for name in sorted(frames)
        ]

    def describe(self) -> dict:
        def limit(v):
            return None if math.isinf(v) else v

        return {
            "model": self.rel_path,
            "base": self.base,
            "joints": [
                {
                    "name": j.name,
                    "type": j.type,
                    "lower": limit(j.lower),
                    "upper": limit(j.upper),
                }
                for j in sorted(self.joints.values(), key=lambda j: j.id)
            ],
            "end_effectors": [
                {
                    "name": e.name,
                    "link": e.link,
                    "dof": len(self.movable_chain(e.name)),
                    "schemes": self.scheme_ids(e.name),
                }
                for e in self.end_effectors.values()
            ],
            "default_ee": self.default_ee,
            "notes": self.notes,
        }


def quaternion_xyzw(rotation: gtsam.Rot3) -> list[float]:
    """Return the rotation as the browser's x, y, z, w quaternion."""
    quat = rotation.toQuaternion()
    return [quat.x(), quat.y(), quat.z(), quat.w()]


def pose_to_json(pose: gtsam.Pose3) -> dict:
    rotation = pose.rotation()
    return {
        "position": list(map(float, pose.translation())),
        "quat_xyzw": quaternion_xyzw(rotation),
        "rpy_deg": list(map(float, np.degrees(rotation.rpy()))),
    }


def pose_from_json(data: dict) -> gtsam.Pose3:
    x, y, z, w = data["quat_xyzw"]
    return gtsam.Pose3(
        gtsam.Rot3.Quaternion(w, x, y, z), gtsam.Point3(*data["position"])
    )


# ------------------------------------------------------------------ IK problem


@dataclass
class IkRequest:
    ee: str
    target: gtsam.Pose3  # desired world pose of the end-effector frame
    q0: dict[str, float]  # warm start (current configuration)
    orientation: bool = True  # False: position-only goal
    pos_tol: float = 1e-3  # meters
    rot_tol: float = math.radians(0.5)  # radians
    max_iters: Optional[int] = None  # None: the scheme's default


@dataclass
class SolverOutput:
    q: dict[str, float]
    iterations: Optional[int]  # None when the solver does not report it
    max_iters: int
    stalled: bool = False  # the solver stopped making progress
    still_improving: bool = False  # error decreased over the last iterations
    limit_tol: float = 1e-6  # how close to a limit counts as "at the limit"
    # For solvers that optimize link poses too: the end-effector pose in the
    # solver's own variables, which may disagree with FK of its joint angles.
    solver_ee_pose: Optional[gtsam.Pose3] = None
    failure_reason: Optional[str] = None


class IkProblem:
    """Chain variables, task error and geometric Jacobian for one IK solve."""

    def __init__(self, model: RobotModel, req: IkRequest):
        self.model, self.req = model, req
        self.eff = model.end_effectors[req.ee]
        self.chain = model.movable_chain(req.ee)
        for j in self.chain:
            if j.type == "helical":
                raise ModelError(
                    f"Joint '{j.name}' is helical, which these "
                    "IK schemes do not support."
                )
        self.q_full = {name: float(req.q0.get(name, 0.0)) for name in model.joints}
        self.lower = np.array([j.lower for j in self.chain])
        self.upper = np.array([j.upper for j in self.chain])
        self.x0 = self.clamp(np.array([self.q_full[j.name] for j in self.chain]))
        # Weight (meters per radian) that makes orientation errors comparable
        # to position errors, proportional to the size of the arm.
        self.w_rot = float(np.clip(0.5 * self.segment_length(), 0.05, 1.0))

    @property
    def rows(self) -> int:
        return 6 if self.req.orientation else 3

    def to_q(self, x: np.ndarray) -> dict[str, float]:
        q = dict(self.q_full)
        q.update({j.name: float(v) for j, v in zip(self.chain, x)})
        return q

    def clamp(self, x: np.ndarray) -> np.ndarray:
        return np.clip(x, self.lower, self.upper)

    def fk(self, x: np.ndarray):
        frames = self.model.link_frames(self.to_q(x))
        return frames, self.model.ee_pose(frames, self.req.ee)

    def rot_error(self, T: gtsam.Pose3) -> np.ndarray:
        """World-frame rotation vector taking the current to the goal orientation."""
        return gtsam.Rot3.Logmap(
            self.req.target.rotation().compose(T.rotation().inverse())
        )

    def errors(self, T: gtsam.Pose3) -> tuple[float, float]:
        pos = float(np.linalg.norm(self.req.target.translation() - T.translation()))
        return pos, float(np.linalg.norm(self.rot_error(T)))

    def converged(self, T: gtsam.Pose3) -> bool:
        pos, rot = self.errors(T)
        return pos <= self.req.pos_tol and (
            not self.req.orientation or rot <= self.req.rot_tol
        )

    def error(self, T: gtsam.Pose3) -> np.ndarray:
        """Weighted task error [w_rot * rotation; position], or position only."""
        dp = self.req.target.translation() - T.translation()
        if not self.req.orientation:
            return dp
        return np.concatenate([self.w_rot * self.rot_error(T), dp])

    def joint_axis_origin(self, frames, j: JointInfo):
        T = frames[j.child]
        return T.rotation().matrix() @ j.axis, T.translation()

    def jacobian(self, frames, T: gtsam.Pose3) -> np.ndarray:
        """Geometric Jacobian of the weighted EE pose; rows match error()."""
        J = np.zeros((6, len(self.chain)))
        p_ee = T.translation()
        for i, j in enumerate(self.chain):
            axis, origin = self.joint_axis_origin(frames, j)
            if j.type == "revolute":
                J[:3, i] = self.w_rot * axis
                J[3:, i] = np.cross(axis, p_ee - origin)
            else:
                J[3:, i] = axis
        return J if self.req.orientation else J[3:]

    def segment_length(self) -> float:
        """Length of the rigid links between consecutive joints and the EE."""
        frames, T = self.fk(self.x0)
        total, prev = 0.0, None
        for j, x in zip(self.chain, self.x0):
            axis, origin = self.joint_axis_origin(frames, j)
            anchor = origin - axis * x if j.type == "prismatic" else origin
            if prev is not None:
                total += float(np.linalg.norm(anchor - prev))
            prev = origin
        if prev is not None:
            total += float(np.linalg.norm(T.translation() - prev))
        return total

    def reach(self, frames, x) -> tuple[np.ndarray, float]:
        """Anchor point of the first joint and an upper bound on reach from it."""
        total = self.segment_length()
        for j in self.chain:
            if j.type == "prismatic":
                total += max(abs(j.lower), abs(j.upper))
        axis, origin = self.joint_axis_origin(frames, self.chain[0])
        anchor = origin - axis * x[0] if self.chain[0].type == "prismatic" else origin
        return anchor, total


def _cap_step(dx: np.ndarray, max_step: float = 0.5) -> np.ndarray:
    """Scale a joint step so no joint moves more than max_step per iteration."""
    biggest = float(np.max(np.abs(dx))) if dx.size else 0.0
    return dx * (max_step / biggest) if biggest > max_step else dx


class _BestTracker:
    """Remembers the lowest-error iterate and whether the error still drops."""

    def __init__(self, x, e):
        self.x, self.norm = x.copy(), float(np.linalg.norm(e))
        self.history = [self.norm]

    def update(self, x, e):
        n = float(np.linalg.norm(e))
        self.history.append(n)
        if n < self.norm:
            self.x, self.norm = x.copy(), n

    def still_improving(self, window: int = 5) -> bool:
        h = self.history[-window:]
        return len(h) >= 2 and h[-1] < 0.999 * h[0]


# ------------------------------------------------------------------ IK schemes


def solve_dls(p: IkProblem, max_iters: int) -> SolverOutput:
    """Damped least squares with Levenberg-Marquardt style adaptive damping."""
    x = p.x0.copy()
    frames, T = p.fk(x)
    e = p.error(T)
    best = _BestTracker(x, e)
    lam, iters, stalled = 0.05, 0, False
    while iters < max_iters and not p.converged(T):
        iters += 1
        J = p.jacobian(frames, T)
        H = J.T @ J + lam**2 * np.eye(len(x))
        x_new = p.clamp(x + _cap_step(np.linalg.solve(H, J.T @ e)))
        frames_new, T_new = p.fk(x_new)
        e_new = p.error(T_new)
        if np.linalg.norm(e_new) < np.linalg.norm(e):
            x, frames, T, e = x_new, frames_new, T_new, e_new
            lam = max(0.5 * lam, 1e-4)
            best.update(x, e)
        else:
            lam *= 3.0
            if lam > 1e4:
                stalled = True
                break
    return SolverOutput(
        p.to_q(best.x), iters, max_iters, stalled, best.still_improving()
    )


def solve_pinv(p: IkProblem, max_iters: int) -> SolverOutput:
    """Newton-Raphson with the Moore-Penrose pseudo-inverse (no damping)."""
    x = p.x0.copy()
    frames, T = p.fk(x)
    e = p.error(T)
    best = _BestTracker(x, e)
    iters = 0
    while iters < max_iters and not p.converged(T):
        iters += 1
        J = p.jacobian(frames, T)
        x = p.clamp(x + _cap_step(np.linalg.pinv(J, rcond=1e-6) @ e))
        frames, T = p.fk(x)
        e = p.error(T)
        best.update(x, e)
    return SolverOutput(p.to_q(best.x), iters, max_iters, False, best.still_improving())


def solve_transpose(p: IkProblem, max_iters: int) -> SolverOutput:
    """Jacobian transpose (steepest descent) with the optimal step length."""
    x = p.x0.copy()
    frames, T = p.fk(x)
    e = p.error(T)
    best = _BestTracker(x, e)
    iters, stalled = 0, False
    while iters < max_iters and not p.converged(T):
        iters += 1
        J = p.jacobian(frames, T)
        g = J.T @ e
        Jg = J @ g
        if float(Jg @ Jg) < 1e-20:
            stalled = True
            break
        alpha = float(e @ Jg) / float(Jg @ Jg)
        x_new = p.clamp(x + _cap_step(alpha * g))
        if np.allclose(x_new, x, atol=1e-12):
            stalled = True
            break
        x = x_new
        frames, T = p.fk(x)
        e = p.error(T)
        best.update(x, e)
    return SolverOutput(
        p.to_q(best.x), iters, max_iters, stalled, best.still_improving()
    )


def solve_ccd(p: IkProblem, max_iters: int) -> SolverOutput:
    """Cyclic coordinate descent: optimize one joint at a time, tip to base."""
    x = p.x0.copy()
    frames, T = p.fk(x)
    best = _BestTracker(x, p.error(T))
    target = p.req.target.translation()
    iters, stalled = 0, False
    while iters < max_iters and not p.converged(T):
        iters += 1
        x_start = x.copy()
        for i in reversed(range(len(p.chain))):
            j = p.chain[i]
            axis, origin = p.joint_axis_origin(frames, j)
            p_ee = T.translation()
            if j.type == "prismatic":
                step = float(axis @ (target - p_ee))
            else:
                u, v = p_ee - origin, target - origin
                u, v = u - axis * (axis @ u), v - axis * (axis @ v)
                w_pos = float(np.linalg.norm(u) * np.linalg.norm(v))
                step = (
                    math.atan2(float(axis @ np.cross(u, v)), float(u @ v))
                    if w_pos > 1e-12
                    else 0.0
                )
                if p.req.orientation:
                    # Blend in the rotation about this axis that best reduces
                    # the orientation error.
                    w_rot = p.w_rot**2
                    step = (w_pos * step + w_rot * float(axis @ p.rot_error(T))) / (
                        w_pos + w_rot
                    )
            x[i] = float(np.clip(x[i] + step, p.lower[i], p.upper[i]))
            frames, T = p.fk(x)
        best.update(x, p.error(T))
        if np.allclose(x, x_start, atol=1e-10):
            stalled = True
            break
    return SolverOutput(
        p.to_q(best.x), iters, max_iters, stalled, best.still_improving()
    )


def solve_gtd(p: IkProblem, max_iters: int) -> SolverOutput:
    """GTDynamics factor-graph IK, optimized with Levenberg-Marquardt.

    Both variants jointly optimize link poses and joint angles, warm-started
    from (and softly anchored to) the current configuration. Full-pose goals
    go through Kinematics.inverse with a PoseGoal, which adds GTDynamics'
    joint-limit factors. Kinematics has no warm-started position-only entry
    point, so position goals use the same kinematic factors (qFactors) plus a
    PointGoalFactor; joint limits are not modeled there.
    """
    model = p.model
    link = model.robot.link(p.eff.link)
    comTee = model.links[p.eff.link].comTlink.compose(p.eff.linkTee)
    warm_start = model.values(p.to_q(p.x0))

    if p.req.orientation:
        params = gtd.KinematicsParameters()
        lm = params.lm_parameters
        lm.setMaxIterations(max_iters)
        params.lm_parameters = lm
        goal = gtd.PoseGoal(link, comTee, p.req.target)
        result = gtd.Kinematics(params).inverse(
            gtd.Slice(0), model.robot, {0: goal}, warm_start, False
        )
        iterations = None  # Kinematics.inverse does not report it.
        limit_tol = 0.045  # JointLimitFactor pushes back 0.04 before a limit.
    else:
        graph = gtd.DynamicsGraph(gtd.OptimizerSetting()).qFactors(model.robot, 0, None)
        graph.push_back(
            gtd.PointGoalFactor(
                gtd.PoseKey(link.id(), 0),
                gtsam.noiseModel.Isotropic.Sigma(3, 1e-4),
                comTee.translation(),
                p.req.target.translation(),
            )
        )
        joint_prior = gtsam.noiseModel.Isotropic.Sigma(1, 0.5)
        for j in model.joints.values():
            key = gtd.JointAngleKey(j.id, 0)
            graph.push_back(
                gtsam.PriorFactorDouble(key, warm_start.atDouble(key), joint_prior)
            )
        initial = gtsam.Values(warm_start)
        fk = model.robot.forwardKinematics(warm_start, 0, model.base)
        for info in model.links.values():
            initial.insert(gtd.PoseKey(info.id, 0), gtd.Pose(fk, info.id, 0))
        lm = gtsam.LevenbergMarquardtParams()
        lm.setMaxIterations(max_iters)
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, lm)
        result = optimizer.optimize()
        iterations = optimizer.iterations()
        limit_tol = 1e-6

    q = {j.name: gtd.JointAngle(result, j.id, 0) for j in model.joints.values()}
    solver_ee_pose = gtd.Pose(result, link.id(), 0).compose(comTee)
    return SolverOutput(
        q, iterations, max_iters, limit_tol=limit_tol, solver_ee_pose=solver_ee_pose
    )


def solve_panda_ikfast(p: IkProblem, max_iters: int) -> SolverOutput:
    """Solve a tool pose analytically, sampling joint7 only when necessary.

    IKFast targets link7 in link0 coordinates. Try the current joint7 first,
    then a bounded grid ordered by distance from it. At the first successful
    sample, select the limit-valid branch closest to the current joint angles.
    Max iters bounds the number of free-joint samples, not optimizer iterations.
    """
    max_iters = max(1, max_iters)
    frames, initial_pose = p.fk(p.x0)
    if p.converged(initial_pose):
        return SolverOutput(p.to_q(p.x0), None, max_iters)
    target = (
        frames[p.model.base]
        .inverse()
        .compose(p.req.target.compose(p.eff.linkTee.inverse()))
    )
    samples = [p.x0[-1]]
    samples.extend(
        sorted(
            np.linspace(p.lower[-1], p.upper[-1], max(0, max_iters - 1)),
            key=lambda theta: abs(theta - p.x0[-1]),
        )
    )
    for theta7 in samples:
        candidates = []
        for solution in gtd.PandaIKFast.inverse(target, float(theta7)):
            x = np.asarray(solution, dtype=float)
            if not np.all(np.isfinite(x)):
                continue
            # IKFast angles are periodic; joint6 may validly exceed pi. Choose
            # the equivalent angles closest to the seed inside each interval.
            lo = np.ceil((p.lower - x - 1e-9) / (2 * math.pi))
            hi = np.floor((p.upper - x + 1e-9) / (2 * math.pi))
            if np.any(lo > hi):
                continue
            turns = np.clip(np.rint((p.x0 - x) / (2 * math.pi)), lo, hi)
            x = p.clamp(x + 2 * math.pi * turns)
            _, pose = p.fk(x)
            if p.converged(pose):
                candidates.append(x)
        if candidates:
            best = min(candidates, key=lambda x: np.linalg.norm(x - p.x0))
            return SolverOutput(p.to_q(best), None, max_iters)
    return SolverOutput(
        p.to_q(p.x0),
        None,
        max_iters,
        failure_reason=(
            f"Panda IKFast found no joint-limit-valid solution in {len(samples)} "
            "joint7 samples. Singular solutions are omitted; this finite search "
            "does not prove the pose unreachable. Try a different joint7 seed, "
            "increase Max iters, or choose another solver."
        ),
    )


@dataclass
class Scheme:
    label: str
    description: str
    solve: Callable[[IkProblem, int], SolverOutput]
    default_iters: int
    requires_orientation: bool = False


SCHEMES = {
    "gtd": Scheme(
        "GTDynamics factor graph (LM)",
        "Optimizes link poses and joint angles together over GTDynamics' "
        "kinematic factor graph, with Levenberg-Marquardt warm-started "
        "from the current configuration. Pose goals use "
        "Kinematics.inverse with a PoseGoal and joint-limit factors. "
        "Position-only goals use a PointGoalFactor and do not enforce "
        "joint limits.",
        solve_gtd,
        100,
    ),
    "dls": Scheme(
        "Damped least squares",
        "Levenberg-Marquardt in joint space on the geometric Jacobian. "
        "The damping adapts each step, so it stays stable near "
        "singularities. Joints are clamped to their limits.",
        solve_dls,
        200,
    ),
    "pinv": Scheme(
        "Jacobian pseudo-inverse",
        "Newton-Raphson with the undamped Moore-Penrose pseudo-inverse. "
        "It converges fast near a solution but can jump around near "
        "singularities.",
        solve_pinv,
        200,
    ),
    "transpose": Scheme(
        "Jacobian transpose",
        "Gradient descent with dq = alpha * J^T e and the optimal step "
        "length. Each step is cheap, but convergence is slow, especially "
        "for orientation.",
        solve_transpose,
        1000,
    ),
    "ccd": Scheme(
        "Cyclic coordinate descent (CCD)",
        "Solves for one joint at a time, from the tip to the base, "
        "rotating each joint to bring the end-effector toward the target. "
        "It is robust for position goals and weaker for orientation.",
        solve_ccd,
        300,
    ),
}

if hasattr(gtd, "PandaIKFast"):
    SCHEMES["panda_ikfast"] = Scheme(
        "Panda IKFast (analytical)",
        "Analytical full-pose IK for the detected Panda arm and its fixed tool "
        "frames. Tries the current joint7 first, then samples its limits if "
        "needed. Selects the nearest joint-limit-valid branch at the first "
        "successful sample. Max iters limits free-joint samples (default 33); "
        "singular solutions are omitted.",
        solve_panda_ikfast,
        33,
        requires_orientation=True,
    )


# ----------------------------------------------------------------- diagnostics


def _fmt_limit(j: JointInfo, value: float) -> str:
    unit = "m" if j.type == "prismatic" else "rad"
    return f"{value:.3f} {unit}"


def diagnose(
    p: IkProblem, out: SolverOutput, T: gtsam.Pose3, frames
) -> tuple[list[str], list[str], list[str]]:
    """Explain why a solve missed its target.

    Returns:
        (reasons, warnings, at_limit_joint_names). reasons is empty iff solved.
    """
    req = p.req
    reasons, warnings = [], []
    solved = p.converged(T)
    x = np.array([out.q[j.name] for j in p.chain])
    dof, task = len(p.chain), p.rows

    # Joints at or beyond their limits, and which way the error pulls them.
    at_limit, blocked, violated = [], [], []
    if dof:
        e = p.error(T)
        J = p.jacobian(frames, T)
        g = J.T @ e  # positive: increasing this joint reduces the error
        tol = out.limit_tol
        for i, j in enumerate(p.chain):
            if x[i] > j.upper + 1e-3 or x[i] < j.lower - 1e-3:
                bound = j.upper if x[i] > j.upper else j.lower
                violated.append(
                    f"{j.name} = {_fmt_limit(j, x[i])} (limit "
                    f"{_fmt_limit(j, bound)})"
                )
            if x[i] >= j.upper - tol:
                at_limit.append(j.name)
                if g[i] > 1e-9:
                    blocked.append(
                        f"{j.name} (upper limit " f"{_fmt_limit(j, j.upper)})"
                    )
            elif x[i] <= j.lower + tol:
                at_limit.append(j.name)
                if g[i] < -1e-9:
                    blocked.append(
                        f"{j.name} (lower limit " f"{_fmt_limit(j, j.lower)})"
                    )
        sv = np.linalg.svd(J, compute_uv=False)
        conditioning = float(sv[-1] / sv[0]) if sv[0] > 1e-12 else 0.0
        gradient_ratio = float(np.linalg.norm(g)) / (
            float(sv[0]) * float(np.linalg.norm(e)) + 1e-15
        )

    if solved:
        if dof and conditioning < 1e-2:
            warnings.append(
                "Close to a kinematic singularity (Jacobian condition "
                f"{conditioning:.1e}): small target moves can need large joint "
                "motions."
            )
        if at_limit:
            warnings.append("Joints at their limits: " + ", ".join(at_limit))
        return reasons, warnings, at_limit

    if out.failure_reason:
        reasons.append(out.failure_reason)
        return reasons, warnings, at_limit

    if dof == 0:
        reasons.append(
            f"'{req.ee}' is rigidly attached to the fixed base '{p.model.base}' "
            "(no movable joints in between), so it cannot move."
        )
        return reasons, warnings, at_limit

    anchor, reach = p.reach(frames, x)
    distance = float(np.linalg.norm(req.target.translation() - anchor))
    if distance > reach + req.pos_tol:
        reasons.append(
            f"Target is out of reach: it is {distance:.3f} m from joint "
            f"'{p.chain[0].name}', but the chain can extend at most "
            f"{reach:.3f} m ({distance - reach:.3f} m short)."
        )
    if dof < task:
        what = "6-DoF pose" if req.orientation else "3-D position"
        hint = (
            " Uncheck 'Constrain orientation' to solve for position only."
            if req.orientation
            else ""
        )
        reasons.append(
            f"The chain to '{req.ee}' has only {dof} movable joint"
            f"{'s' if dof != 1 else ''}, fewer than the {task} needed for an "
            f"arbitrary {what}; only a {dof}-D subset of targets is "
            f"reachable.{hint}"
        )
    if violated:
        reasons.append(
            "The solution violates joint limits (this solver does not "
            "enforce them): " + "; ".join(violated)
        )
    if blocked:
        reasons.append(
            "Blocked by joint limits: getting closer would need these joints "
            "to move past their limits: " + ", ".join(blocked) + "."
        )
    if conditioning < 1e-3 and dof >= task:
        reasons.append(
            "The arm is at a kinematic singularity (Jacobian condition "
            f"{conditioning:.1e}): it has lost the ability to move the "
            "end-effector in some direction, e.g. fully stretched or with "
            "aligned joint axes."
        )
    if out.solver_ee_pose is not None:
        gap = float(np.linalg.norm(out.solver_ee_pose.translation() - T.translation()))
        if gap > req.pos_tol:
            reasons.append(
                "The factor graph settled where link poses and joint angles "
                f"disagree: its end-effector link pose is {gap * 1000:.1f} mm "
                "from the pose implied by its joint angles, because the soft "
                "joint constraints were traded off against the goal. This "
                "is a local minimum."
            )
    start_frames, start_T = p.fk(p.x0)
    start_sv = np.linalg.svd(p.jacobian(start_frames, start_T), compute_uv=False)
    if start_sv[0] > 1e-12 and start_sv[-1] / start_sv[0] < 1e-3 and dof >= task:
        reasons.append(
            "The solve started from a singular configuration (for example "
            "the fully stretched home pose), where the Jacobian gives no "
            "useful direction for some motions. Bend the arm first with the "
            "joint sliders, or move the target in smaller steps."
        )
    if (
        out.iterations is not None
        and out.iterations >= out.max_iters
        and out.still_improving
    ):
        reasons.append(
            f"Hit the iteration limit ({out.max_iters}) while the error was "
            "still decreasing. Raise 'Max iterations' or use a faster "
            "converging scheme."
        )
    if not reasons and gradient_ratio < 1e-3:
        reasons.append(
            "Stuck in a local minimum: no small joint motion reduces the "
            "remaining error. Try another scheme, or approach the target from "
            "a different configuration."
        )
    if not reasons and out.stalled:
        reasons.append(
            "The solver stopped making progress before reaching " "the tolerance."
        )
    if not reasons:
        reasons.append(
            "The solver stopped while the error could still be reduced. "
            "Try again from here, raise 'Max iterations', or try another "
            "scheme."
        )

    pos_err, rot_err = p.errors(T)
    residual = f"{pos_err * 1000:.2f} mm"
    if req.orientation:
        residual += f", {math.degrees(rot_err):.2f}°"
    tolerance = f"{req.pos_tol * 1000:.2f} mm"
    if req.orientation:
        tolerance += f", {math.degrees(req.rot_tol):.2f}°"
    reasons.append(
        f"Remaining error {residual} exceeds the tolerance " f"({tolerance})."
    )
    return reasons, warnings, at_limit


# ---------------------------------------------------------------------- solve


def solve(model: RobotModel, scheme_id: str, req: IkRequest) -> dict:
    """Run one IK scheme and return a JSON-ready result with diagnostics."""
    if scheme_id not in SCHEMES:
        raise ModelError(f"Unknown IK scheme '{scheme_id}'.")
    scheme = SCHEMES[scheme_id]
    if scheme_id not in model.scheme_ids(req.ee, req.orientation):
        raise ModelError(
            f"{scheme.label} requires a compatible Panda arm, a frame rigidly "
            "attached to link7, and Constrain orientation enabled."
        )
    max_iters = req.max_iters or scheme.default_iters
    p = IkProblem(model, req)

    start = time.perf_counter()
    try:
        out = (
            scheme.solve(p, max_iters)
            if p.chain
            else SolverOutput(p.to_q(p.x0), 0, max_iters)
        )
    except RuntimeError as e:  # GTSAM/GTDynamics optimizer failures
        frames, T = p.fk(p.x0)
        return {
            "status": "error",
            "q": p.to_q(p.x0),
            "ee_pose": pose_to_json(T),
            "skeleton": model.skeleton(frames),
            "reasons": [f"The {scheme.label} solver raised an error: {e}"],
            "warnings": [],
            "at_limits": [],
            "time_ms": 1000 * (time.perf_counter() - start),
        }
    elapsed = time.perf_counter() - start

    frames = model.link_frames(out.q)
    T = model.ee_pose(frames, req.ee)
    reasons, warnings, at_limit = diagnose(p, out, T, frames)
    pos_err, rot_err = p.errors(T)
    return {
        "status": "failed" if reasons else "solved",
        "q": out.q,
        "ee_pose": pose_to_json(T),
        "skeleton": model.skeleton(frames),
        "pos_err": pos_err,
        "rot_err_deg": math.degrees(rot_err),
        "iterations": out.iterations,
        "max_iters": max_iters,
        "time_ms": 1000 * elapsed,
        "dof": len(p.chain),
        "reasons": reasons,
        "warnings": warnings,
        "at_limits": at_limit,
    }


def forward(model: RobotModel, ee: str, q: dict[str, float]) -> dict:
    frames = model.link_frames(q)
    return {
        "q": {name: float(q.get(name, 0.0)) for name in model.joints},
        "ee_pose": pose_to_json(model.ee_pose(frames, ee)),
        "skeleton": model.skeleton(frames),
    }
