"""Executor settings, the checks every plan must pass, and the command law.

The planner's own limits are soft (hinge costs), so the executor checks each
plan before it uses it, independently of the planner: joint limits, joint
speed and acceleration against the planner's limits (with a tolerance),
and the clearance of the arm's spheres to the walls and floor, by its own
forward kinematics. A plan that fails stops the run.

The command is the repo's plan_executor law: the plan's velocity plus a
clipped position correction, clipped again to a hard speed cap that must
stay below ur_bridge's max_joint_speed (the bridge faults above it).
"""

from __future__ import annotations

from dataclasses import dataclass

from . import kinematics


@dataclass
class Settings:
    rate_hz: float = 50.0             # command rate
    kp: float = 1.5                   # 1/s, position correction gain
    max_correction: float = 0.1       # rad/s, clip of the position correction
    command_cap: float = 0.55         # rad/s, hard clip of every command (< ur_bridge max_joint_speed)
    tracking_abort: float = 0.08      # rad: stop when any joint is further than this from the plan
    stale_after: float = 0.2          # s: stop when the arm's state is older than this
    start_tolerance: float = 0.02     # rad: how close to q_start the arm must be to begin
    limit_tolerance: float = 1.10     # plans may exceed v_limit and u_limit by this factor at most
    min_environment_clearance: float = 0.0   # m: plans must keep every sphere off the walls and floor
    plan_samples: int = 10            # checks per interval
    late_fraction: float = 0.8        # stop if a re-plan takes longer than this fraction of dt
    trace_hz: float = 25.0            # measured samples kept for the replay
    move_speed: float = 0.2           # rad/s, peak joint speed when moving to the start
    settle_time: float = 1.0          # s of position hold after the last state
    ghost_timeout: float = 120.0      # s to wait for the planner's ghost run


class Limits:
    """The planner's limits, as announced in its ghost message."""

    def __init__(self, message):
        self.v_limit = list(message['v_limit'])
        self.u_limit = list(message['u_limit'])
        self.q_lower = list(message['q_lower'])
        self.q_upper = list(message['q_upper'])
        self.dt = float(message['dt'])
        self.num_intervals = int(message['num_intervals'])


def check_path(path, t_from, limits, settings, robot=None, planes=None, times=None):
    """Problems of a plan (or TimedPath) from t_from on; [] if it may be executed."""
    problems = []
    tol = settings.limit_tolerance
    times = times if times is not None else (
        path.samples(t_from, settings.plan_samples) if hasattr(path, 'u') else path.samples(t_from))
    worst_gap, worst_plane = float('inf'), None
    for t in times:
        q, v, u = path.evaluate(t)
        for j in range(len(q)):
            if not limits.q_lower[j] <= q[j] <= limits.q_upper[j]:
                problems.append(f'joint {j + 1} leaves its limits at t = {t:.2f} s')
            if abs(v[j]) > tol * limits.v_limit[j]:
                problems.append(f'joint {j + 1} speed {abs(v[j]):.3f} rad/s exceeds the '
                                f'{limits.v_limit[j]:.3f} rad/s limit at t = {t:.2f} s')
            if abs(u[j]) > tol * limits.u_limit[j]:
                problems.append(f'joint {j + 1} acceleration {abs(u[j]):.3f} rad/s^2 exceeds the '
                                f'{limits.u_limit[j]:.3f} rad/s^2 limit at t = {t:.2f} s')
        if robot is not None and planes:
            gap, name = kinematics.plane_clearance(robot, q, planes)
            if gap < worst_gap:
                worst_gap, worst_plane = gap, name
        if len(problems) >= 3:
            break
    if worst_gap < settings.min_environment_clearance:
        problems.append(f'the plan brings the arm within {100 * worst_gap:.1f} cm of the {worst_plane}')
    return problems


def clip(value, bound):
    return max(-bound, min(bound, value))


def command(q_ref, v_ref, q, settings):
    """Joint velocity command: v_ref + clip(kp (q_ref - q)), clipped to the hard cap."""
    return [clip(vr + clip(settings.kp * (qr - qm), settings.max_correction), settings.command_cap)
            for qr, vr, qm in zip(q_ref, v_ref, q)]


def hold(q_target, q, settings):
    """Command that pulls the arm to q_target and holds it there."""
    return command(q_target, [0.0] * len(q), q, settings)
