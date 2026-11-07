#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file steap.py
# @brief Simulated STEAP execution: ISAM2 replanning with measurement factors.
# @author GTDynamics contributors

"""Execute a GPMP2 plan state by state with simulated disturbances and replan.

Follows the BAR Lab paper's STEAP loop. The batch GPMP2 solution seeds an ISAM2
Bayes tree. Each support state is commanded in turn; its measured configuration
is fused as a position-only ``PriorFactorVector(X(k), q_measured)`` with the
paper's measurement sigmas; ISAM2 then re-estimates the remaining states before
the next command. The robot is simulated: the measurement is the command plus a
seeded AR(1) disturbance, so no hardware is involved.

ISAM2 takes unguarded linearized steps, which can leap away near stiff limit and
collision factors. An update that moves a remaining state by more than
``REPLAN_TRUST`` is rejected, and the rest of the maneuver replans with warm-started
Levenberg-Marquardt, softly tethered to the batch plan plus a fading correction because
the cost is nearly flat along large motions.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import gtsam
import numpy as np
from gtsam.symbol_shorthand import V, X

from planner import CmrgPlanner, Plan

MEASUREMENT_SIGMA_ARM_DEG = 0.002
MEASUREMENT_SIGMA_GANTRY_M = 0.005
EXTRA_ISAM_ITERATIONS = 3
REPLAN_TRUST = 0.15
FALLBACK_ITERATIONS = 50
FALLBACK_TETHER_SIGMA = 0.05


@dataclass
class Disturbance:
    """Execution disturbance: per-joint AR(1) noise on commanded configurations.

    Gantry sigmas are in metres and arm sigmas in degrees; ``persistence`` is the
    AR(1) coefficient (0 gives independent noise per state).
    """

    gantry_sigma_m: float = 0.010
    arm_sigma_deg: float = 0.10
    persistence: float = 0.8
    seed: int = 0

    def sigmas(self) -> np.ndarray:
        """Return the per-DoF standard deviation for both arms (m, rad)."""
        return np.concatenate([[self.gantry_sigma_m] * 3 + [np.deg2rad(self.arm_sigma_deg)] * 6] * 2)

    def sequence(self, rng: np.random.Generator, count: int, active: np.ndarray) -> np.ndarray:
        """Return (count, dof) disturbance with zero at k = 0 and none on inactive DoFs."""
        sigma = self.sigmas() * active
        noise = np.zeros((count, len(sigma)))
        innovation = np.sqrt(max(1.0 - self.persistence**2, 0.0))
        for k in range(1, count):
            noise[k] = self.persistence * noise[k - 1] + innovation * sigma * rng.standard_normal(len(sigma))
        return noise


@dataclass
class Execution:
    """Result of executing one maneuver: nominal, commanded and measured states."""

    nominal: np.ndarray
    commanded: np.ndarray
    measured: np.ndarray
    open_loop: np.ndarray
    velocities: np.ndarray
    estimates: list[np.ndarray] = field(default_factory=list)
    graph_error: float = 0.0
    final_error: float = 0.0
    solver: str = "isam2"


def measurement_model(dof: int) -> gtsam.noiseModel.Base:
    """Return the paper's diagonal measurement noise: 5 mm gantry, 0.002 degrees arm."""
    sigmas = np.concatenate([[MEASUREMENT_SIGMA_GANTRY_M] * 3 + [np.deg2rad(MEASUREMENT_SIGMA_ARM_DEG)] * 6] * (dof // 9))
    return gtsam.noiseModel.Diagonal.Sigmas(sigmas)


def isam_params() -> gtsam.ISAM2Params:
    """Return Dogleg ISAM2 parameters, as in the paper's STEAP planner."""
    params = gtsam.ISAM2Params()
    params.setOptimizationParams(gtsam.ISAM2DoglegParams())
    return params


def remaining_jump(candidate: gtsam.Values, previous: gtsam.Values, first: int, count: int) -> float:
    """Return the largest change to any state from ``first`` to the end, in m or rad."""
    return max(float(np.max(np.abs(candidate.atVector(X(j)) - previous.atVector(X(j))))) for j in range(first, count))


def tethered_states(plan: Plan, latest: np.ndarray, k: int) -> dict[int, np.ndarray]:
    """Return the batch plan after state ``k`` plus the latest error, fading to zero at the goal."""
    last = plan.state_count - 1
    error = latest - plan.values.atVector(X(k))
    return {j: plan.values.atVector(X(j)) + error * (last - j) / (last - k) for j in range(k + 1, last + 1)}


def replan_batch(plan: Plan, readings: list[np.ndarray], model) -> gtsam.Values:
    """Return a warm-started, tethered Levenberg-Marquardt replan through every measurement so far."""
    graph = gtsam.NonlinearFactorGraph()
    graph.push_back(plan.graph)
    initial = gtsam.Values(plan.values)
    for k, reading in enumerate(readings):
        graph.add(gtsam.PriorFactorVector(X(k), reading, model))
        initial.update(X(k), reading)
    tether = gtsam.noiseModel.Isotropic.Sigma(len(readings[0]), FALLBACK_TETHER_SIGMA)
    for j, state in tethered_states(plan, readings[-1], len(readings) - 1).items():
        graph.add(gtsam.PriorFactorVector(X(j), state, tether))
        initial.update(X(j), state)
    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(FALLBACK_ITERATIONS)
    return gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()


def execute(planner: CmrgPlanner, plan: Plan, disturbance: Disturbance, rng: np.random.Generator,
            feedback: bool = True) -> Execution:
    """Execute ``plan`` with simulated disturbances, replanning after every measurement.

    ``feedback=False`` skips the measurement factors, giving the open-loop run under the same disturbance.
    """
    n, dof = plan.state_count, planner.dof
    nominal = plan.states()
    active = (np.abs(nominal[-1] - nominal[0]) > 1e-6).astype(float)
    noise = disturbance.sequence(rng, n, active)
    isam = gtsam.ISAM2(isam_params())
    isam.update(plan.graph, plan.values)
    for _ in range(EXTRA_ISAM_ITERATIONS):
        isam.update()
    result = isam.calculateEstimate()
    use_isam = remaining_jump(result, plan.values, 0, n) <= REPLAN_TRUST
    if not use_isam:
        result = plan.values
    model = measurement_model(dof)
    commanded, measured, estimates = [], [], []
    for k in range(n):
        command = np.clip(result.atVector(X(k)), planner.lower, planner.upper)
        reading = command + noise[k] if feedback else nominal[k] + noise[k]
        commanded.append(command)
        measured.append(reading)
        if feedback:
            fused = gtsam.NonlinearFactorGraph()
            fused.add(gtsam.PriorFactorVector(X(k), reading, model))
            if use_isam:
                isam.update(fused)
                candidate = isam.calculateEstimate()
                use_isam = remaining_jump(candidate, result, k + 1, n) <= REPLAN_TRUST if k + 1 < n else True
            if use_isam:
                result = candidate
            else:
                result = replan_batch(plan, measured, model)
        estimates.append(np.array([result.atVector(X(j)) for j in range(n)]))
    velocities = np.array([result.atVector(V(k)) for k in range(n)])
    measured_array = np.array(measured)
    return Execution(
        nominal, np.array(commanded), measured_array, nominal + noise, velocities, estimates,
        plan.error, float(np.max(np.abs(measured_array[-1] - nominal[-1]))),
        "isam2" if use_isam else "levenberg-marquardt")
