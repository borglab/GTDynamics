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


def measurement_model(dof: int) -> gtsam.noiseModel.Base:
    """Return the paper's diagonal measurement noise: 5 mm gantry, 0.002 degrees arm."""
    sigmas = np.concatenate([[MEASUREMENT_SIGMA_GANTRY_M] * 3 + [np.deg2rad(MEASUREMENT_SIGMA_ARM_DEG)] * 6] * (dof // 9))
    return gtsam.noiseModel.Diagonal.Sigmas(sigmas)


def isam_params() -> gtsam.ISAM2Params:
    """Return Dogleg ISAM2 parameters, as in the paper's STEAP planner."""
    params = gtsam.ISAM2Params()
    params.setOptimizationParams(gtsam.ISAM2DoglegParams())
    return params


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
            isam.update(fused)
            result = isam.calculateEstimate()
        estimates.append(np.array([result.atVector(X(j)) for j in range(n)]))
    velocities = np.array([result.atVector(V(k)) for k in range(n)])
    measured_array = np.array(measured)
    return Execution(
        nominal, np.array(commanded), measured_array, nominal + noise, velocities, estimates,
        plan.error, float(np.max(np.abs(measured_array[-1] - nominal[-1]))))
