# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_settings.py
# @brief Explicit defaults for the G1 coupled walking example.
# @author GTDynamics contributors
"""Explicit numerical recipe for the G1 coupled walking example.

SI units except fields named Degrees; native helpers have no hidden defaults.
Zero initial motion, optional gravity support, supported walking slices,
chronological elimination and per-iteration reporting are deliberate choices of this experiment.
"""


from typing import Any


def solver_settings() -> dict[str, Any]:
    """Return a fresh editable recipe; no global mutable configuration."""
    return {
        "initialization": "contact-gp-dynamics",
        "maxIterations": 5,
        "qpMaxIterations": 1000,
        "qpBackend": "gtsam",
        "admmRho": 1.0,
        "admmUseDiagonalMetric": False,
        "admmConstraintSplitting": False,
        "admmProjectionThreads": 1,
        "admmProjectionSweeps": 2000,
        "admmAbsoluteTolerance": 1e-9,
        "admmRelativeTolerance": 1e-9,
        "linearSolver": "automatic",
        "equalityOnly": False,
        "qpUseProjectedGradientInitialization": False,
        "qpUseFeasibleStep": True,
        "qpUseTemporalOrdering": False,
        "gaugeSlice": 0,
        "retractionThreads": 8,
        "historyWindow": 1,
        "useLinkCollocation": True,
        "poseRotationSigmaDegrees": 0.05,
        "poseTranslationSigma": 0.0005,
        "twistAngularSigmaDegrees": 2.0,
        "twistLinearSigma": 0.02,
        "jointPositionSigmaDegrees": 0.05,
        "jointVelocitySigmaDegrees": 2.0,
        "stanceVelocitySigma": 0.1,
        "gravity": [0.0, 0.0, -9.81],
        "effortSigma": 100.0,
        "wrenchSigma": 500.0,
        "torqueSmoothSigma": 100.0,
        "wrenchSmoothSigma": 500.0,
        "baseAngularAccelerationSigmaDegrees": 1000.0,
        "baseLinearAccelerationSigma": 30.0,
        "jointAccelerationSigmaDegrees": 2000.0,
        "projectionAngularAccelerationSigmaDegrees": 90.0,
        "projectionLinearAccelerationSigma": 3.0,
        "projectionJointAccelerationSigmaDegrees": 1000.0,
        "projectionForceSigma": 500.0,
        "projectionMomentSigma": 30.0,
        "projectionFeasibilityTolerance": 1e-07,
        "projectionActiveTolerance": 1e-06,
        "projectionStationarityTolerance": 0.0001,
        "relativeErrorTol": 1e-05,
        "absoluteErrorTol": 0.0001,
        "lambdaInitial": 0.01,
        "diagonalDamping": True,
    }


def configure_solve(
    initialization: str,
    gauge_slice: int,
    gauge_time: float,
    max_iterations: int,
    retraction_threads: int,
    *,
    qp_backend: str = "gtsam",
    first_qp_backend: str | None = None,
    linear_solver: str = "automatic",
    equality_only: bool = False,
    qp_use_feasible_step: bool = True,
    qp_use_projected_gradient_initialization: bool = False,
    qp_use_temporal_ordering: bool = False,
    qp_max_iterations: int = 1000,
    admm_rho: float = 1.0,
    admm_use_diagonal_metric: bool = False,
    admm_constraint_splitting: bool = False,
    admm_projection_threads: int = 1,
    use_link_collocation: bool = True,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return native options and report metadata for the selected root gauge.

    Translation and yaw select coordinates at the first retained support slice.
    Roll and pitch remain free; the initializer supplies only their starting
    guess. Every retained configuration remains free in the coupled solve.
    """
    if linear_solver not in ("automatic", "gaussian", "qp"):
        raise ValueError(f"Unknown linear solver: {linear_solver}")
    if linear_solver == "gaussian" and not equality_only:
        raise ValueError("Gaussian solves require equality_only=True")
    if qp_backend not in ("gtsam", "proxqp", "admm"):
        raise ValueError(f"Unknown QP backend: {qp_backend}")
    uses_qp = linear_solver == "qp" or (
        linear_solver == "automatic" and not equality_only
    )
    if (
        uses_qp
        and qp_backend in ("proxqp", "admm")
        and (qp_use_feasible_step or qp_use_projected_gradient_initialization)
    ):
        raise ValueError(
            "ProxQP and ADMM require --qp-step full and projected gradients off"
        )
    if first_qp_backend is not None and (
        first_qp_backend not in ("gtsam", "proxqp")
        or qp_backend != "admm"
        or not uses_qp
    ):
        raise ValueError(
            "The first QP backend requires a GTSAM or ProxQP start followed by ADMM"
        )
    options = solver_settings()
    options.update(
        initialization=initialization,
        gaugeSlice=gauge_slice,
        maxIterations=max_iterations,
        retractionThreads=retraction_threads,
        qpBackend=qp_backend,
        qpMaxIterations=qp_max_iterations,
        admmRho=admm_rho,
        admmUseDiagonalMetric=admm_use_diagonal_metric,
        admmConstraintSplitting=admm_constraint_splitting,
        admmProjectionThreads=admm_projection_threads,
        useLinkCollocation=use_link_collocation,
        linearSolver=linear_solver,
        equalityOnly=equality_only,
        qpUseFeasibleStep=qp_use_feasible_step,
        qpUseProjectedGradientInitialization=qp_use_projected_gradient_initialization,
        qpUseTemporalOrdering=qp_use_temporal_ordering,
    )
    if first_qp_backend is not None:
        options["firstIterationQpBackend"] = first_qp_backend
    metadata = {
        "gauge": {
            "coordinates": ["world_x", "world_y", "world_z", "yaw"],
            "rotationFree": ["roll", "pitch"],
            "translationSigmaMeters": 1.0,
            "yawSigmaRadians": 1.0,
            "initialTilt": "Initializer guess only; estimated from observations",
        },
        "gaugeSlice": gauge_slice,
        "gaugeTimeSeconds": gauge_time,
        "maxIterations": max_iterations,
        "retractionThreads": retraction_threads,
        "solverSettings": options.copy(),
        "collocation": {
            name: options[name]
            for name in (
                "poseRotationSigmaDegrees",
                "poseTranslationSigma",
                "twistAngularSigmaDegrees",
                "twistLinearSigma",
                "jointPositionSigmaDegrees",
                "jointVelocitySigmaDegrees",
                "stanceVelocitySigma",
            )
        },
    }
    return options, metadata
