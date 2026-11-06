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
        "gaugeSlice": 0,
        "retractionThreads": 8,
        "historyWindow": 1,
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
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return native options and report metadata for the selected root gauge.

    Translation and yaw select coordinates at the first retained support slice.
    Roll and pitch remain free; the initializer supplies only their starting
    guess. Every retained configuration remains free in the coupled solve.
    """
    options = solver_settings()
    options.update(
        initialization=initialization,
        gaugeSlice=gauge_slice,
        maxIterations=max_iterations,
        retractionThreads=retraction_threads,
    )
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
