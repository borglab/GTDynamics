"""Explicit numerical recipe for the G1 coupled walking example.

SI units except fields named Degrees; native helpers have no hidden defaults.
Zero initial dynamics, supported walking slices, chronological elimination and
per-iteration reporting are deliberate choices of this experiment.
"""


def solver_settings() -> dict:
    """Return a fresh editable recipe; no global mutable configuration."""
    return {
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
