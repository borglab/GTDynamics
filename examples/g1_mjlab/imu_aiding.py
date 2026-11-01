"""Gal3 IMU aiding for the G1 kinematic trajectory example.

X_k = world_T_linkCOM, v_k = world linear velocity of that COM. The measured
Open-IO vectors use pelvis axes at the pelvis IMU site. GTSAM's body_P_sensor
maps those vectors into the COM frame and applies its centripetal correction.
The standard correction omits tangential (angular-acceleration) lever-arm terms.
One ConstantBias in measurement axes is shared by every interval. No reference
poses after the initial gauge, or reference velocities, enter this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from mujoco_io import KinematicRecording, load_model, load_recording, pose

BIAS_KEY: int = gtsam.symbol("b", 0)


def preintegrate_intervals(
    times: NDArray[np.float64],
    acceleration: NDArray[np.float64],
    angular_rate: NDArray[np.float64],
    boundaries: NDArray[np.float64],
    params: gtsam.PreintegrationParams,
    bias: gtsam.imuBias.ConstantBias,
) -> list[gtsam.PreintegratedImuMeasurementsG]:
    """Integrate native timestamps, splitting samples exactly at slice times.

    Measurements are linearly interpolated within each native interval and
    evaluated at each subinterval midpoint. No endpoint extrapolation or
    assumed 200 Hz dt is used. Bracketing measurements must be supplied.
    """
    if (
        len(times) < 2
        or acceleration.shape != (len(times), 3)
        or angular_rate.shape != (len(times), 3)
        or not all(
            np.isfinite(a).all()
            for a in (times, acceleration, angular_rate, boundaries)
        )
        or not np.all(np.diff(times) > 0)
        or not np.all(np.diff(boundaries) > 0)
    ):
        raise ValueError("IMU arrays must be finite, aligned, and strictly ordered")
    if len(boundaries) < 2 or boundaries[0] < times[0] or boundaries[-1] > times[-1]:
        raise ValueError(
            "Native IMU timestamps must bracket all trajectory slice times"
        )
    pims: list[gtsam.PreintegratedImuMeasurementsG] = []
    for start, end in zip(boundaries, boundaries[1:]):
        pim = gtsam.PreintegratedImuMeasurementsG(params, bias)
        cursor = float(start)
        index = int(np.searchsorted(times, cursor, side="right") - 1)
        while cursor < end:
            stop = min(float(times[index + 1]), float(end))
            fraction = ((cursor + stop) / 2 - times[index]) / (
                times[index + 1] - times[index]
            )
            acc = (1 - fraction) * acceleration[index] + fraction * acceleration[
                index + 1
            ]
            gyro = (1 - fraction) * angular_rate[index] + fraction * angular_rate[
                index + 1
            ]
            pim.integrateMeasurement(acc, gyro, stop - cursor)
            cursor = stop
            index += 1
        if abs(pim.deltaTij() - (end - start)) > 1e-10:
            raise RuntimeError("Preintegration duration differs from slice interval")
        pims.append(pim)
    return pims


@dataclass
class ImuAiding:
    factors: gtsam.NonlinearFactorGraph
    metadata: dict[str, Any]
    bias: gtsam.imuBias.ConstantBias

    def initialize(self, walk: KinematicRecording, seed: gtsam.Values) -> gtsam.Values:
        """Seed COM velocities from the initializer's poses, without LIO velocities."""
        values = gtsam.Values(seed)
        positions = np.array(
            [
                seed.atPose3(gtd.PoseKey(walk.root.id(), k)).translation()
                for k in range(walk.count)
            ]
        )
        velocities = np.gradient(positions, walk.times, axis=0)
        for k, velocity in enumerate(velocities):
            values.insert(gtd.LinearVelocityKey(walk.root.id(), k), velocity)
        values.insert(BIAS_KEY, self.bias)
        return values


def build_imu_aiding(walk: KinematicRecording, directory: Path) -> ImuAiding:
    """Use the wrapped trajectory builder; supply one bias and a broad speed prior."""
    arrays, recording_metadata = load_recording(directory)
    calibration = recording_metadata.get("motionSource", {}).get(
        "stationaryImuCalibration"
    )
    model = load_model(directory)
    root = model.body("robot/pelvis").id
    site = model.site("robot/imu_in_pelvis")
    # Open-IO has already re-expressed the vectors in pelvis axes. The site
    # translation remains physical; do not apply a sensor-axis rotation twice.
    body_T_com: gtsam.Pose3 = pose(model.body_ipos[root], model.body_iquat[root])
    body_T_sensor = gtsam.Pose3(gtsam.Rot3(), site.pos)
    com_T_sensor: gtsam.Pose3 = body_T_com.inverse() * body_T_sensor
    params: gtsam.PreintegrationParams = gtsam.PreintegrationParams.MakeSharedU(9.81)
    params.setBodyPSensor(com_T_sensor)
    # Continuous-time noise densities; provisional settings, not a sensor calibration.
    params.setAccelerometerCovariance(np.eye(3) * 0.1**2)
    params.setGyroscopeCovariance(np.eye(3) * 0.01**2)
    params.setIntegrationCovariance(np.eye(3) * 1e-8)
    gyro_bias = np.zeros(3)
    gyro_sigma = 0.05
    if calibration is not None:
        gyro_bias = np.asarray(calibration["gyroMeanRadiansPerSecond"], dtype=float)
        gyro_sigma = float(calibration["gyroPriorSigmaRadiansPerSecond"])
        if (
            gyro_bias.shape != (3,)
            or not np.isfinite(gyro_bias).all()
            or not np.isfinite(gyro_sigma)
            or gyro_sigma <= 0
        ):
            raise ValueError(
                "Stationary gyro calibration must be finite with positive sigma"
            )
    bias = gtsam.imuBias.ConstantBias(np.zeros(3), gyro_bias)
    bias_prior_sigmas = np.array([0.5] * 3 + [gyro_sigma] * 3)
    pims = preintegrate_intervals(
        arrays["imu_time"],
        arrays["imu_acceleration"],
        arrays["imu_angular_rate"],
        walk.times,
        params,
        bias,
    )
    factors: gtsam.NonlinearFactorGraph = gtd.Kinematics().imuFactors(
        walk.trajectory, walk.root.id(), pims, BIAS_KEY
    )
    factors.add(
        gtsam.PriorFactorConstantBias(
            BIAS_KEY,
            bias,
            gtsam.noiseModel.Diagonal.Sigmas(bias_prior_sigmas),
        )
    )
    # The same prior is used for both initializers; no reference velocity input.
    factors.add(
        gtsam.PriorFactorVector(
            gtd.LinearVelocityKey(walk.root.id(), 0),
            np.zeros(3),
            gtsam.noiseModel.Isotropic.Sigma(3, 3.0),
        )
    )
    return ImuAiding(
        factors,
        {
            "enabled": True,
            "factor": "GalileanImuFactor",
            "intervals": len(pims),
            "intervalSeconds": [pim.deltaTij() for pim in pims],
            "poseFrame": "world_T_linkCOM",
            "velocityFrame": "world; linear velocity of link COM",
            "biasFrame": "pelvis IMU measurement axes",
            "biasVariables": 1,
            "com_T_sensor": com_T_sensor.matrix().tolist(),
            "gravityWorld": [0.0, 0.0, -9.81],
            "accelerometerNoiseDensity": 0.1,
            "gyroscopeNoiseDensity": 0.01,
            "integrationCovariance": 1e-8,
            "biasPriorMean": bias.vector().tolist(),
            "biasPriorSigmas": bias_prior_sigmas.tolist(),
            "stationaryCalibration": calibration,
            "initialVelocityPriorSigma": 3.0,
            "sensorCorrection": "GTSAM rotation and centripetal lever arm; tangential angular-acceleration term omitted",
            "integration": "native timestamps; piecewise-linear measurements, midpoint integration, exact slice boundaries",
        },
        bias,
    )
