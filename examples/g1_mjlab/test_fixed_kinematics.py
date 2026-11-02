# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file test_fixed_kinematics.py
# @brief Test GP acceleration priors, frame conventions, and source sampling.
# @author GTDynamics contributors

"""Independent motion and frame checks for GP-derived acceleration observations."""

from __future__ import annotations

from types import SimpleNamespace
import unittest

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np

from fixed_kinematics import acceleration_prior_values, prepare_fixed_kinematics
from g1_model import load_g1_model
from mujoco_io import import_robot, model_bodies, pose


class FixedKinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_g1_model()
        cls.robot = import_robot(model_bodies(cls.model))

    def recording(self, rotating: bool) -> SimpleNamespace:
        """Fit known rigid motion from independent compiled MuJoCo body poses."""
        model = self.model
        times = np.array([0, 0.07, 0.18, 0.3, 0.47, 0.6, 0.8, 0.93, 1.0])
        data = mujoco.MjData(model)
        joints = self.robot.orderedJoints()
        problem = gtd.FloatingBaseInterpolator(
            self.robot,
            "pelvis",
            np.ones(6),
            np.ones(len(joints)),
            np.full(len(joints), 1e-4),
        )
        omega, acceleration = np.array([0.1, 0.2, 0.4]), np.array([0.3, -0.2, 0.1])
        body_to_com = pose(model.body_ipos[1], model.body_iquat[1])
        for time in times:
            data.qpos[:] = model.qpos0
            if rotating:
                rotation = gtsam.Rot3.Expmap(omega * time)
                data.qpos[:3] = 0
                data.qpos[3:7] = rotation.toQuaternion().coeffs()[[3, 0, 1, 2]]
                world_velocity = np.cross(omega, rotation.rotate(model.body_ipos[1]))
            else:
                data.qpos[:3] += 0.5 * acceleration * time**2
                world_velocity = acceleration * time
            mujoco.mj_forward(model, data)
            world_com = pose(data.xpos[1], data.xquat[1]) * body_to_com
            angles = np.array(
                [
                    data.qpos[model.joint("robot/" + joint.name()).qposadr[0]]
                    for joint in joints
                ]
            )
            problem.addKnot(float(time), world_com, angles)
            problem.addBaseVelocityMeasurement(float(time), world_velocity, 1e-8)
        return SimpleNamespace(
            model=model,
            interpolation=problem.solve(),
            knot_times=times,
            dense_times=times,
            record={
                "sampling": {"sourceIndices": list(range(10, 19))},
                "contactPhaseInference": {
                    "phases": [
                        {"start": 10, "end": 18, "contacts": ["left", "right"]},
                    ]
                },
            },
        )

    def test_constant_world_acceleration(self) -> None:
        walk = self.recording(rotating=False)
        fixed = prepare_fixed_kinematics(walk, 0.18, 0.8)
        np.testing.assert_array_equal(fixed.indices, np.arange(2, 7))
        self.assertEqual([slice_.k for slice_ in fixed.slices], list(range(12, 17)))
        root = fixed.robot.link("pelvis").id()
        for slice_ in fixed.slices:
            k = slice_.k
            self.assertFalse(fixed.values.exists(gtd.TwistAccelKey(root, k)))
            rotation = gtd.Pose(fixed.values, root, k).rotation().matrix()
            twist = gtd.Twist(fixed.values, root, k)
            derivative = gtd.TwistAccel(fixed.acceleration_priors, root, k)
            physical = rotation @ (derivative[3:] + np.cross(twist[:3], twist[3:]))
            np.testing.assert_allclose(physical, [0.3, -0.2, 0.1], atol=1e-7)

    def test_body_derivative_and_outer_endpoint_stencils(self) -> None:
        walk = self.recording(rotating=True)
        root = self.robot.link("pelvis").id()
        # Constant body twist has zero body derivative, including for a COM
        # offset from the rotation origin. Its physical acceleration is nonzero.
        for time in [0, 0.18, 0.6, 1.0]:
            prior = acceleration_prior_values(
                walk.interpolation, self.robot, time, 7, 0.001
            )
            derivative = gtd.TwistAccel(prior, root, 7)
            np.testing.assert_allclose(derivative, 0, atol=1e-7)
            state = walk.interpolation.interpolate(time)
            physical = state.basePose.rotation().rotate(
                derivative[3:] + np.cross(state.baseTwist[:3], state.baseTwist[3:])
            )
            omega = np.array([0.1, 0.2, 0.4])
            expected = np.cross(omega, np.cross(omega, state.basePose.translation()))
            np.testing.assert_allclose(physical, expected, atol=1e-7)

    def test_invalid_differentiation_step(self) -> None:
        walk = self.recording(rotating=False)
        for step in [0, -1, np.nan]:
            with self.assertRaises(ValueError):
                acceleration_prior_values(walk.interpolation, self.robot, 0.3, 0, step)


if __name__ == "__main__":
    unittest.main()
