"""Joint-Jacobian body velocities agree with differentiated relative G1 FK."""

import unittest

import mujoco
import numpy as np

from g1_model import load_g1_model, sole_frame
from mujoco_io import point_velocity_in_frame


class BodyVelocityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_g1_model()
        cls.pelvis = cls.model.body("robot/pelvis").id

    def test_relative_jacobian_against_finite_difference(self) -> None:
        model = self.model
        data = mujoco.MjData(model)
        data.qpos[:] = model.qpos0
        rng = np.random.default_rng(17)
        data.qvel[6:] = rng.normal(0, 0.5, model.nv - 6)
        data.qvel[:6] = [1.3, -0.7, 0.2, 0.4, -0.3, 0.6]
        mujoco.mj_forward(model, data)
        for side in ("left", "right"):
            body = model.body(f"robot/{side}_ankle_roll_link").id
            point = sole_frame(model, side)[:3, 3]
            velocity = point_velocity_in_frame(model, data, body, point, self.pelvis)
            positions = []
            for sign in (-1, 1):
                displaced = mujoco.MjData(model)
                displaced.qpos[:] = data.qpos
                mujoco.mj_integratePos(model, displaced.qpos, data.qvel, sign * 1e-6)
                mujoco.mj_forward(model, displaced)
                world = (
                    displaced.xpos[body] + displaced.xmat[body].reshape(3, 3) @ point
                )
                positions.append(
                    displaced.xmat[self.pelvis].reshape(3, 3).T
                    @ (world - displaced.xpos[self.pelvis])
                )
            np.testing.assert_allclose(
                velocity, (positions[1] - positions[0]) / 2e-6, atol=1e-8
            )
            data.qvel[:6] = 0
            np.testing.assert_allclose(
                velocity,
                point_velocity_in_frame(model, data, body, point, self.pelvis),
                atol=1e-12,
            )

    def test_world_reference_does_not_change_body_velocity(self) -> None:
        model = self.model
        data = mujoco.MjData(model)
        data.qpos[:] = model.qpos0
        data.qvel[model.jnt_dofadr[model.joint("robot/left_hip_pitch_joint").id]] = 0.7
        body = model.body("robot/left_ankle_roll_link").id
        point = sole_frame(model, "left")[:3, 3]
        mujoco.mj_forward(model, data)
        before = point_velocity_in_frame(model, data, body, point, self.pelvis)
        data.qpos[:3] = [10, -5, 3]
        data.qpos[3:7] = np.array([0.7, 0.2, -0.3, 0.4]) / np.linalg.norm(
            [0.7, 0.2, -0.3, 0.4]
        )
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(
            before,
            point_velocity_in_frame(model, data, body, point, self.pelvis),
            atol=1e-12,
        )
        data.qvel[:] = 0
        np.testing.assert_allclose(
            point_velocity_in_frame(model, data, body, point, self.pelvis),
            0,
            atol=1e-12,
        )


if __name__ == "__main__":
    unittest.main()
