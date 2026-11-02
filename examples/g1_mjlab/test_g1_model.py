"""Cross-check the portable model, COM conversion and sole frames against MuJoCo."""

from pathlib import Path
import json
import tempfile
import unittest

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np

from g1_model import VERSIONS, prepare_model, sole_frame
from mujoco_io import import_robot, model_bodies, pose


class CanonicalModelTests(unittest.TestCase):
    def test_portable_export_and_contact_fk(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = prepare_model(Path(temporary))
            manifest = json.loads((directory / "provenance.json").read_text())
            self.assertEqual(manifest["packages"], VERSIONS)
            self.assertEqual(manifest["entry"], "g1_mjlab")
            model = mujoco.MjModel.from_xml_path(str(directory / "scene.xml"))
            binary = mujoco.MjModel.from_binary_path(str(directory / "model.mjb"))
            self.assertEqual((model.nq, model.nv, model.nu), (36, 35, 29))
            self.assertEqual(model.opt.timestep, 0.005)
            for field in ("timestep", "iterations", "solver", "integrator", "cone"):
                self.assertEqual(getattr(model.opt, field), getattr(binary.opt, field))
            np.testing.assert_allclose(model.actuator_gainprm, binary.actuator_gainprm)
            np.testing.assert_array_equal(model.geom_contype, binary.geom_contype)
            robot = import_robot(model_bodies(model))
            data = mujoco.MjData(model)
            data.qpos[:3] = [1.0, 2.0, 1.0]
            data.qpos[3:7] = [0.9, 0.1, -0.2, 0.3]
            data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
            data.qpos[7:] = np.linspace(-0.2, 0.2, 29)
            mujoco.mj_forward(model, data)
            root = robot.link("pelvis")
            known = gtsam.Values()
            for joint in robot.joints():
                address = model.jnt_qposadr[model.joint("robot/" + joint.name()).id]
                known.insert(
                    gtd.JointAngleKey(joint.id(), 0),
                    float(data.qpos[address] - model.qpos0[address]),
                )
            # COM inertia axes need not coincide with the body axes.
            worldTcom = pose(data.xpos[root.id()], data.xquat[root.id()]) * pose(
                model.body_ipos[root.id()], model.body_iquat[root.id()]
            )
            known.insert(gtd.PoseKey(root.id(), 0), worldTcom)
            known.insert(gtd.TwistKey(root.id(), 0), np.zeros(6))
            fk = robot.forwardKinematics(known, 0, "pelvis")
            for side in ("left", "right"):
                link = robot.link(f"{side}_ankle_roll_link")
                bodyTcontact = gtsam.Pose3(sole_frame(model, side))
                bodyTcom = pose(model.body_ipos[link.id()], model.body_iquat[link.id()])
                contact = gtd.PoseOnLink(link, bodyTcom.inverse() * bodyTcontact)
                expected = (
                    pose(data.xpos[link.id()], data.xquat[link.id()]) * bodyTcontact
                )
                np.testing.assert_allclose(
                    contact.predict(fk, 0).matrix(), expected.matrix(), atol=1e-10
                )
            site = model.site("robot/imu_in_pelvis")
            bodyTimu = pose(site.pos, site.quat)
            comTimu = (
                pose(model.body_ipos[root.id()], model.body_iquat[root.id()]).inverse()
                * bodyTimu
            )
            np.testing.assert_allclose(
                (worldTcom * comTimu).translation(), data.site_xpos[site.id], atol=1e-10
            )


if __name__ == "__main__":
    unittest.main()
