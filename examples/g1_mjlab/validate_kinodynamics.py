"""Check saved coupled arrays against MuJoCo inverse dynamics and sole cones.

This is a validation oracle only; it does not initialize or modify the solve.
MuJoCo constraints are disabled. Its inverse forces plus passive forces are
compared with estimated motor torques and the generalized sole forces, because
GTDynamics models inertial/gravity mechanics without MuJoCo's passive damping.
"""

from __future__ import annotations
import argparse
import json
from pathlib import Path
import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from g1_model import load_g1_model
from mujoco_io import pose
from sole_support import support_rectangle


def validate(directory: Path) -> dict:
    a = dict(np.load(directory / "kinodynamics.npz", allow_pickle=False))
    model = load_g1_model()
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
    data = mujoco.MjData(model)
    root = int(a["root_id"])
    root_column = a["link_ids"].tolist().index(root)
    body_T_com = pose(model.body_ipos[root], model.body_iquat[root])
    joints = [model.joint("robot/" + name).id for name in a["joint_names"]]
    qa = model.jnt_qposadr[joints]
    da = model.jnt_dofadr[joints]
    errors, speeds = [], []
    for k in np.unique(np.linspace(0, len(a["times"]) - 1, 21, dtype=int)):
        world_T_com = gtsam.Pose3(a["poses"][k, root_column])
        world_T_body = world_T_com * body_T_com.inverse()
        q = world_T_body.rotation().toQuaternion()
        data.qpos[:] = model.qpos0
        data.qpos[:7] = [*world_T_body.translation(), q.w(), q.x(), q.y(), q.z()]
        data.qpos[qa] = a["joint_angles"][k] + model.qpos0[qa]
        v = body_T_com.AdjointMap() @ a["body_twists"][k, root_column]
        dv = body_T_com.AdjointMap() @ a["body_twist_accelerations"][k, root_column]
        R = world_T_body.rotation().matrix()
        data.qvel[:3], data.qvel[3:6] = R @ v[3:], v[:3]
        data.qacc[:3] = R @ (dv[3:] + np.cross(v[:3], v[3:]))
        data.qacc[3:6] = dv[:3]
        data.qvel[da], data.qacc[da] = (
            a["joint_velocities"][k],
            a["joint_accelerations"][k],
        )
        mujoco.mj_inverse(model, data)
        applied = np.zeros(model.nv)
        applied[da] = a["joint_torques"][k]
        for foot, link in enumerate(a["contact_link_ids"]):
            column = a["link_ids"].tolist().index(link)
            com_T_sole = gtsam.Pose3(a["contact_transforms"][foot])
            world_T_sole = gtsam.Pose3(a["poses"][k, column]) * com_T_sole
            wrench = a["wrenches"][k, foot]
            R = world_T_sole.rotation().matrix()
            mujoco.mj_applyFT(
                model,
                data,
                R @ wrench[3:],
                R @ wrench[:3],
                world_T_sole.translation(),
                int(link),
                applied,
            )
            if a["contact"][k, foot]:
                speeds.append(
                    np.linalg.norm(
                        (
                            com_T_sole.inverse().AdjointMap()
                            @ a["body_twists"][k, column]
                        )[3:]
                    )
                )
        errors.append(data.qfrc_inverse + data.qfrc_passive - applied)
    errors = np.asarray(errors)
    cone_violation = 0.0
    swing = 0.0
    for foot, side in enumerate(("left", "right")):
        A = gtd.RectangularContactWrench(*support_rectangle(model, side), 0.8).matrix()
        for k, active in enumerate(a["contact"][:, foot]):
            if active:
                cone_violation = max(
                    cone_violation, float(np.max(A @ a["wrenches"][k, foot]))
                )
            else:
                swing = max(swing, float(np.linalg.norm(a["wrenches"][k, foot])))
    result = {
        "samples": len(errors),
        "maxMuJoCoForceDisagreementN": float(np.max(np.abs(errors[:, :3]))),
        "maxMuJoCoMomentDisagreementNm": float(np.max(np.abs(errors[:, 3:6]))),
        "maxMuJoCoJointTorqueDisagreementNm": float(np.max(np.abs(errors[:, da]))),
        "maxSoleConeViolation": cone_violation,
        "maxSwingWrenchNorm": swing,
        "stanceFootSpeedRmsMetersPerSecond": float(np.sqrt(np.mean(np.square(speeds)))),
        "scope": "21 evenly spaced inverse-dynamics samples; sole feasibility at every slice; passive forces removed from the MuJoCo comparison",
    }
    if np.max(np.abs(errors)) > 1e-6 or max(cone_violation, swing) > 1e-6:
        raise AssertionError(result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    result = validate(args.result)
    text = json.dumps(result, indent=2) + "\n"
    (args.result / "validation.json").write_text(text)
    print(text)
