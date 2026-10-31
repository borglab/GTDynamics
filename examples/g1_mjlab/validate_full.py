# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file validate_full.py
# @brief Independent import, kinematics, wrench, armature and objective checks.
# @author Frank Dellaert

"""Independent import, kinematics, wrench, armature and objective checks.

MuJoCo matrices/Jacobians below are validation oracles only. The executable
receives the restricted full_model snapshot and prescribed test forces.
"""

import argparse
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from full_model import snapshot
from mujoco_io import load_model, load_recording


def run(solver, request):
    process = subprocess.run(
        [str(solver.resolve())],
        input=json.dumps(request) + "\n",
        text=True,
        capture_output=True,
        timeout=60,
        check=True,
    )
    result = json.loads(process.stdout)
    if "error" in result:
        raise RuntimeError(result["error"] + "\n" + process.stderr[-2000:])
    return result


def validate(solver, directory):
    model = load_model(directory)
    walk, _ = load_recording(directory)
    maxima = dict.fromkeys(
        (
            "pose",
            "twist",
            "jacobian",
            "acceleration_bias",
            "dynamics",
            "objective",
            "equality",
        ),
        0.0,
    )
    cases = []
    rng = np.random.default_rng(27)
    formulations = {
        "pyramidal": "pyramidal", "elliptic_norm": "elliptic",
        "signed_square": "elliptic", "friction_factor": "elliptic",
    }
    for formulation, cone in formulations.items():
        model.opt.cone = getattr(mujoco.mjtCone, "mjCONE_" + cone.upper())
        for index in (200, 421, len(walk["qpos"]) - 1):
            data = mujoco.MjData(model)
            for name in ("qpos", "qvel", "ctrl"):
                getattr(data, name)[:] = walk[name][index]
            mujoco.mj_forward(model, data)
            request = {**snapshot(model, data, cone), "friction_model": formulation}
            M = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, data, M)
            J = data.efc_J.reshape(data.nefc, model.nv)
            for mode in ("zero", "random", "reference"):
                force = {
                    "zero": np.zeros(data.nefc),
                    "random": rng.normal(size=data.nefc) * 10,
                    "reference": data.efc_force,
                }[mode]
                result = run(solver, {**request, "prescribed_force": force.tolist()})
                acceleration = np.array(result["acceleration"])
                expected = np.linalg.solve(
                    M, np.array(request["applied"]) - data.qfrc_bias + J.T @ force
                )
                error = float(np.max(np.abs(acceleration - expected)))
                maxima["dynamics"] = max(maxima["dynamics"], error)
                maxima["equality"] = max(maxima["equality"], result["equality_l2"])
                expected_cost = (
                    0.5 * acceleration @ M @ acceleration
                    + 0.5 * (force * data.efc_R) @ force
                    - data.efc_aref @ force
                )
                maxima["objective"] = max(
                    maxima["objective"], abs(expected_cost - result["objective"])
                )
                if formulation == "friction_factor":
                    expected_penalty = 0.0
                    for contact in request["contacts"]:
                        j = contact["address"]
                        mu = contact["friction"][0]
                        residual = max(0.0, force[j + 1:j + 3] @ force[j + 1:j + 3] - mu * mu * force[j] ** 2)
                        expected_penalty += 0.5 * (residual / 0.001) ** 2
                    assert np.isclose(result["penalty_cost"], expected_penalty, rtol=1e-10, atol=1e-7)
                graph = result["graph"]
                assert (
                    graph["ambient_dimension"] - graph["equality_dimension"]
                    == data.nefc
                )
                assert graph["equality_dimension"] > 500
                for body in graph["bodies"]:
                    b = body["id"]
                    jp, jr, dp, dr = [np.zeros((3, model.nv)) for _ in range(4)]
                    mujoco.mj_jacBodyCom(model, data, jp, jr, b)
                    mujoco.mj_jacDot(model, data, dp, dr, data.xipos[b], b)
                    rotation = data.ximat[b].reshape(3, 3)
                    transform = np.eye(4)
                    transform[:3, :3], transform[:3, 3] = rotation, data.xipos[b]
                    jacobian = np.vstack((rotation.T @ jr, rotation.T @ jp))
                    twist = jacobian @ data.qvel
                    bias = np.r_[
                        rotation.T @ dr @ data.qvel,
                        rotation.T @ dp @ data.qvel - np.cross(twist[:3], twist[3:]),
                    ]
                    for name, expected_value in (
                        ("pose", transform),
                        ("twist", twist),
                        ("jacobian", jacobian),
                        ("acceleration_bias", bias),
                    ):
                        actual = body[
                            "acceleration_map" if name == "jacobian" else name
                        ]
                        maxima[name] = max(
                            maxima[name],
                            float(np.max(np.abs(np.array(actual) - expected_value))),
                        )
                cases.append(
                    {
                        "cone": cone,
                        "formulation": formulation,
                        "index": index,
                        "force": mode,
                        "acceleration_max_abs": error,
                    }
                )
    for name, error in maxima.items():
        assert error < (1e-6 if name == "objective" else 1e-8), (name, error)
    return {"cases": cases, "maximum_errors": maxima, "passed": True}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solver", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(args.solver, args.data)
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text)
