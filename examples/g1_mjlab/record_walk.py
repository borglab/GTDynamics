# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file record_walk.py
# @brief Record a pretrained MJLab G1 velocity policy without retraining.
# @author Frank Dellaert

"""Record a pretrained MJLab G1 velocity policy without retraining.

Run in the MJLab environment; checkpoint tensors are loaded with weights_only.
The default checkpoint was trained with a 10 ms policy period, hence decimation=2.
"""

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.velocity.config.g1.env_cfgs import unitree_g1_flat_env_cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--speed", type=float, default=0.5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    cfg = unitree_g1_flat_env_cfg(play=True)
    cfg.seed = 42
    cfg.scene.num_envs = 1
    cfg.decimation = 2
    cfg.auto_reset = False
    # Nominal model: disable startup domain randomization so CPU/GPU parameters
    # agree. Keep reset events, with a deterministic starting heading/position.
    cfg.events = {k: v for k, v in cfg.events.items() if v.mode == "reset"}
    cfg.events["reset_base"].params["pose_range"] = {}
    command = cfg.commands["twist"]
    command.ranges.lin_vel_x = (args.speed, args.speed)
    command.ranges.lin_vel_y = (0.0, 0.0)
    command.ranges.ang_vel_z = (0.0, 0.0)
    command.heading_command = False
    command.ranges.heading = None
    command.rel_standing_envs = 0.0
    command.rel_heading_envs = 0.0
    command.rel_forward_envs = 1.0
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
    state = torch.load(args.checkpoint, map_location="cuda:0", weights_only=True)[
        "actor_state_dict"
    ]

    def policy(obs):
        x = (obs - state["obs_normalizer._mean"]) / (
            state["obs_normalizer._std"] + 1e-2
        )
        for i in (0, 2, 4, 6):
            x = torch.nn.functional.linear(
                x, state[f"mlp.{i}.weight"], state[f"mlp.{i}.bias"]
            )
            if i != 6:
                x = torch.nn.functional.elu(x)
        return x

    obs, _ = env.reset()
    records = {name: [] for name in ("qpos", "qvel", "ctrl", "qacc", "qfrc_actuator")}
    with torch.inference_mode():
        for step in range(round(args.seconds / env.step_dt)):
            obs, _, terminated, truncated, _ = env.step(policy(obs["actor"]))
            if bool(torch.any(terminated | truncated)):
                raise RuntimeError(f"Policy terminated at {step * env.step_dt:.2f}s")
            for name, entries in records.items():
                entries.append(getattr(env.sim.data, name)[0].cpu().numpy().copy())
    mujoco.mj_saveModel(env.sim.mj_model, str(args.output / "model.mjb"), None)
    np.savez_compressed(
        args.output / "walk.npz", **{k: np.array(v) for k, v in records.items()}
    )
    qpos = np.array(records["qpos"])
    metadata = {
        "checkpoint_sha256": hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        "seconds": args.seconds,
        "policy_dt": env.step_dt,
        "command_speed_m_s": args.speed,
        "displacement_m": (qpos[-1, :3] - qpos[0, :3]).tolist(),
        "pelvis_height_range_m": [float(qpos[:, 2].min()), float(qpos[:, 2].max())],
        "mujoco_version": mujoco.__version__,
        "domain_randomization": False,
        "cone": "pyramidal",
    }
    (args.output / "walk.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))
    env.close()


if __name__ == "__main__":
    main()
