# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file download_policy.py
# @brief Download the pinned public MJLab G1 PPO policy and verify its checksum.
# @author Frank Dellaert

"""Download the pinned public MJLab G1 PPO policy and verify its checksum."""

import argparse
import hashlib
from pathlib import Path

import requests

BASE = "https://huggingface.co/robomotic/mjlab-policies/resolve/f277ca8/g1_velocity/2026-04-26_20-11-37/"
CHECKSUM = "b45a629ce3b6c279201be60362408cb0586fac077e577964bd66a50c32fd4763"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    args.destination.mkdir(parents=True, exist_ok=True)
    for name in ("model_final.pt", "env.yaml", "agent.yaml", "mjlab.diff", "README.md"):
        response = requests.get(BASE + name, timeout=120)
        response.raise_for_status()
        if (
            name == "model_final.pt"
            and hashlib.sha256(response.content).hexdigest() != CHECKSUM
        ):
            raise RuntimeError("Checkpoint checksum mismatch")
        (args.destination / name).write_bytes(response.content)
    print(args.destination / "model_final.pt")


if __name__ == "__main__":
    main()
