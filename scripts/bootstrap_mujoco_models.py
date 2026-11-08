#!/usr/bin/env python3
"""Prepare the canonical G1 through the pinned Menagerie/MJLab Python APIs."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import mujoco

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "examples/g1_mjlab"))
from g1_model import prepare_model
from g1_urdf import write_ik_urdf


def main() -> int:
    source = prepare_model()
    write_ik_urdf(
        mujoco.MjModel.from_xml_path(str(source / "scene.xml")),
        REPO / "models/urdfs/g1/g1.urdf",
    )
    assets = REPO / "apps/contact-initialization/public/assets"
    assets.mkdir(parents=True, exist_ok=True)
    for link in (REPO / "models/mujoco/unitree_g1", assets / "unitree_g1"):
        if link.is_symlink():
            link.unlink()
        elif link.exists():
            raise RuntimeError(f"Expected a generated symlink: {link}")
        link.symlink_to(os.path.relpath(source, link.parent))
    provenance = json.loads((source / "provenance.json").read_text())
    files = [
        "unitree_g1/" + name
        for name in provenance["files"]
        if name.endswith((".xml", ".stl", ".STL", ".obj", ".png"))
    ]
    (assets / "manifest.json").write_text(
        json.dumps(
            {
                "source": "mujoco-menagerie",
                "revision": provenance["modelRevision"],
                "packages": provenance["packages"],
                "scene": "unitree_g1/scene.xml",
                "license": "unitree_g1/LICENSE",
                "files": files,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"G1 assets ready at {source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
