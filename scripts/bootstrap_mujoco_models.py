#!/usr/bin/env python3
"""Fetch the pinned Menagerie G1 and share it with Contact Initialization.

Model XML/meshes live in models/mujoco/.cache, outside Git. The app and the
retargeted-motion importer refer to the same checkout through local symlinks.
The website build materializes those links and includes the model license.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

MENAGERIE_URL = "https://github.com/google-deepmind/mujoco_menagerie.git"
MENAGERIE_COMMIT = "a03e87bf13502b0b48ebbf2808928fd96ebf9cf3"


def run(cmd: list[str], cwd: Path | None = None) -> None:
    subprocess.run(cmd, cwd=cwd, check=True)


def ensure_symlink(link_path: Path, target_path: Path) -> None:
    if link_path.is_symlink():
        if link_path.resolve() == target_path.resolve():
            return
        link_path.unlink()
    elif link_path.exists():
        if link_path.is_dir():
            shutil.rmtree(link_path)
        else:
            link_path.unlink()
    link_path.parent.mkdir(parents=True, exist_ok=True)
    link_path.symlink_to(os.path.relpath(target_path, link_path.parent))


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    models = repo_root / "models/mujoco"
    checkout = models / ".cache/mujoco_menagerie" / MENAGERIE_COMMIT / "repo"
    checkout.parent.mkdir(parents=True, exist_ok=True)
    if not checkout.exists():
        run(
            [
                "git",
                "clone",
                "--filter=blob:none",
                "--sparse",
                MENAGERIE_URL,
                str(checkout),
            ]
        )
    run(["git", "fetch", "--depth", "1", "origin", MENAGERIE_COMMIT], cwd=checkout)
    run(["git", "checkout", "--detach", MENAGERIE_COMMIT], cwd=checkout)
    run(["git", "sparse-checkout", "set", "unitree_g1"], cwd=checkout)
    source = checkout / "unitree_g1"
    ensure_symlink(models / "unitree_g1", source)
    assets = repo_root / "apps/contact-initialization/public/assets"
    ensure_symlink(assets / "unitree_g1", source)
    description = ET.parse(source / "g1.xml")
    mesh_files = {
        f"unitree_g1/assets/{mesh.attrib['file']}"
        for mesh in description.findall("./asset/mesh")
    }
    manifest = {
        "source": MENAGERIE_URL,
        "revision": MENAGERIE_COMMIT,
        "scene": "unitree_g1/g1.xml",
        "license": "unitree_g1/LICENSE",
        "files": ["unitree_g1/g1.xml", *sorted(mesh_files)],
    }
    (assets / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"G1 assets ready at {source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
