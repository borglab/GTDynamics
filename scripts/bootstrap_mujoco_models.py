#!/usr/bin/env python3
"""Bootstrap MuJoCo Menagerie models for GTDynamics webdemo.

What it does:
1. Sparse-clones/pulls google-deepmind/mujoco_menagerie at a pinned commit.
2. Materializes only required model folders:
   - unitree_g1
   - unitree_a1
   - boston_dynamics_spot
3. Creates symlinks:
   - models/mujoco/<model>
   - webdemo/public/examples/scenes/<model>
4. Regenerates webdemo/public/examples/scenes/files.json from on-disk assets.

This keeps large binary model files out of this git repository.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

MENAGERIE_URL = "https://github.com/google-deepmind/mujoco_menagerie.git"
MENAGERIE_COMMIT = "a03e87bf13502b0b48ebbf2808928fd96ebf9cf3"
MODELS = ["unitree_g1", "unitree_a1", "boston_dynamics_spot"]
ASSET_EXTS = {".xml", ".obj", ".stl", ".png", ".jpg", ".jpeg", ".mtl", ".skn"}


def run(cmd: list[str], cwd: Path | None = None) -> None:
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True)


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
    rel_target = os.path.relpath(target_path, link_path.parent)
    link_path.symlink_to(rel_target)


def generate_files_json(scenes_dir: Path) -> list[str]:
    files: list[str] = []
    for model in MODELS:
        model_dir = scenes_dir / model
        if not model_dir.exists():
            continue
        for path in sorted(model_dir.rglob("*")):
            if not path.is_file():
                continue
            if path.suffix.lower() not in ASSET_EXTS:
                continue
            files.append(path.relative_to(scenes_dir).as_posix())
    return files


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    cache_root = Path.home() / ".cache" / "gtdynamics" / "mujoco_menagerie"
    checkout_root = cache_root / MENAGERIE_COMMIT
    menagerie_repo = checkout_root / "repo"

    checkout_root.mkdir(parents=True, exist_ok=True)

    if not menagerie_repo.exists():
        run([
            "git", "clone", "--filter=blob:none", "--sparse", MENAGERIE_URL, str(menagerie_repo)
        ])

    run(["git", "fetch", "--depth", "1", "origin", MENAGERIE_COMMIT], cwd=menagerie_repo)
    run(["git", "checkout", "--detach", MENAGERIE_COMMIT], cwd=menagerie_repo)
    run(["git", "sparse-checkout", "set", *MODELS], cwd=menagerie_repo)

    models_dir = repo_root / "models" / "mujoco"
    scenes_dir = repo_root / "webdemo" / "public" / "examples" / "scenes"

    for model in MODELS:
        source = menagerie_repo / model
        ensure_symlink(models_dir / model, source)
        ensure_symlink(scenes_dir / model, models_dir / model)

    files = generate_files_json(scenes_dir)
    files_json_path = scenes_dir / "files.json"
    files_json_path.write_text(json.dumps(files, indent=2) + "\n", encoding="utf-8")

    print("Bootstrap complete.")
    print(f"Menagerie commit: {MENAGERIE_COMMIT}")
    print(f"Model symlinks under: {models_dir}")
    print(f"Scene symlinks under: {scenes_dir}")
    print(f"Regenerated: {files_json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
