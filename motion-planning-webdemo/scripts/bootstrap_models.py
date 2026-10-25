#!/usr/bin/env python3
"""Download the pinned Panda model used by the browser, without vendoring meshes."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

MENAGERIE_URL = "https://github.com/google-deepmind/mujoco_menagerie.git"
MENAGERIE_COMMIT = "a03e87bf13502b0b48ebbf2808928fd96ebf9cf3"
MODEL = "franka_emika_panda"
ASSET_EXTENSIONS = {".xml", ".obj", ".stl", ".png", ".jpg", ".jpeg", ".mtl", ".skn"}


def run(*args: str, cwd: Path | None = None) -> None:
    subprocess.run(args, cwd=cwd, check=True)


def main() -> None:
    app_root = Path(__file__).resolve().parents[1]
    checkout = app_root / ".cache" / "mujoco_menagerie"
    if not checkout.exists():
        checkout.parent.mkdir(parents=True, exist_ok=True)
        run("git", "clone", "--filter=blob:none", "--sparse", "--no-checkout",
            MENAGERIE_URL, str(checkout))
    current = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=checkout, text=True,
        check=True, capture_output=True).stdout.strip()
    if current != MENAGERIE_COMMIT or not (checkout / MODEL / "panda.xml").exists():
        run("git", "fetch", "--depth", "1", "origin", MENAGERIE_COMMIT, cwd=checkout)
        run("git", "sparse-checkout", "set", MODEL, cwd=checkout)
        run("git", "checkout", "--detach", MENAGERIE_COMMIT, cwd=checkout)
    assets = app_root / "public" / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    source = checkout / MODEL
    target = assets / MODEL
    if target.is_symlink():
        if target.resolve() != source.resolve():
            target.unlink()
    elif target.exists():
        raise RuntimeError(f"Refusing to replace existing directory: {target}")
    if not target.is_symlink():
        target.symlink_to(os.path.relpath(source, assets), target_is_directory=True)
    files = sorted(f"{MODEL}/{p.relative_to(source).as_posix()}"
                   for p in source.rglob("*")
                   if p.is_file() and p.suffix.lower() in ASSET_EXTENSIONS)
    manifest = {
        "source": MENAGERIE_URL,
        "revision": MENAGERIE_COMMIT,
        "scene": f"{MODEL}/scene.xml",
        "files": files,
    }
    (assets / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Panda ready: {len(files)} assets at Menagerie {MENAGERIE_COMMIT[:12]}")
    print(f"Asset license: {source / 'LICENSE'}")


if __name__ == "__main__":
    main()
