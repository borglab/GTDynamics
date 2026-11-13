"""
* GTDynamics Copyright 2026, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* See LICENSE for the license information
*
* @file  server.py
* @brief Local backend for the interactive URDF IK app: serves URDF/mesh
*        files and a JSON API backed by GTDynamics. The browser UI is the
*        Vite app in ../src, whose dev server proxies api/, files/ and pkg/
*        here (see ../vite.config.mjs).
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

import ik_backend as ik

DEFAULT_PORT = 8767  # keep in sync with ../vite.config.mjs

for ext, mime in (
    (".dae", "model/vnd.collada+xml"),
    (".stl", "model/stl"),
    (".obj", "model/obj"),
    (".urdf", "application/xml"),
    (".js", "text/javascript"),
):
    mimetypes.add_type(mime, ext)


class ModelStore:
    """Loads models on demand and caches them.

    Each URDF is test-parsed in a subprocess first, because some malformed
    models crash GTDynamics' native parser.
    """

    def __init__(self, urdf_root: Path):
        self.urdf_root = urdf_root
        self.lock = threading.Lock()  # serializes all GTDynamics calls
        self.models: dict[str, ik.RobotModel] = {}
        self.errors: dict[str, str] = {}

    def list(self) -> list[dict]:
        paths = sorted(
            p.relative_to(self.urdf_root).as_posix()
            for p in self.urdf_root.rglob("*.urdf")
        )
        return [
            {
                "path": rel,
                "status": (
                    "ok"
                    if rel in self.models
                    else "error" if rel in self.errors else "unknown"
                ),
            }
            for rel in paths
        ]

    def get(self, rel: str) -> ik.RobotModel:
        path = (self.urdf_root / rel).resolve()
        if (
            self.urdf_root not in path.parents
            or path.suffix != ".urdf"
            or not path.is_file()
        ):
            raise ik.ModelError(f"No URDF named '{rel}'.")
        with self.lock:
            if rel in self.models:
                return self.models[rel]
            if rel in self.errors:
                raise ik.ModelError(self.errors[rel])
        error = ik.check_loadable(path)
        with self.lock:
            try:
                if error is not None:
                    raise ik.ModelError(error)
                self.models[rel] = ik.RobotModel(self.urdf_root, rel)
            except ik.ModelError as e:
                self.errors[rel] = str(e)
                raise
            return self.models[rel]


class FileIndex:
    """Resolves ROS package:// mesh URIs to files under the models directory."""

    def __init__(self, root: Path):
        self.root = root
        self.files = sorted(
            p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()
        )

    def resolve(self, package: str, rel: str) -> Path | None:
        for suffix in (f"{package}/{rel}", rel):
            for f in self.files:
                if f == suffix or f.endswith("/" + suffix):
                    return self.root / f
        return None


def _finite(value):
    """JSON cannot encode inf/nan; send them as null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_finite(v) for v in value]
    return value


def make_handler(store: ModelStore, files: FileIndex):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            if not self.path.startswith(("/files/", "/pkg/")):
                super().log_message(fmt, *args)

        def send_json(self, data, status=200):
            body = json.dumps(_finite(data)).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_file(self, path: Path | None, root: Path):
            if (
                path is None
                or not path.resolve().is_file()
                or root.resolve() not in path.resolve().parents
            ):
                self.send_error(404)
                return
            body = path.read_bytes()
            mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = unquote(urlparse(self.path).path)
            if path.startswith("/files/"):
                self.send_file(files.root / path[len("/files/") :], files.root)
            elif path.startswith("/pkg/"):
                package, _, rel = path[len("/pkg/") :].partition("/")
                self.send_file(files.resolve(package, rel), files.root)
            elif path == "/api/models":
                self.send_json(
                    {
                        "models": store.list(),
                        # Relative to the page, so it works under any base path.
                        "urdf_url_prefix": "files/"
                        + store.urdf_root.relative_to(files.root).as_posix()
                        + "/",
                    }
                )
            elif path == "/api/schemes":
                self.send_json(
                    [
                        {
                            "id": key,
                            "label": s.label,
                            "description": s.description,
                            "default_iters": s.default_iters,
                            "requires_orientation": s.requires_orientation,
                        }
                        for key, s in ik.SCHEMES.items()
                    ]
                )
            else:
                self.send_error(404)

        def do_POST(self):
            path = urlparse(self.path).path
            length = int(self.headers.get("Content-Length", 0))
            body_bytes = self.rfile.read(length)
            try:
                body = json.loads(body_bytes or b"{}")
                if path == "/api/load":
                    self.send_json(self.load(body))
                elif path == "/api/fk":
                    self.send_json(self.fk(body))
                elif path == "/api/ik":
                    self.send_json(self.ik(body))
                else:
                    self.send_error(404)
            except ik.ModelError as e:
                self.send_json({"error": str(e)}, 422)
            except Exception as e:  # report, don't kill the server
                traceback.print_exc()
                self.send_json({"error": f"{type(e).__name__}: {e}"}, 500)

        def load(self, body):
            model = store.get(body["model"])
            with store.lock:
                info = model.describe()
                q = model.default_configuration()
                info.update(ik.forward(model, model.default_ee, q))
            return info

        def fk(self, body):
            model = store.get(body["model"])
            with store.lock:
                return ik.forward(model, body["ee"], body["q"])

        def ik(self, body):
            model = store.get(body["model"])
            req = ik.IkRequest(
                ee=body["ee"],
                target=ik.pose_from_json(body["target"]),
                q0=body["q"],
                orientation=bool(body.get("orientation", True)),
                pos_tol=float(body.get("pos_tol", 1e-3)),
                rot_tol=math.radians(float(body.get("rot_tol_deg", 0.5))),
                max_iters=int(body["max_iters"]) if body.get("max_iters") else None,
            )
            with store.lock:
                return ik.solve(model, body["scheme"], req)

    return Handler


def main():
    parser = argparse.ArgumentParser(
        description="Backend for the interactive URDF inverse kinematics app "
        "(GTDynamics)."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--urdf-root",
        type=Path,
        default=ik.DEFAULT_URDF_ROOT,
        help="directory searched for *.urdf files " "(default: %(default)s)",
    )
    args = parser.parse_args()

    urdf_root = args.urdf_root.resolve()
    store = ModelStore(urdf_root)
    # Serve the parent directory too, so "../meshes/..." references resolve.
    files = FileIndex(urdf_root.parent)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(store, files))
    print(
        f"URDF IK backend serving {urdf_root} at "
        f"http://{args.host}:{args.port}/  (Ctrl+C to stop)",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
