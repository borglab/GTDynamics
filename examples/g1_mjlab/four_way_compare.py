# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file four_way_compare.py
# @brief Compare four friction formulations in full-graph CMC dynamics.
# @author Frank Dellaert

"""Run all four CMC friction formulations with fresh CPU and MJLab timings."""

import argparse
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

from full_model import snapshot
from timing import native_forward, warp_forward

VARIANTS = {
    "pyramidal": "pyramidal",
    "elliptic_norm": "elliptic",
    "signed_square": "elliptic",
    "friction_factor": "elliptic",
}
THRESHOLDS = {
    "acceleration_max_abs": 1e-3,
    "equality_l2": 1e-5,
    "canonical_cone_violation_l2": 1e-4,
    "projected_gradient_inf": 1e-3,
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def canonical_violation(force, cone, contacts):
    if cone == "pyramidal":
        return float(np.linalg.norm(np.maximum(-force, 0)))
    violations = []
    for contact in contacts:
        j = contact["address"]
        n, tangent = force[j], force[j + 1 : j + 3]
        violations.extend((max(-n, 0), max(np.linalg.norm(tangent / contact["friction"]) - n, 0)))
    return float(np.linalg.norm(violations))


def projected_gradient(force, gradient, cone, contacts):
    """Validation only: project in original force coordinates after solving."""
    trial = force - gradient
    if cone == "pyramidal":
        return float(np.max(np.abs(force - np.maximum(trial, 0)), initial=0))
    projected = trial.copy()
    for contact in contacts:
        j = contact["address"]
        mu, mu2 = contact["friction"]
        if not np.isclose(mu, mu2):
            raise ValueError(
                "Independent projected-gradient check requires isotropic friction"
            )
        n, t = trial[j], trial[j + 1 : j + 3]
        length = np.linalg.norm(t)
        if n >= 0 and length <= mu * n:
            continue
        normal = max(0, (n + mu * length) / (1 + mu * mu))
        projected[j] = normal
        projected[j + 1 : j + 3] = t * (mu * normal / length) if length else 0
    return float(np.max(np.abs(force - projected), initial=0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--solver", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--gtsam-source", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=24)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--timing-repeats", type=int, default=30)
    parser.add_argument("--timing-warmup", type=int, default=5)
    parser.add_argument("--friction-sigma", type=float, default=0.001)
    parser.add_argument("--skip-warp", action="store_true")
    args = parser.parse_args()
    if min(args.samples, args.repeats, args.timing_repeats, args.timing_warmup) < 1 or args.timeout <= 0 or args.friction_sigma <= 0:
        parser.error("Counts, timeout, and friction sigma must be positive")
    output = args.output or args.data
    output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parent
    root = source.parents[1]
    walk = np.load(args.data / "walk.npz")
    metadata = json.loads((args.data / "walk.json").read_text())
    indices = np.linspace(round(2 / metadata["policy_dt"]), len(walk["qpos"]) - 1, args.samples, dtype=int).tolist()
    if len(set(indices)) != len(indices):
        parser.error("Requested samples exceed distinct available states")
    linked = subprocess.check_output(["ldd", str(args.solver.resolve())], text=True)
    libraries = {}
    for name in ("libgtsam.so.4", "libgtdynamics.so"):
        line = next(line for line in linked.splitlines() if line.strip().startswith(name + " =>"))
        path = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
        libraries[name] = {"path": str(path), "sha256": digest(path)}
    config = {
        "schema": "gtdynamics-four-way-v1",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "variants": VARIANTS, "indices": indices, "repeats": args.repeats,
        "solver": "CMC", "timeout_seconds": args.timeout,
        "friction_factor_sigma": args.friction_sigma, "thresholds": THRESHOLDS,
        "native_iterations": 100, "native_tolerance": 1e-12,
        "timing_repeats": args.timing_repeats, "timing_warmup": args.timing_warmup,
        "warp_evaluated": not args.skip_warp,
        "reference_timings": "fresh in this run; none reused",
        "solver_build": json.loads(subprocess.check_output([str(args.solver.resolve()), "--build-info"], text=True)),
        "solver_sha256": digest(args.solver), "libraries": libraries,
        "gtdynamics_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "gtdynamics_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root)),
        "gtsam_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=args.gtsam_source, text=True).strip(),
        "gtsam_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=args.gtsam_source)),
        "source_sha256": {str(p.relative_to(root)): digest(p) for p in [
            source / name for name in ("full_dynamics.cpp", "full_model.py", "four_way_compare.py", "timing.py")
        ] + [p for directory in ("cmopt", "cmcopt", "constraints") for p in sorted((root / "gtdynamics" / directory).glob("*")) if p.suffix in (".cpp", ".h")] + [root / "gtdynamics/factors/ContactDynamicsFrictionConeFactor.h"]},
        "inputs": {name: digest(args.data / name) for name in ("model.mjb", "walk.npz", "walk.json")},
        "versions": {name: importlib.metadata.version(name) for name in ("mujoco", "mujoco-warp", "warp-lang")},
        "platform": platform.platform(),
        "gt_timing_scope": "GTDynamics model/FK, graph assembly, feasible initialization, equality component/basis, and CMC optimization; excludes JSON/process startup and shared collision generation",
        "native_timing_scope": "complete mj_forward, warm kernels, zero qacc_warmstart each call",
        "warp_timing_scope": "synchronized CUDA graph mjw.forward, zero qacc_warmstart; excludes state transfer, allocation, and compilation",
    }
    save(output / "four-way-config.json", config)
    references, cases = {}, {}
    for cone in ("pyramidal", "elliptic"):
        model = mujoco.MjModel.from_binary_path(str(args.data / "model.mjb"))
        model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
        model.opt.cone = getattr(mujoco.mjtCone, "mjCONE_" + cone.upper())
        model.opt.iterations, model.opt.tolerance = 100, 1e-12
        warp = None if args.skip_warp else warp_forward(model, walk, indices, args.timing_warmup, args.timing_repeats)
        if warp is not None:
            save(output / f"four-way-warp-{cone}.json", warp)
        references[cone] = []
        for ordinal, index in enumerate(indices):
            data = mujoco.MjData(model)
            for name in ("qpos", "qvel", "ctrl"):
                getattr(data, name)[:] = walk[name][index]
            native = native_forward(model, data, args.timing_warmup, args.timing_repeats)
            request = snapshot(model, data, cone)
            # Oracle matrices only: none are included in the solver request.
            M = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, data, M)
            J = data.efc_J.reshape(data.nefc, model.nv)
            H = J @ np.linalg.solve(M, J.T) + np.diag(data.efc_R)
            linear = J @ np.linalg.solve(M, np.array(request["applied"]) - data.qfrc_bias) - data.efc_aref
            cases[cone, index] = (request, data.qacc.copy(), data.efc_force.copy(), H, linear)
            reference = {"index": index, "native": native, "contacts": len(request["contacts"])}
            if warp is not None:
                reference["warp"] = warp["single"][ordinal]
                reference["warp_acceleration_max_abs"] = float(np.max(np.abs(np.array(warp["single"][ordinal]["acceleration"]) - data.qacc)))
            references[cone].append(reference)
        print("Fresh reference timings complete:", cone, flush=True)
    save(output / "four-way-references.json", references)

    with (output / "four-way-solver.log").open("w") as log:
        def run(variant, index):
            cone = VARIANTS[variant]
            request, reference_accel, reference_force, H, linear = cases[cone, index]
            payload = {**request, "solver": "CMC", "friction_model": variant, "friction_sigma": args.friction_sigma}
            started = time.perf_counter()
            try:
                proc = subprocess.run([str(args.solver.resolve())], input=json.dumps(payload) + "\n", text=True,
                                      stdout=subprocess.PIPE, stderr=log, timeout=args.timeout, check=True)
                result = json.loads(proc.stdout)
                if "error" not in result:
                    result["graph"].pop("bodies", None)
                    force, accel = np.array(result["force"]), np.array(result["acceleration"])
                    result["canonical_cone_violation_l2"] = canonical_violation(force, cone, request["contacts"])
                    result["acceleration_max_abs"] = float(np.max(np.abs(accel - reference_accel)))
                    result["force_max_abs"] = float(np.max(np.abs(force - reference_force), initial=0))
                    result["projected_gradient_inf"] = projected_gradient(force, H @ force + linear, cone, request["contacts"])
                    result["validated"] = bool(all(np.isfinite(result[k]) and result[k] < v for k, v in THRESHOLDS.items()))
                else:
                    result["validated"] = False
            except subprocess.TimeoutExpired:
                result = {"error": f"timeout after {args.timeout:g} s", "validated": False}
            except (subprocess.CalledProcessError, json.JSONDecodeError) as error:
                result = {"error": str(error), "validated": False}
            result["process_wall_seconds"] = time.perf_counter() - started
            return result

        for variant in VARIANTS:
            result = run(variant, indices[0])
            print("Warmup:", variant, result.get("error", "returned"), flush=True)
        results, names = [], list(VARIANTS)
        for ordinal, index in enumerate(indices):
            row = {"index": index, "runs": {v: [] for v in names}, "request_sha256": {
                c: hashlib.sha256(json.dumps(cases[c, index][0], sort_keys=True).encode()).hexdigest()
                for c in ("pyramidal", "elliptic")}}
            for repeat in range(args.repeats):
                shift = (ordinal + repeat) % len(names)
                for variant in names[shift:] + names[:shift]:
                    row["runs"][variant].append(run(variant, index))
            results.append(row)
            save(output / "four-way-results.json", results)
            print(index, {v: {"valid": sum(r["validated"] for r in rr),
                             "errors": sum("error" in r for r in rr)} for v, rr in row["runs"].items()}, flush=True)
    config["completed_utc"] = datetime.now(timezone.utc).isoformat()
    save(output / "four-way-config.json", config)


if __name__ == "__main__":
    main()
