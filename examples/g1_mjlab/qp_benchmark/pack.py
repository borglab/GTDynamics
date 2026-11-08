#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file pack.py
# @brief Convert the pinned native capture to portable sparse benchmark data.
# @author GTDynamics contributors
"""Pack and validate a capture; SciPy supplies sparse file interchange."""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import scipy.sparse as sp

from problem import load_factors, quadratic_data


def read_factors(path, keys):
    """Read the local capture stream without assembling a dense global matrix."""
    offsets = {int(entry["key"]): entry["offset"] for entry in keys}
    dimension = sum(entry["dim"] for entry in keys)
    values, rows, columns, rhs, sigmas, inverse_sigmas, factors = (
        [],
        [],
        [],
        [],
        [],
        [],
        [],
    )
    row_offset = 0
    with path.open("rb") as stream:
        count = int(np.fromfile(stream, dtype=np.uint64, count=1)[0])
        for _ in range(count):
            key_count, row_count, col_count = map(
                int, np.fromfile(stream, dtype=np.uint64, count=3)
            )
            blocks = np.fromfile(stream, dtype=np.uint64, count=key_count * 2).reshape(
                -1, 2
            )
            matrix = np.fromfile(stream, dtype=np.float64, count=row_count * col_count)
            local = sp.coo_matrix(matrix.reshape((row_count, col_count), order="F"))
            column_map = np.concatenate(
                [
                    np.arange(
                        offsets[int(key)], offsets[int(key)] + int(dim), dtype=np.int32
                    )
                    for key, dim in blocks
                ]
            )
            assert len(column_map) == col_count
            values.append(local.data)
            rows.append(local.row + row_offset)
            columns.append(column_map[local.col])
            rhs.append(np.fromfile(stream, dtype=np.float64, count=row_count))
            sigmas.append(np.fromfile(stream, dtype=np.float64, count=row_count))
            inverse_sigmas.append(
                np.fromfile(stream, dtype=np.float64, count=row_count)
            )
            factors.append(
                {
                    "row_start": row_offset,
                    "row_count": row_count,
                    "keys": [str(int(key)) for key, _ in blocks],
                }
            )
            row_offset += row_count
        assert not stream.read(1)
    if not count:
        return sp.csr_matrix((0, dimension)), np.empty(0), np.empty(0), np.empty(0), []
    matrix = sp.coo_matrix(
        (np.concatenate(values), (np.concatenate(rows), np.concatenate(columns))),
        shape=(row_offset, dimension),
    ).tocsr()
    return (
        matrix,
        np.concatenate(rhs),
        np.concatenate(sigmas),
        np.concatenate(inverse_sigmas),
        factors,
    )


def pack(directory):
    """Save exact captured coefficients and validate native costs and gradients."""
    raw = directory / "raw"
    with (raw / "keys.csv").open() as stream:
        keys = [
            dict(key=row["key"], offset=int(row["offset"]), dim=int(row["dim"]))
            for row in csv.DictReader(stream)
        ]
    jacobian, rhs, sigmas, inverse_sigmas, costs = read_factors(raw / "costs.bin", keys)
    inequalities, bounds, constraint_sigmas, constraint_inverse_sigmas, constraints = (
        read_factors(raw / "inequalities.bin", keys)
    )
    dimension = jacobian.shape[1]
    native = json.loads((raw / "native_checks.json").read_text())
    provenance = json.loads((directory / "provenance.json").read_text())
    log = (directory / "capture.log").read_text()
    first_step = log.find("Kinodynamics iteration 1:")
    second_qp = log.find("GLOBAL_QP index=2 ")
    complete = log.find("CAPTURE_COMPLETE")
    if not 0 <= first_step < second_qp < complete:
        raise ValueError("Capture must follow an accepted nonlinear step")
    assert dimension == native["dimension"]
    assert np.all(sigmas > 0) and np.isfinite(sigmas).all()
    assert np.array_equal(1.0 / sigmas, inverse_sigmas)
    for array in (jacobian.data, inequalities.data, rhs, bounds, inverse_sigmas):
        assert np.isfinite(array).all()
    assert np.count_nonzero(bounds) == 0
    assert np.all(constraint_sigmas == 1)
    assert np.all(constraint_inverse_sigmas == 1)
    # The constructor appends one LM damping factor per nonempty key.
    damping_start = len(costs) - len(keys)
    dimensions = {entry["key"]: entry["dim"] for entry in keys}
    assert {factor["keys"][0] for factor in costs[damping_start:]} == set(dimensions)
    for factor in costs[damping_start:]:
        assert len(factor["keys"]) == 1
        assert factor["row_count"] == dimensions[factor["keys"][0]]
    damping_row_start = costs[damping_start]["row_start"]
    damping_sigmas = sigmas[damping_row_start:]
    assert np.all(damping_sigmas == damping_sigmas[0])
    with (raw / "rows.csv").open() as stream:
        row_origins = list(csv.DictReader(stream))
    metadata = {
        "format": "gtdynamics-sparse-qp-v1",
        "name": provenance.get("dataset_name", "G1CMCTWO"),
        "global_qp_call": 2,
        "accepted_nonlinear_steps_before_capture": 1,
        "clip_seconds": provenance["clip_seconds"],
        "samples": provenance["samples"],
        "sampling": provenance.get("sampling"),
        "dimension": dimension,
        "inequality_rows": inequalities.shape[0],
        "equality_rows": 0,
        "variable_bounds": None,
        "jacobian_rows": jacobian.shape[0],
        "jacobian_nnz": jacobian.nnz,
        "inequality_nnz": inequalities.nnz,
        "cost_definition": "0.5 * ||diag(inverse_sigmas) * (J*x-r)||^2",
        "constraint_definition": "G*x <= h",
        "keys": keys,
        "cost_factors": costs,
        "inequality_factors": constraints,
        "inequality_row_origins": row_origins,
        "damping_factor_start": damping_start,
        "lm_lambda": float(1.0 / damping_sigmas[0] ** 2),
        "additional_qp_regularization": native["regularization"],
        "native_checks": native,
    }
    sp.save_npz(directory / "J.npz", jacobian)
    sp.save_npz(directory / "G.npz", inequalities)
    np.savez_compressed(
        directory / "vectors.npz",
        r=rhs,
        sigmas=sigmas,
        inverse_sigmas=inverse_sigmas,
        h=bounds,
        x0=np.zeros(dimension),
    )
    (directory / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    # Confirm portable storage is lossless before checking mathematical equivalence.
    loaded_j, loaded_g, vectors, _ = load_factors(directory)
    assert (loaded_j != jacobian).nnz == 0 and (loaded_g != inequalities).nnz == 0
    for name, original in (
        ("r", rhs),
        ("sigmas", sigmas),
        ("inverse_sigmas", inverse_sigmas),
        ("h", bounds),
    ):
        assert np.array_equal(vectors[name], original)
    weighted = jacobian.multiply(inverse_sigmas[:, None]).tocsr()
    weighted_rhs = rhs * inverse_sigmas
    hessian, linear, _, _, constant = quadratic_data(directory)
    assert (hessian - hessian.T).nnz == 0
    gradient_references = np.fromfile(
        raw / "native_gradients.bin", dtype=np.float64
    ).reshape(2, dimension)
    np.savez_compressed(
        directory / "native_gradients.npz",
        zero=gradient_references[0],
        probe=gradient_references[1],
    )
    checks = {
        "lossless_sparse_roundtrip": True,
        "zero_feasible": True,
        "P_nnz": hessian.nnz,
        "objective_constant": constant,
    }
    for index, name in enumerate(("zero", "probe")):
        point = (
            np.zeros(dimension)
            if index == 0
            else 1e-4 * (np.arange(dimension) % 17 - 8)
        )
        residual = weighted @ point - weighted_rhs
        cost = float(residual @ residual / 2)
        native_cost = native[f"{name}_cost"]
        np.testing.assert_allclose(cost, native_cost, rtol=1e-12, atol=1e-8)
        quadratic_cost = float(
            point @ (hessian @ point) / 2 + linear @ point + constant
        )
        np.testing.assert_allclose(quadratic_cost, native_cost, rtol=1e-12, atol=1e-8)
        gradient = weighted.T @ residual
        reference = gradient_references[index]
        gradient_error = np.linalg.norm(gradient - reference, ord=np.inf)
        assert gradient_error <= 1e-12 * max(1, np.linalg.norm(reference, ord=np.inf))
        checks[name] = {
            "native_cost": native_cost,
            "loaded_cost": cost,
            "quadratic_cost": quadratic_cost,
            "gradient_error_inf": float(gradient_error),
        }
    probe = 1e-4 * (np.arange(dimension) % 17 - 8)
    expected = np.array([float(row["probe_residual"]) for row in row_origins])
    np.testing.assert_allclose(
        inequalities @ probe - bounds, expected, rtol=1e-12, atol=1e-12
    )
    checks["native_constraint_probe_matches"] = True
    (directory / "validation.json").write_text(json.dumps(checks, indent=2) + "\n")
    print(json.dumps(checks, indent=2))


if __name__ == "__main__":
    if sys.byteorder != "little":
        raise RuntimeError("This native capture was produced on little-endian x86-64")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    pack(parser.parse_args().directory)
