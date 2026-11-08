#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file problem.py
# @brief Load the G1 sparse factor dataset or generate a standard QP benchmark.
# @author GTDynamics contributors
"""Portable sparse data; no GTSAM installation is needed for Hessian conversion."""

import argparse
import json
from pathlib import Path

import numpy as np
import scipy.io
import scipy.sparse as sp


def load_factors(directory):
    """Return raw J, G, vectors and metadata, preserving original noise scales.

    Objective: 0.5 * ||diag(inverse_sigmas) * (J*x - r)||^2.
    Constraints: G*x <= h. Factor/key boundaries are in metadata.json.
    All NPZ files use NumPy/SciPy serialization without pickled objects.
    """
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    if metadata["format"] != "gtdynamics-sparse-qp-v1":
        raise ValueError("Unsupported dataset version")
    with np.load(directory / "vectors.npz", allow_pickle=False) as data:
        vectors = {key: data[key] for key in data.files}
    return (
        sp.load_npz(directory / "J.npz"),
        sp.load_npz(directory / "G.npz"),
        vectors,
        metadata,
    )


def quadratic_data(directory):
    """Return sparse P, q, G, h and the omitted least-squares constant.

    Computing J.T*J introduces the usual floating-point normal-equation rounding;
    GTSAM can instead reconstruct the original factors with load_factors().
    """
    jacobian, inequalities, vectors, _ = load_factors(directory)
    weighted = jacobian.multiply(vectors["inverse_sigmas"][:, None]).tocsc()
    rhs = vectors["r"] * vectors["inverse_sigmas"]
    hessian = (weighted.T @ weighted).tocsc()
    linear = np.asarray(-weighted.T @ rhs).ravel()
    return hessian, linear, inequalities.tocsc(), vectors["h"], float(rhs @ rhs / 2)


def make_problem(directory):
    """Construct a standard qpbenchmark.Problem, for its code submission path.

    Yield this object from a TestSet iterator; do not call Problem.save(), whose
    current NPZ implementation densifies the Hessian and loses sparse G.
    """
    from qpbenchmark import Problem

    hessian, linear, inequalities, bounds, _ = quadratic_data(directory)
    metadata = json.loads((Path(directory) / "metadata.json").read_text())
    return Problem(
        hessian,
        linear,
        inequalities,
        bounds,
        None,
        None,
        None,
        None,
        name=metadata["name"],
    )


def save_mat(directory, output):
    """Write the existing free_for_all benchmark's sparse MAT convention."""
    hessian, linear, inequalities, bounds, _ = quadratic_data(directory)
    dimension = linear.size
    count = bounds.size
    matrix = sp.vstack([inequalities, sp.eye(dimension)], format="csc")
    scipy.io.savemat(
        output,
        {
            "P": hessian,
            "q": linear[:, None],
            "A": matrix,
            "l": np.full((count + dimension, 1), -1e20),
            "u": np.concatenate([bounds, np.full(dimension, 1e20)])[:, None],
            "n": dimension,
            "m": count + dimension,
        },
        do_compression=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--mat", type=Path, required=True)
    arguments = parser.parse_args()
    save_mat(arguments.directory, arguments.mat)
