# Kinodynamics failure investigation

The shared-linearization commit `f71c907d` reproduces PR #469 head's successful
result exactly when both run in the same environment. The old benchmark failure
is triggered by a change in the NumPy environment and also occurs at #469's head.

## Controlled results

| Revision | NumPy | Outcome | Optimization time |
| --- | --- | --- | --- |
| `45172046` (#469 head) | 2.2.6 | Five iterations, feasible | 81.61 s |
| `f71c907d` (shared linearizations) | 2.2.6 | Five iterations, feasible | 81.16 s |
| `45172046` | 2.5.2 | Same cone-QP AL exception | No completed solve |
| `f71c907d` | 2.5.2 | Same cone-QP AL exception | No completed solve |
| `f71c907d`, successful interpreter with only NumPy replaced | 2.5.2 | Same cone-QP AL exception | No completed solve |

These are single reproduction runs on CPU 2 with eight retraction workers sharing
that CPU. They do not replace the earlier repeated benchmark or measure normal
multicore app latency. The failing solves take about 52.3–52.6 seconds from the
Python `solve_walk` call to the exception, including preparation and restoration.
The original report instead records complete process wall time for failures.

Both successful runs have objective **9120.017395552624**, equality L2
**4.137014483932463e-12**, and inequality L2 **1.28621769347088e-7**. All saved
numerical arrays are bit-identical across these revisions. Five accepted
iterations establish a completed bounded run, not convergence to a minimum.

## What changes with the environment

The unsuccessful environment changes the initial Values before the coupled
native optimization. The largest local-coordinate difference is **3.67e-9**
across mixed variable units, with L2 **3.287e-7**. The successful initial Values
hash is `ca76c7cce24de36d3940ac1c3341043e750d72a10f8d4ac8b2528bcd0b949e68`;
the unsuccessful hash is
`a4b978f09bccb083fd4a5e9fbf57cf9db028e4ea0e44a0793e1c8f0a7d621247`.

Replacing only the imported NumPy package and its backend within the successful
Python interpreter reproduces both that second hash and the exact exception.
GTSAM, GTDynamics, the native example module, source inputs, and solver settings
remain fixed in that check. The precise NumPy operation producing the difference
has not been localized. This experiment does not establish a NumPy correctness
bug; small numerical differences are expected across numerical environments.

The native solver is sensitive to these small changes. The active-constraint
history changes, then its local tangent-cone QP fallback reports:

```text
CMC QP AL fallback did not converge:
stationarity=1.18637e-06, violation=3.4788e-12, iterations=19
```

`RobustQp.cpp` requests stationarity at most `1e-7` and violation at most `1e-9`.
The first criterion fails despite the small violation. These 19 iterations are
inside AL, not the trajectory optimizer. GTSAM's BCL loop can stop when the inner
subproblem fails its convergence test; the exact inner stopping branch was not
instrumented in this investigation. Neither tolerances nor solver behavior were
changed to obtain a successful result.

## Reproduction and provenance

The isolated workspace is `/tmp/cmc-linearization-investigation`. It contains
`run-case.py`, `run-followups.py`, `run-numpy25.py`, `comparison.json`, and each
run's configuration, loaded-library hashes, initial Values, logs, and outcome.
The `head` worktree/build are links to the preserved PR investigation; `shared`
is a detached worktree at `f71c907d` with an independent Release Clang 18.1.3 build.
Both load GTSAM `84b26b1cb3469a6384f691d605bf76d8db4901fe` from the same build.
No solver source was edited in either worktree.

The frozen recording, model, and labels are the earlier investigation's
`/tmp/pr469-investigation/inputs`. Model packages are MuJoCo 3.11.0,
MJLab 1.6.0, and mujoco-menagerie 2026.10.6. The case uses 78–98 seconds,
1000 samples at 50 Hz, five outer iterations, and zero dynamics initialization.
Python/C/NumPy seeds are zero; OMP/OpenBLAS/MKL threads are one.

After the isolated builds exist, the matched follow-up runs are reproduced with:

```bash
/home/dellaert/miniconda3/envs/py312/bin/python   /tmp/cmc-linearization-investigation/run-followups.py
```

Run-specific `command.json` files record the actual solver interpreter and
environment; the launching interpreter above only orchestrates subprocesses.
`run-numpy25.py` additionally records the loaded NumPy version and module path,
because installed-distribution metadata alone can identify a different NumPy
when explicitly overriding its import path.

One initial head launch stopped at a harness path assertion because the build
path was a symlink. The assertion was corrected to compare resolved paths; that
launch executed no solve and contributes no numerical or timing result.

## Consequence for the refactor report

The old failed rows remain valid observations of their recorded environment.
They do not demonstrate a regression in #469 or `f71c907d`, and matching failures
do not establish solution equivalence for those runs. The new successful paired
run provides direct equivalence evidence for `45172046` versus `f71c907d`.
The subsequent ownership refactor was deliberately excluded from this experiment.

A complete benchmark rerun is not required to explain this discrepancy. A change
to QP robustness would change the numerical implementation and require renewed
validation and benchmarking. Pin and record the loaded Python numerical packages
and initial-state fingerprints in future benchmark runs.
