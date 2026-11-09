# CMC-Opt: equality coordinates, a global QP, and local retraction

The unknowns are poses, velocities, accelerations, forces, torques, landmarks,
and IMU bias. Soft factors fit the measurements; equalities enforce mechanics;
inequalities limit contact wrenches and torques.

## Types, points, and tangent geometry

`IEConstraintManifold` defines a component's space through constraint factors and selected
linearization/retraction policies. `CMPoint` is a numerical value with bindings
from local slots to ambient pose, joint, force, and torque keys.
The type owns symbolic equality preparation and, when certified affine, its
constant local equality basis. `CMTangentSpace` references a read-only bound
basis and contains the active-boundary derivatives at that point. Nonlinear
numerical bases are rebuilt per point, independently of LM weights. A step is an ordinary vector in the equality coordinates.

The G1 example still discovers its components automatically. Their component
keys are independent of ambient keys; temporal ordering obtains time from each
point's bindings. Binding-specific factor views and retraction preparation are
rebuilt on point construction; mutable projection work is detached per call. Different fixed q/v values produce different temporary
affine dynamics projections, even when the slices share structure.

See the [type/value API](../../gtdynamics/cmcopt/IEConstraintManifold.md) and the
[small manual example](../example_constraint_manifold/main_shared_ieconstraint_manifold.cpp).

## What the linear step does

At each feasible point, component $i$ supplies an equality basis $B_i$.
Writing $\Delta x_i=B_i\xi_i$ removes the equality equations from the linear
step. The Gaussian objective still connects components across the trajectory.

Active inequality boundaries become $Dg_i(x)B_i\xi_i\le0$. CMC assembles these
rows and the reduced objective into one global `gtsam::QpProblem`. Each LM
trial adds damping and calls GTSAM's sparse active-set QP solver. Zero is a
feasible starting increment. GTSAM selects and releases working-set rows;
GTDynamics no longer computes a second, face-dependent nullspace $N$.

Only currently selected boundaries constrain this QP. For example, a slack
bound $x\le1$ at $x=0$ does not limit its step. Nonlinear retraction handles that
boundary if the proposed finite movement reaches it. This preserves the CMC
tangent-cone model rather than switching to affine approximations of all bounds.

## What happens after the QP

Lift the solution through $B$, retract each component to nonlinear feasibility,
and compare actual objective reduction with the undamped quadratic prediction.
LM accepts the trial or increases damping and solves another QP. Retractors
still enforce all inequalities, and can operate independently in parallel.
Only wholly active vector constraint blocks are promoted to equalities.

The global solver defaults to full convergence within `qpMaxIterations`
(default 1,000), using GTSAM's default dual-first `Automatic` formulation.
`qpUseProjectedGradientInitialization` optionally seeds primal solves; dual
solves ignore initialization strategies. `qpUseFeasibleStep` instead requests
feasible descent using GTSAM's `computeFeasibleStep`, using the same optional
projected-gradient initialization.
The G1 example selects this mode by default; `--qp-step full` requests convergence,
and `--projected-gradient-initialization` enables seeding (now disabled by default).
`--qp-ordering temporal` passes the chronological component ordering, retiring
each stance landmark at its last observation and keeping the bias until the end.
The default `--qp-ordering automatic` leaves ordering to GTSAM's COLAMD.
An optional native ProxQP backend uses the same QP and retraction pipeline;
select `--qp-backend proxqp --qp-step full` after installing ProxSuite and
enabling `GTDYNAMICS_WITH_PROXQP`. It does not offer feasible-step or
projected-gradient modes. Installation instructions are in the root README.
A numerical failure or exhausted iteration budget rejects the trial. A no-progress
result supplies a zero trial and does not certify QP optimality.
`qpIterations` counts main active-set iterations, excluding initialization solves,
not factorizations. This can change trajectories relative to the historical
configuration that skipped full QP refinement. LM still checks nonlinear
retraction and objective improvement for every proposed step.

See [the implementation contract](../../gtdynamics/cmcopt/ComponentCoordinates.md)
and [IELMOptimizerState](../../gtdynamics/cmcopt/IELMOptimizerState.cpp).
The following measurements describe the earlier implementation and are not
claims about the current sparse-QP path.

## Historical measurements before the sparse QP replacement

These measurements predate the rebase onto master at `e836e474`. That merge
includes the extracted articulated retractor, an example-specific G1 solver,
and a revised gauge. The numbers below compare the earlier implementations;
they do not measure the rebased example.

Measured 2026-11-03 on an Intel Core i7-14700F, Release Clang `-O3`, eight
retraction workers, and `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`. Every run
uses the same 78–98 s SSIO clip, 1,000 original 50 Hz slices, five outer
iterations, measurements, initialization, priors, damping, and constraints.
The original implementation is PR #468 commit
`4de63c2606da2d5be485a5f4424a2f590995758c`.

| Version | Optimization | Total from contact initialization | Optimization speedup | Total speedup |
| --- | ---: | ---: | ---: | ---: |
| Original implementation | 78.092 s | 86.909 s | 1.00× | 1.00× |
| Preserved experiment, run 1 | 33.276 s | 42.118 s | 2.35× | 2.06× |
| Preserved experiment, run 2 | 34.463 s | 43.248 s | 2.27× | 2.01× |
| Component-coordinate architecture, run 1 | 32.634 s | 41.396 s | 2.39× | 2.10× |
| Component-coordinate architecture, run 2 | 33.685 s | 42.430 s | 2.32× | 2.05× |
| Component-coordinate architecture, final run 3 | 32.547 s | 41.236 s | 2.40× | 2.11× |

Total includes contact initialization, measurement preparation and IMU
preintegration, graph assembly, feasibility restoration, optimization, and
error-history recording. It excludes input/model loading, result-array
extraction, serialization, and browser loading. The numerical runs contain
no profiling hooks.

Most of the speedup was already established by the experiment: local face
elimination, cost-factor compaction, and cached feasibility checks. The new
architecture preserves that gain while removing the constrained fallback and
extending the contract to optional refinement. These timings do not establish
a substantial additional speedup over the experiment. They also do not imply
the same gain for every robot or constraint pattern.

## Numerical checks

All 20 arrays saved by each architectural run exactly match the corresponding
arrays from the preserved experiment. Compared with the original constrained
solver, the final objective changes from `9123.931172484721` to
`9123.931184057772`, a relative difference of approximately $1.27\times10^{-9}$.
Both implementations report feasible solutions after five iterations.

The new equality residual norm is $4.14\times10^{-12}$ and the inequality
violation norm is $1.13\times10^{-7}$. Against the original solver, the maximum
joint-angle difference is approximately $0.000084$ degrees, the maximum joint
torque difference is $0.000076$ Nm, the maximum sole-force vector difference is
$0.000595$ N, and the maximum sole-moment vector difference is $0.000077$ Nm.
The maximum link-position difference is $1.12\times10^{-7}$ m. Maximum joint acceleration
differences are $0.00571$ rad/s², against a baseline maximum magnitude of
$181.24$ rad/s². The comparison therefore covers dynamic variables as well as
the displayed poses.

Validation includes:

- Fixed-working-set optima against constrained Gaussian QR, with coupled
  factors, nonunit noise, transformed damping, redundant/scaled rows, tiny
  kernel coefficients, zero-DOF components, and ordering preservation.
- Direct inspection that reduced graphs contain soft cost factors only, plus
  equality of the original and reduced quadratic costs under recovery.
- Sixty-four coupled QP cases checked against exhaustive enumeration of all
  working sets using constrained QR, with both empty and fully active starts.
- Face release, opposing and redundant rows, partial multirow blocks, failure
  on iteration-budget exhaustion, and rejection of nonlocal/affine cone rows.
- Existing CMC manifold, tangent-cone, reduced AL retraction, fixed dynamics,
  and coupled kinodynamics tests: all eight relevant native suites pass.
- Four direct Python binding tests for coupled and fixed dynamics pass.

Optional refinement can exhaust its working-set budget on difficult or
degenerate cases. It reports failure and lets LM handle the unsuccessful trial;
it does not silently switch to a global constrained backend. The original G1
benchmark disables that optional refinement, so its timing and exact-array
agreement do not measure refinement performance on a full trajectory.

## Reproduction

The numerical example remains the student-facing entry point. The command
below now uses the revised G1 solver and gauge from master, so it will not
reproduce the historical numerical results above:

```bash
python examples/g1_mjlab/kinodynamic_walk.py \
  --start 78 --end 98 --max-iterations 5 --retraction-threads 8 \
  --output /tmp/kinodynamics-architecture
```

Use the configured `py312` environment, matching GTSAM/GTDynamics Python
bindings, and prepared G1 model/data cache. In this workspace,
`/tmp/run-kinodynamics` supplies those paths and the thread environment.
Measured outputs are in `/tmp/kinodynamics-architecture-{1,2,3}/`; the machine-
readable comparison is `/tmp/cmc-architecture-comparison.json`.


The subsequent component-ownership refactor has a separate
[downloadable HTML report](../../gtdynamics/cmcopt/ComponentRefactorReport.html),
with the full example inventory and matched before/after measurements.
