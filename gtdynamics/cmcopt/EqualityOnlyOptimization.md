# Equality-only optimization through CMC

Authors: Yetong Zhang and Frank Dellaert.

Empty inequality collections define equality-only spaces. These problems use
`IEConstraintManifold`, `CMPoint`, `CMTangentSpace`, and the same CMC LM loop as
problems with corners. The old equality-only `ConstraintManifold` value and its
optimizer are not involved.

## Selecting the linear step

```cpp
IELMParams params;
params.linearSolver = IELMParams::LinearSolver::Automatic;
IELMOptimizer optimizer(params);
auto result = optimizer.optimize(costs, equalities, {}, initialValues);
```

`Automatic` selects a direct Gaussian solve only if every point reports zero
inequality blocks. A slack inequality is still an inequality; having no active
QP rows does not enable this path. `Gaussian` explicitly requires an equality-only
problem and rejects inequalities. `QP` explicitly uses the existing
`qpBackend` selection, including GTSAM or optional ProxQP, even with empty
inequalities. QP iteration and initialization settings apply only to QP steps.

For either solver, the common compacted objective is

\[
\min_\xi \tfrac12\|J B_x\xi+r\|^2+
           \tfrac\lambda2\|D\xi\|^2.
\]

Equality geometry eliminates equality directions before the solve. Gaussian
steps call `SolveLinear` directly and honor `lmParams.linearSolverType` and
ordering; its default is GTSAM multifrontal Cholesky. The sparse GTSAM QP path
uses its own solver and QR machinery. Both preserve the same diagonal or
isotropic damping, restore zero-dimensional increments, lift through the same
basis, and evaluate predicted reduction on the undamped model. Numerical failure
rejects the trial through LM damping retries. Neither path silently switches
backends.

Equality-only states retain the compacted Gaussian objective without packing a
QP. Explicit QP steps pack that objective when requested. Nonlinear equality
retraction, projection metrics, callbacks, history limits, and parallel component
retraction remain part of the same CMC implementation.

## Timing the bounded G1 relaxation

`kinodynamic_walk.py --equality-only` omits inequalities from discovery,
optimization, and retraction. It preserves the original inequality collection
only for reporting `originalInequalityDimension` and `originalInequalityL2`.
`feasible` describes the selected relaxed problem, not satisfaction of those
omitted bounds. This is a numerical benchmark, not a replacement contact-feasible
trajectory or an updated app recording.

With the normal G1 Python/library environment, run:

```sh
python examples/g1_mjlab/benchmark_equality_only.py \
  --data /path/to/openio-contact-source \
  --labels apps/contact-labels/public/snippets/middle.json.gz \
  --model-cache /path/to/prepared/g1-mjlab \
  --output build/equality-only-comparison \
  --include-proxqp --include-legacy
```

Omit `--include-proxqp` for a build without that backend. `--include-legacy`
adds old CM-Opt and a one-worker CMC run; the old optimizer has serial retraction. The optional model cache
must contain the existing prepared `scene.xml` and its assets; otherwise normal
G1 model loading is used. This runner uses Linux CPU affinity and runtime library
maps. Configure its environment for matching GTSAM/GTDynamics libraries and TBB,
and single-thread BLAS/OpenMP.

The workload is fixed to 78–98 seconds, 171 single/double-support slices, five
outer LM iterations, the default `contact-gp-dynamics` initializer, eight TBB
retraction workers, and automatic ordering. Both QP backends solve to convergence;
projected-gradient initialization and additional QP regularization are off.
Three fresh processes per solver run sequentially with alternating order. Raw
summaries, arrays, source hashes, input/library hashes, and an aggregate
`comparison.json` are written beneath the output directory.

`IELMOptimizer::timings()` reports objective linearization/compaction, the full
linear step (damping, solver preparation, solve, checks, and lifting), and the
nonlinear update (retraction plus candidate evaluation). These times include
rejected trials and are elapsed wall times, not factorization counts. They do
not sum to the entire optimization time: graph transformation, state bookkeeping,
callbacks, and stopping logic also take time. Initial feasibility restoration is
reported separately. Solver-call counts distinguish Gaussian and QP attempts.

## Measured comparison

Three fresh runs per case used the same Release Clang 18.1.3 build, GTSAM
`c940b44158d8bfa7394da233dd63aa225bc8fed2` (the existing local benchmark build),
ProxSuite 0.7.3, system TBB 2021.11, CPUs 0–7, single-thread BLAS/OpenMP,
and identical inputs. GTSAM's library SHA-256 was
`49edc43dc92749353306c1c94ce7cdc0d951b719e06f611450aa6679c526940f`.
The new `develop` dependency policy does not change the recorded revision used
for this comparison. No builds or tests ran concurrently with timing.

| Solver | Retraction workers | Median solver stage (s) | Final objective |
| --- | ---: | ---: | ---: |
| Old CM `LMManifoldOptimizer` | 1 | 13.586038 | 8891.104442039225 |
| CMC direct Gaussian | 1 | 10.494301 | 8891.104442041498 |
| CMC direct Gaussian | 8 | 4.849476 | 8891.104442041498 |
| CMC GTSAM full QP | 8 | 7.736967 | 8891.104442041473 |
| CMC ProxQP full QP | 8 | 8.307647 | 8891.104442041873 |

The old solver stage includes 0.900690 s of transformed-problem preparation
at the median run. Excluding that setup, its median LM stage is **12.685348 s**,
versus **10.494301 s** for one-worker CMC: CMC is **17.27% faster** even after
excluding the old setup cost. All solver-stage timings include optimizer
teardown. The preliminary three-backend-only campaign reported 4.650741 s
for CMC Gaussian because teardown was then outside that timer; its median
initialization-through-solve time was 6.674308 s, compared with 6.685277 s
in the final campaign. Use the final campaign for comparisons.

This baseline calls the existing `LMManifoldOptimizer` with actual
`ConstraintManifold` values and `SubstituteFactor` costs. It does not replace the
old optimizer with CMC under a different label. Both start from the same restored
ambient values and use the same equality constraints, independent-coordinate
selection, objective, LM damping, ordering, and iteration budget. Old CM uses its
existing `BasisRetractor`, which fixes those independent coordinates and solves
for the remaining values. CMC uses articulated forward kinematics and conditioned
affine mechanics. They implement the same equality chart in this workload through
different numerical procedures. This is an optimizer comparison from common
restored values, not a rerun of an old application's entire initialization
pipeline. The old CM implementation is unchanged.

The old/new objective difference is **2.27e-9**; the maximum group-error history
difference is **1.43e-9**. Final equality violation L2 is **3.78e-11** for old CM
and **1.73e-12** for CMC Gaussian. Maximum old/new pose-matrix entry differences
are **5.22e-11**, sole-wrench differences **1.57e-8**, and joint-torque differences
**6.31e-9**. All three repetitions are deterministic in numerical results.
The original, omitted inequality violation L2 is approximately **149.1588343**
for every solver; these are equality-only solutions.

For the eight-worker runs, median linear-step time is **1.195100 s** for Gaussian,
**4.104459 s** for GTSAM QP, and **4.658820 s** for ProxQP. Retraction time stays
near **1.97 s**. GTSAM QP needs one working iteration per LM step here; the
remaining gap is not active-set changes. Direct Gaussian bypasses QP preparation
and optimality checks and uses Cholesky, whereas the sparse GTSAM QP backend uses
QR. These measurements do not isolate the contribution of each internal operation.
The slower old serial retraction is also an implementation difference, so the
old/new speed comparison does not isolate the type/value separation alone.

The final raw runs and comparisons are in
`build/equality-only-with-legacy/`. The preliminary comparison is retained in
`build/equality-only-comparison/`; `build/equality-only-legacy-smoke/` is a
preflight run using a fresh projection workspace and is excluded from the final
comparison. All 15 final runs have matching input, binary, library, and CPU
fingerprints. The root C++ regression checks pass 13 suites (including six new
equality-only tests); seven G1 Python tests pass. Four suites also pass with
ProxQP disabled. No other initializer, full trajectory, or app recording changed.

## Affine Dynamics app comparison

The [Dynamics app](../../apps/dynamics/README.md#affine-equality-only-showcase)
provides the affine counterpart to the nonlinear benchmark above. Its GP-derived
q/v snapshot is substituted before constructing types; estimated accelerations,
contact wrenches, and torques remain free. It uses the same native problem
builders for CMC and an actual old-CM baseline.

Three bounded 57-knot runs per solver produce median complete native-call times
of **0.616 s** for CMC Gaussian, **0.685 s** for CMC GTSAM QP, **0.802 s** for old
CM, and **2.678 s** for CMC ProxQP. Gaussian, GTSAM QP, and old CM take four LM
steps and agree at objective **13.100276284396958**. ProxQP takes 19 LM steps and
35 QP attempts, reaching **13.10027628439752**. Maximum equality violation L2 is
**2.17e-12**. Each CMC case constructs exactly one numerical equality basis for
each of its 57 runtime types, independent of the number of LM trials.

This affine campaign uses CPUs 8–15, whereas the nonlinear campaign used 0–7;
compare solvers within a campaign rather than interpreting their absolute
runtimes as equal workloads. The app documentation records setup, timing
scopes, relaxed-bound diagnostics, state differences, and a reproduction command.
Independent MuJoCo validation agrees within 1.9e-12 N/Nm, and independent
resultant/corner contact checks agree at all 57 knots. The normal constrained
solve and checked-in recordings are unchanged. Both
nonlinear and affine comparisons support migration toward a single optimizer;
they do not remove the old CM implementation.
