# Equality-only optimization through CMC

Authors: Yetong Zhang and Frank Dellaert.

Equality-only problems have the stronger `EConstraintManifold` / `ECMPoint`
contract, specializing `IEConstraintManifold` / `IECMPoint`. Both categories
share equality preparation, `CMTangentSpace`, recovery, and transformed factors.
The GTSAM LM/Gauss–Newton/Dogleg wrappers and hand-written
`LMManifoldOptimizer` use `ECMPoints`; CMC's LM also accepts them through explicit
widening. General IE definitions can still describe empty inequality collections.
The old `ConstraintManifold` value representation and duplicate substitution
implementation have been removed. See the
[migration guide](../constrained/README.md#migration-and-solver-behavior).

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
  --include-proxqp --include-manifold-lm
```

Omit `--include-proxqp` for a build without that backend.
`--include-manifold-lm` adds the migrated `LMManifoldOptimizer` on `ECMPoint`
values and a one-worker CMC run. It measures the retained equality-only LM
algorithm, not the removed historical value implementation. The optional model cache
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

## Historical pre-migration comparison (`6aea8ba1`)

The numerical tables below are preserved from baseline commit `6aea8ba1`.
Reproducing the old value implementation requires that revision and its
`--include-legacy` option. The current runner uses `manifold-lm` for the migrated
backend and does not relabel these historical results as new measurements.

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

At that revision, this baseline called `LMManifoldOptimizer` with actual
`ConstraintManifold` values and `SubstituteFactor` costs. It did not replace the
old optimizer with CMC under a different label. Both start from the same restored
ambient values and use the same equality constraints, independent-coordinate
selection, objective, LM damping, ordering, and iteration budget. Old CM uses its
existing `BasisRetractor`, which fixes those independent coordinates and solves
for the remaining values. CMC uses articulated forward kinematics and conditioned
affine mechanics. They implement the same equality chart in this workload through
different numerical procedures. This is an optimizer comparison from common
restored values, not a rerun of an old application's entire initialization
pipeline. The old CM implementation was unchanged for that comparison; the
current implementation retains its LM loop on the shared typed representation.

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
builders for CMC and, at baseline commit `6aea8ba1`, the actual old-CM value
implementation. The current runner compares the migrated `manifold-lm` backend
using `ECMPoint`; the times below remain historical baseline measurements.

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
nonlinear and affine historical comparisons established agreement before the
representation migration. The current code shares the typed representation
while preserving the separate optimizer algorithms.

## Typed migration comparison

The migration was compared with the independently preserved source and binaries
from baseline commit **`6aea8ba1`**, rather than reusing the historical timings
above. Both versions used the same Release Clang 18.1.3 configuration, GTSAM
`c940b44158d8bfa7394da233dd63aa225bc8fed2`, ProxSuite 0.7.3, system TBB,
CPUs **8–15**, single-thread BLAS/OpenMP, and identical input/model files.
The comparison checks GTSAM/TBB binary hashes, CPU affinity, input hashes, and
nonlinear solver settings. Each case ran three times in fresh processes,
sequentially with alternating case order. No builds or tests ran during timing.

The affine workload contains 57 fixed-q/v knots in 78–98 seconds, 2,337 reduced
coordinates, and one retraction worker. These times measure the complete native
Dynamics call, including preparation and diagnostics, excluding GP fitting:

| Backend | Baseline median (s) | Migrated median (s) | Change |
| --- | ---: | ---: | ---: |
| Constrained CMC reference | 0.650716 | 0.662025 | +1.7% |
| Equality CMC Gaussian | 0.591452 | 0.602626 | +1.9% |
| Equality CMC GTSAM QP | 0.637837 | 0.651371 | +2.1% |
| Equality CMC ProxQP | 2.466586 | 2.574530 | +4.4% |
| Equality `LMManifoldOptimizer` | 0.757107 | 0.539048 | −28.8% |

For `LMManifoldOptimizer`, median LM time falls from 0.472522 to 0.257588 s;
manifold preparation changes from 0.172791 to 0.181494 s. Its migrated values
use type-owned constant affine geometry throughout the LM loop. Construction
counter tests independently verify that affine geometry is constructed once per
type, rather than once per point. Other CMC affine cases retain one numerical
equality construction for each of their 57 types.

The nonlinear workload uses the default `contact-gp-dynamics` initializer,
171 support slices in the same 78–98 second clip, 18,297 reduced coordinates,
and five outer LM iterations. These times measure the solver stage, including
transformed-problem preparation for manifold LM and optimizer teardown, and
exclude initial feasibility restoration:

| Backend | Workers | Baseline median (s) | Migrated median (s) | Change |
| --- | ---: | ---: | ---: | ---: |
| Equality CMC Gaussian | 8 | 4.850847 | 4.862299 | +0.2% |
| Equality CMC GTSAM QP | 8 | 7.687887 | 7.753216 | +0.8% |
| Equality CMC ProxQP | 8 | 8.332356 | 8.327177 | −0.1% |
| Equality CMC Gaussian | 1 | 10.399411 | 10.309044 | −0.9% |
| Equality `LMManifoldOptimizer` | 1 | 13.568809 | 14.069335 | +3.7% |

No median slowdown exceeds the 5% investigation threshold. The nonlinear
manifold-LM loop remains serial and separate from CMC's loop; no new solver
algorithm or parallelization is implied by this migration.

Every matched before/after case has exactly equal objectives and state arrays
in all three repetitions. Affine final objectives are 13.100276284396958
(13.10027628439752 for ProxQP); maximum equality violation L2 is 2.17e-12.
Nonlinear final objectives remain approximately 8891.10444204, with maximum
equality violation L2 3.78e-11. Original omitted-bound violation remains
approximately 149.1588343 for the nonlinear relaxation and zero for this affine
workload. Equality-only feasibility does not assert satisfaction of omitted
inequalities. App recordings are unchanged.

An additional independent MuJoCo check agrees within 1.84e-12 N/Nm and verifies
resultant/corner contact-cone agreement at all 57 affine knots. Its temporary
export is under `/tmp/typed-affine-validation/`; the validation report is
`build/typed-final-affine/independent-validation.json`.

Raw runs, state arrays, provenance, source patches, and source hashes are in
`build/typed-baseline-affine/`, `build/typed-baseline-nonlinear/`,
`build/typed-final-affine/`, and `build/typed-final-nonlinear/`.
`build/typed-after-affine/` and the incomplete `build/typed-after-nonlinear/`
were preliminary runs before final review fixes and are excluded above.
Recreate the checked comparison with:

```sh
python examples/g1_mjlab/compare_manifold_migration.py \
  --affine-before build/typed-baseline-affine \
  --affine-after build/typed-final-affine \
  --nonlinear-before build/typed-baseline-nonlinear \
  --nonlinear-after build/typed-final-nonlinear \
  --output build/typed-migration-comparison.json
```
