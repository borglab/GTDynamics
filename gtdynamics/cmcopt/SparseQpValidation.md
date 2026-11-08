# Sparse tangent-QP validation history

The current implementation uses GTSAM `c940b4415` or the optional native
ProxQP backend. The refreshed 171-slice, 20-second kinodynamics comparison
passes nonlinear feasibility and independent physics checks for both full-QP
backends: median optimization is 11.1138 s for ProxQP and 23.9122 s for GTSAM,
with final objective 8,931.759726. The app replays the median ProxQP run.
See the [current results and dependency information](../../apps/kinodynamics/README.md#native-proxqp-comparison).

The notes below preserve the earlier solver failures, fixes, and measurements;
their acceptance status and dependency revisions describe those historical
experiments.

## November 5, 2026

Two numerical defects exposed by G1 now have local fixes on top of GTSAM PR
2912. All targeted suites and the 51-sample trajectory pass. The full cold-start
QP remains under validation, so full-workload acceptance remains open. The
measured small-case slowdown and full-case active-set work must not be confused with the numerical failures below.

## Local fixes and matched measurements

GTSAM worktree branch `codex/cmc-qp-square-root` contains three local commits:

- `0e96c71c4`: retain every finite weighted coefficient in constrained QR.
- `8f7c52063`: retain original Jacobian costs through `QpCost`, gradient/error
  evaluation, linearization, and sparse QR elimination.
- `b20aa2673`: inspect feasible zero-regularization QP minima immediately,
  avoiding a duplicate sparse solve before checking multipliers.

The GTDynamics adapter now stacks matching-scope costs as Jacobian factors,
matching the old compaction representation. Its previous Hessian conversion
was an added source of roundoff. Both builds use the fixed GTSAM headers and
library; TBB remains enabled in both projects. No QP tolerances, iteration
limits, regularization, or starting-value conventions were changed. No code
was pushed or PR opened.

### Why the previous active-set implementation behaved differently

PR 472 includes an active-set QP refinement method, but its default
`iqpMaxIterations = 0` disables that refinement. The default path constructs a
feasible step and stops; it does not require a QP optimality certificate. When
refinement is enabled, its local $N$ bases eliminate the working constraints
before ordinary Gaussian solves. The new path expresses those rows as hard
constraints in GTSAM, exercising constrained QR directly.

Two distinct representation problems were established:

1. Forming a Hessian from least-squares costs and converting it back to a
   Jacobian loses weak directions and can introduce roundoff-scale negative
   eigenvalues. The captured 216-dimensional failing factor below reproduces
   the conversion exception. Preserve square-root factors through the sparse
   solve instead. Hessian-native QPs retain their existing behavior.
2. `noiseModel::Constrained::QR` discarded raw coefficients with magnitude at
   most `1e-9`, before considering their weights. After elimination, a tiny
   coefficient with a tiny sigma can still carry substantial information.
   The fix includes all finite weighted coefficients and retains the existing
   weighted rank test. Hard rows remain handled separately.

A minimal regression has the finite equation `1e-10 * y = 2e-10` with sigma
`1e-10`, next to the hard equation `x = 0`. This is an ordinary unit-weight
`y = 2` measurement. The original QR routine drops it and returns rank one;
the fixed routine returns rank two and the correct equation. Running the new
regression against the original routine confirms this failure.

The captured small G1 working graph had stationarity infinity norm
`0.0264157349144`. Keeping Jacobians alone did not remove it, and five residual
correction solves made essentially no improvement. Removing the raw-coefficient
cutoff reduces an independent residual evaluation to `2.8012e-9` and allows
the unchanged QP convergence checks to pass.

The zero-regularization path previously waited for another identical solve to
observe a small step before inspecting multipliers. GTSAM already inspected
feasible candidates immediately when additional proximal regularization was
positive. Extending that existing check to zero avoids redundant solves while
retaining every primal and optimality check. Regressions require an
unconstrained QP to finish in one iteration and a bound-release QP in two.

### Validation and timing

Passing: all 13 affected GTDynamics C++ suites, all six native G1 Python
integration tests, and GTSAM's `testQpSolver`, `testQpProblem`, `testNoiseModel`,
and `testLpSolver`. Added upstream regressions cover square-root cost error and
linearization, release of a boundary in a QP with a weak finite direction,
and preservation of small weighted coefficients. Dependent/opposing-row,
vector-block, damping, retry, and nonlinear retraction tests continue to pass.

Matched sequential runs, with no concurrent builds or tests, use GTSAM
`b20aa26730423e9dc46fe8c05ece7179a26332f2`, library SHA-256
`0d4b644367883eef62550a34d4c832ab904847367b7ea522d06c0d66390e197a`.
Both have TBB on and identical inputs. The small run uses 51 samples, two outer
iterations, and one retraction worker.

| Measurement | PR 472 | Sparse QP with fixes |
| --- | ---: | ---: |
| Optimization | 1.256652 s | 2.280635 s |
| Final objective | 874.3249215582 | 867.0325813253 |
| Equality violation L2 | 1.0281e-12 | 1.0355e-12 |
| Inequality violation L2 | 5.4208e-8 | 2.9862e-8 |
| Feasible at 1e-5 | Yes | Yes |

The converged-QP path takes about 1.8 times as long in this small case. It also
reaches a different, lower objective after the same two outer steps. These are
single-run observations, not a convergence-rate or speedup claim. The two
accepted LM trials take 1 and 6 QP iterations respectively; each sparse
active-set iteration rebuilds and eliminates its working graph. Before the
duplicate-solve fix, a matched run took 3.288777 s, with 2 and 12 QP iterations,
and exactly the same final objective. That fix reduced this observed runtime
by about 31%.

For the full 1,000-sample, five-step, eight-worker workload, the new PR 472
baseline reproduces the published result: configuration initialization
`1.240839 s`, optimization `28.978726 s`, total initialization and solve
`41.010679 s`, objective `9074.0827634688`, and nonlinear feasibility true.
A full sparse-QP diagnostic was stopped after more than 100 iterations of the
second QP, while it was still making progress. It started that QP with all 378
rows active; 330 remained at iteration 100. Individual sampled active-set
iterations took about 3–3.5 seconds. This is active-set work, not the earlier
fixed-point stall, and there is no completed full-workload runtime or final
objective to compare.

The progress hook invokes the unchanged solver and logs every twentieth
iteration. Its explicit linkage loaded system TBB 12.11, whereas the matched
completed runs loaded TBB 12.13. Treat this as an iteration diagnostic, not a
matched runtime comparison.

A separate diagnostic used the same zero starting point but an empty fresh
working set, explicitly preloading matching TBB 12.13. With GTSAM `8f7c52063`
(before the duplicate-solve improvement), the full second QP **converged in
151 iterations with 62 active rows**, passed its unchanged certificate, and
produced an accepted nonlinear step with logged objective approximately
18,420. The probe was stopped during the following outer step. This establishes
that the identified numerical fixes solve the full constrained QP in this
case; it is not a completed five-step trajectory measurement. Compilation of
the subsequent duplicate-solve fix overlapped the end of this diagnostic, so
its elapsed time is not a clean performance result.

The empty-working-set experiment changes no production code and reuses no
previous solution. All inequalities remain in the QP, and GTSAM adds blocking
rows itself. Initializing every homogeneous row as active forces many releases;
starting empty still adds rows individually. Working-set initialization and
reusing sparse elimination work are possible future GTSAM performance tasks.
The production adapter retains the agreed ordinary cold solver call.

Raw final records are `cmc-final-small-{baseline,candidate}.json` and
`cmc-final-full-baseline.json`. The full progress logs are
`cmc-fixed-full-candidate.log` and `cmc-empty-full.log`. These, the regression
logs, the captured small working graph, and diagnostic sources are preserved
under `build/qp2912-diagnostics/local-fixes` (ignored local artifacts). The
minimal regressions themselves are committed in the GTSAM worktree.

## Before local fixes: GTSAM PR 2912 and TBB

Built [GTSAM PR 2912](https://github.com/borglab/gtsam/pull/2912), branch
`codex/mpc-sparse-active-set`, at
`f3a49dcb0272c31bbb34f9c53d17cdc11e3badd2` in an isolated managed worktree:
`/home/dellaert/.codex/worktrees/gtsam-qp2912/gtsam`.
Both comparison versions were rebuilt against its headers, library, and Python
bindings, with `GTSAM_WITH_TBB=ON` and `GTDYNAMICS_WITH_TBB=ON`.
Release, CUDA, bundled Eigen, and the other GTSAM numerical configuration
options match the previous build. Actual loaded library paths were recorded;
the Python runs both loaded TBB `libtbb.so.12.13`. GTSAM library SHA-256:
`3cd45c5e84ebab54fb1a7999ed1832c2c3ec1ba0aa3ddffe47b6adf0f76a7bea`.

At that stage, the candidate's `build` used this dependency. PR 472 (`ad64754e`) uses a
separate `build-qp2912` directory, preserving its original build. The cached
`GTSAM_INCLUDE_DIR` was explicitly updated along with `GTSAM_DIR`; initially
it retained the old header path and caused a linker mismatch. Both final
builds and Python imports were verified before measurement. No GTSAM source
changes, QP tolerance changes, additional regularization, or warm starts were
introduced. QP iteration limit remains 1,000.

Validation passes:

- All four upstream suites: `testQpSolver`, `testNoiseModel`, `testLpSolver`,
  and `testQpProblem`.
- All 13 affected GTDynamics C++ suites, including `testIELMQp`,
  `testSparseQpDependentRows`, and `testFixedKinematicsDynamics`.
- All six native G1 Python integration tests.

### Full trajectory

One sequential run per version, without concurrent compilation. Identical
78–98 s inputs, 1,000 slices, five outer iterations, eight retraction workers,
contact-GP-dynamics initialization, and `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`.

| Measurement | PR 472 + GTSAM 2912 | Sparse QP + GTSAM 2912 |
| --- | ---: | ---: |
| Configuration initialization | 1.2391 s | Not separately recorded on failure |
| Coupled optimization | 28.6109 s | Failed during the second outer step |
| Total initialization through solve | 40.7803 s | No completed solve |
| Initial objective | 368,865.9968373461 | Same starting problem; log prints 368,866 |
| Final objective | 9,074.0827634057 | No final result |
| Equality violation L2 | 4.1052e-12 | No final result |
| Inequality violation L2 | 3.7840e-8 | No final result |
| Feasible at 1e-5 | Yes | No final result |
| Completed outer iterations | 5 | 1 |

The candidate completed one step, then exhausted all nine damping trials
from 0.001 through 100,000. Wall time from clip preparation to failure was
25.5421 s; this is not a successful optimization runtime. Peak RSS was
10,396,360 KiB. No full-workload speedup or slowdown ratio can be reported.

A separate GDB run identifies the throw in
`JacobianFactor::JacobianFactor(const HessianFactor&)`, called while assembling
`EliminateQR` inside multifrontal elimination. This occurs before constrained
QR can process the assembled Jacobian. It is distinct from the redundant-dual
failure fixed by PR 2912.

A temporary capture hook saved a failing two-key factor with dimensions
111 and 105 (217 augmented rows). A standalone program using only GTSAM
reproduces the same conversion exception from the saved matrix. Its 216x216
Hessian has minimum eigenvalue approximately -1.94e-9 and maximum 8.00e6
(relative minimum -2.42e-16). This is consistent with roundoff-scale
indefiniteness; the spectrum alone does not establish the complete root cause.

### Small-trajectory diagnostic

The matched 51-sample PR 472 rerun completed in 1.2512 s of optimization,
objective 874.324921558, and remained feasible. The candidate's second outer
step exhausted its 1,000 QP iterations at lambda 0.001 and entered retries.
This additional probe was stopped during retries; it has no completed runtime
or final result. A short diagnostic compilation overlapped part of the probe,
so its elapsed time is deliberately excluded from performance comparisons.

A separate bounded diagnostic delegated every calculation to the unchanged
solver and inspected its state at QP iteration 20. The four active rows and
values were unchanged, but the checks were:

| Check | Measured value | Required tolerance |
| --- | ---: | ---: |
| Stationarity infinity norm | 0.0264155622450 | 1e-7 |
| Absolute primal-dual gap | 9.621465527e-6 | 1e-7 |
| Maximum primal violation | 8.995845654e-16 | 1e-7 |
| Largest positive active inequality multiplier | 0 | 1e-7 |

Thus the new certificate rejects an iterate whose values have stopped changing
within the step tolerance, which the old step-change criterion could accept. Tolerances were not relaxed. The earlier
2.1x small-case ratio belongs to the old GTSAM build and must not be presented
as a result for PR 2912.

### Local artifacts and reproducer

Raw results, logs, the captured matrix, and diagnostic sources are retained
under `build/qp2912-diagnostics` in this GTDynamics worktree (ignored build
artifacts). The GTSAM-only reproducer can be built and run with:

```sh
cmake -S build/qp2912-diagnostics -B build/qp2912-diagnostics/repro-build \
  -DGTSAM_DIR=/home/dellaert/.codex/worktrees/gtsam-qp2912/gtsam/build \
  -DCUDAToolkit_ROOT=/usr/local/cuda
cmake --build build/qp2912-diagnostics/repro-build
build/qp2912-diagnostics/repro-build/reproduce \
  build/qp2912-diagnostics/cmc-qp2912-failing-hessian.txt
```

It prints the reproduced conversion failure and returns zero when the captured
failure is observed; it is a diagnostic, not a passing robustness test. The
baseline/candidate numerical records are `cmc-qp2912-full-{baseline,candidate}.json`,
and the optimality-check record is `cmc-qp2912-small-certificate.log`.

The sections below preserve historical results with GTSAM `84b26b1c`, before
PR 2912, and do not describe the currently linked build.

## Dependency reproducer

`tests/testSparseQpDependentRows.cpp` uses only GTSAM APIs. Both problems have
objective `(x-2)^2/2`, start at `x=0`, select the sparse subproblem solver, and
set additional regularization to zero:

- `-x <= 0`, `-2x <= 0`: unique minimizer `x=2`.
- `-x <= 0`, `x <= 0`: unique feasible point and minimizer `x=0`.

Both throw `IndeterminateSystemException` near the auxiliary variable 1. A
stack trace reaches Cholesky elimination from `ActiveSetSolver::iterate`;
no CMC geometry or retraction is involved. The primal objectives are strictly
convex, so adding LM damping does not address the dependent-row issue.

Reproduce with the configured GTDynamics build:

```sh
cmake --build build --target testSparseQpDependentRows testIELMQp
ctest --test-dir build -R '^(testSparseQpDependentRows|testIELMQp)$' --output-on-failure
```

The standalone test can also be copied into GTSAM's constrained tests. Repair
belongs in the sparse QP backend; the GTDynamics acceptance tests should pass
unchanged after that repair.

## Targeted results

Release build with Clang 18 and GTSAM revision
`84b26b1cb3469a6384f691d605bf76d8db4901fe`:

- Eight existing C++ suites pass: `testIEManifoldOptimizer`,
  `testIEManifoldLinearization`, `testIEConstraintManifold`, `testTangentCone`,
  `testReducedALRetractor`, `testArticulatedRetractor`, `testIEHalfSphere`, and
  `testIECartPoleWithFriction`.
- Nine of ten new `testIELMQp` cases pass. The dependent/opposing-row case
  fails for both inputs. Passing cases cover coupled components and free
  variables, same-scope cost compaction, partial vector blocks, slack-boundary
  retraction, damping/predicted reduction, fixed components, singular-trial
  rejection, and exhausted-retry failure reporting.
- Both standalone GTSAM reproducer cases fail as described above.
- `testKinodynamicInitializer` and `g1_full_dynamics_factors` pass.
- `testFixedKinematicsDynamics` fails in `AccelerationIsEstimated` and
  `DownwardPriorAtConeApex` with an indeterminate-system error after all damping
  retries. The unchanged PR 472 binary passes this suite with the same GTSAM
  library. These are production-path regressions, not just synthetic cases.
- All six G1 native Python integration tests pass.

## Bounded G1 comparison

One run per version, using PR 472 (`ad64754e`) as baseline, the same GTSAM
library, cached canonical MuJoCo model, SSIO inputs, and contact-GP-dynamics
initialization. Clip 78–79 s contains 51 samples and 5,379 component equality
coordinates. Both runs use two outer iterations, one retraction worker,
`OMP_NUM_THREADS=1`, and `OPENBLAS_NUM_THREADS=1`. Python is 3.12, NumPy 2.5.2,
and MuJoCo 3.11.0. GTSAM's own TBB configuration is identical for both runs.

| Measurement | PR 472 | Sparse QP |
| --- | ---: | ---: |
| Initial objective | 21,098.67412492842 | 21,098.67412492842 |
| Final objective | 874.3250628411 | 867.0327222031 |
| Equality violation L2 | 1.0541e-12 | 1.0182e-12 |
| Inequality violation L2 | 5.4208e-8 | 2.9861e-8 |
| Feasible at 1e-5 | Yes | Yes |
| Coupled optimization | 1.2139 s | 2.5353 s |
| Component setup + feasibility restoration | 0.6132 s | 0.6165 s |
| Preparation through solve wall time | 2.3371 s | 3.6561 s |

Candidate peak process RSS was 1,242,036 KiB, including Python/model loading.
The paired runs used GTSAM library SHA-256
`d8b278f84cc498103c6a91188c2284e2140d07e9565317607069cdb33c17671a`.
These are bounded, single-run observations, not convergence or speedup claims.
The sparse run is slower in this sample and takes different steps because it
always solves the tangent QP; PR 472 defaults to a feasible-step approximation.
The sparse backend also chooses its own ordering/elimination, whereas the old
G1 path supplied a chronological ordering and sequential Cholesky.

A preliminary ungrouped-cost run was stopped after excessive memory use.
The final code preserves same-scope cost compaction before `QpCost` conversion;
the reported candidate run uses that final path. Historical reports and app
recordings were not regenerated.


## Full G1 comparison

Measured after the bounded comparison, at the user's request, using the
README's full workload: 78–98 s, 1,000 source slices at 50 Hz, five outer
iterations, eight retraction workers, and contact-GP-dynamics initialization.
PR 472 is `ad64754e`; the candidate is `d53fd140`. Both use the same inputs,
cached model, Python environment, GTSAM library hash, and single-threaded
OpenMP/OpenBLAS settings listed above. Runs were sequential, one per version.
The reduced component dimension is 106,584.

| Measurement | PR 472 rerun | Sparse QP |
| --- | ---: | ---: |
| Configuration initialization | 1.2460 s | Not separately recorded on failure |
| Coupled optimization | 27.9800 s | Failed during the second outer step |
| Total initialization through solve (README scope) | 39.7690 s | No completed solve |
| Initial objective | 368,865.9968373461 | Same starting problem; log prints 368,866 |
| Final objective | 9,074.0828569349 | No final result |
| Equality violation L2 | 4.0946e-12 | No final result |
| Inequality violation L2 | 9.4392e-8 | No final result |
| Feasible at 1e-5 | Yes | No final result |
| Completed outer iterations | 5 | 1 |

The baseline reproduces the README's 1.245 s initialization, 28.01 s
optimization, 39.64 s total, and 9,074.082857 objective. The total column uses
`totalInitializationAndSolveSeconds`, excluding clip preparation and export.
It is distinct from the component-setup `initializationSeconds` native field.

The candidate completed its first step, reducing the objective to approximately
75,950. The next QP failed at all nine damping values from 0.001 through
100,000, throwing `IndeterminateSystemException` near reduced variable
18298068214481855. LM then reported exhausted damping trials. Wall time from
clip preparation through exception handling was 27.1219 s; this is **time to
failure**, not a completed optimization time or a speedup. Peak process RSS
was 10,438,284 KiB.

A separate timing-only diagnostic run identifies the failure in the first
**primal** Gaussian solve of the second QP, with 378 active scalar inequality
rows. No dual graph or multiplier solve was reached on any of these retries.
This establishes the failing stage, but does not establish that this failure
has the same cause as the dependent-row unit-test failures. Increasing primal
LM damping did not resolve it. No solver fallback or GTSAM modification was
used.

Local raw artifacts are `/tmp/cmc-qp-full-{baseline,candidate}.json`, their
`.log` files, `/tmp/cmc-qp-full-{baseline,candidate}-time.txt`, and the runner
`/tmp/cmc-qp-full-g1.py`. Diagnostic full-run records are
`/tmp/cmc-qp-profile-full.{json,log}`.

## Measured cause of the bounded-run slowdown

A separate diagnostic rerun of the 51-sample case timed calls to the unchanged
GTSAM library using a temporary interposition library. It delegated all
computations to the original methods and produced the same final objective.
These instrumented timings are separate from the headline benchmark timings.

The second outer step used 12 QP iterations. Its working set shrank from nine
active scalar rows to four, releasing one row at a time. Each iteration calls
`buildWorkingGraph(...).optimize()`, which performs fresh multifrontal
elimination and back-substitution. An iteration that checks multipliers also
repeats the primal solve; this sparse implementation does not reuse its
previous factorization.

| Second-step diagnostic | Wall time |
| --- | ---: |
| All 12 QP iterations | 1.2166 s |
| Primal Gaussian solves within those iterations | 1.2122 s |
| Dual-graph construction | 0.0032 s |
| Dual Gaussian solves | 0.0003 s |

PR 472 reports two linear solves for that step and accepts its default
feasible-step approximation. The candidate requires a converged QP. Repeated
primal solves are the dominant measured QP cost and explain much of the
small-run slowdown; multiplier calculations are negligible here. Retraction
iteration counts actually fell from 168 to 109 for that step. The candidate
also changes ordering/elimination and adds QP assembly during linearization;
this measurement does not isolate their individual contributions.

The full workload has no completed candidate timing, so the small-case 2.1x
ratio must not be extrapolated to it. Fix the production sparse-solver failure
before making a full-workload performance comparison. Diagnostic sources and
logs are `/tmp/cmc-qp-profile.cpp`, `/tmp/cmc-qp-profile.so`, and
`/tmp/cmc-qp-profile-small.{json,log}`.
