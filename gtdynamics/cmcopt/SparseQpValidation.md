# Sparse tangent-QP validation — 2026-11-05

This replacement is **not ready to merge**: GTSAM's sparse active-set solver
fails on dependent active inequality rows. The implementation deliberately
retains failing acceptance tests and has no alternative global backend.

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
