# Reviving the cable robot kinodynamics experiment from PR #363

This change restores Gerry Chen's active kinodynamics experiment on current
GTDynamics. It preserves the original problem and solver schedules while using
current GTSAM constraints and GTDynamics manifold APIs.

## Provenance and ancestry

- Original PR: https://github.com/borglab/GTDynamics/pull/363
- Original head: `4722a210c85b0b764bb2a9d6c6c258cd7bd7bdb3`, branch
  `feature/manifold_optimizer_cablerobot_kinodynamics`.
- The PR metadata reports 81 commits and 43 files against its historical master
  base `a04c6b8b411917834576eccf5df022a9a6abb26a`.
- Migration target: master `9b251c918ebb71a5673da6004a3367dd3b07f255`, which includes
  naming PR #427, FK PRs #441/#443, and the GTSAM-selected pybind11 header fix.
- There are **two** merge bases with current master:
  `2dc1a06ff19386c71decc593376409bee26fa426` and
  `3353dc2d3bce7243f7a180d32a5928da97236fac`. The second is the #331 snapshot
  merged into the source by `f635026b`.
- Dependency PR #331 was merged on February 18, 2026, as
  `a6a264235093641a7c995a6746ea9031ed753ad7`. Current master contains that merge.
  Only `08b5b344`, `f635026b`, `a4e6ca74`, and `4722a210` from #363 remain outside
  master's ancestry.

The original 43-file net diff against `2dc1a06` was first squashed in an isolated
branch, then split into four file groups. The split tip was verified to have an
identical tree to the source head. Those groups were rebased onto current master:
the 37-file cable prerequisite group was dropped as landed through #331, and the
two portability edits were dropped as already applied or superseded. The two
remaining groups were adapted and supplemented with tests and this migration
record. Neither the original branch nor PR #363 was changed.

## Review guide and original-to-final mapping

| Final commit group | Source and resulting files |
| --- | --- |
| 1. Cable robot problem construction | `08b5b344`/`a4e6ca74`: `gtdynamics/utils/CableRobotUtils.{h,cpp}` moves to `gtdynamics/cablerobot/utils/`, so it builds only with cable support. Namespace, includes, parameter names, shared pointers and basis-key callback follow current APIs. |
| 2. Benchmark and wrapper integration | The original `scripts/gerry02_cablerobot_kinodynamics_manifold.cpp` and plotting notebook move under `examples/example_constraint_manifold/`. `CableRobotBenchmark.h` records explicit solver settings. The shared runner gains optional penalty/AL configuration hooks. Cable wrapper return types and local include/library precedence are corrected. |
| 3. Regression and CI coverage | New cable model/manifold and analytical solver-schedule tests; Clang Debug enables cable support and runs a 60-second bounded eight-sample smoke benchmark. Other configurations retain their defaults. |
| 4. Migration record | This provenance, semantic mapping, validation and limitation record. |
| 5. Naming alignment | Replace the revived shorthand with descriptive parameter, geometry and local names in the utility, example, tests and plotting notebook; preserve all equations and numeric settings. |

Gerry remains the author of the first two revived groups; the original Gerry Chen
and Yetong Zhang file attribution is retained. New tests and documentation are
migration work.

The final naming pass follows PR #427: `waypointWeight` and `torqueWeight`
represent Q and R, `timeStep` represents dt, and `torquePrior` is the nominal
motor torque. Geometry uses `frameMountPoints`, `endEffectorMountPoints`,
`endEffectorId` and `endEffector`; loops use `cableIndex` and `stepIndex`.
The mathematical notation below and exported CSV columns are unchanged.
Existing master APIs and wrapper declarations are unaffected by this pass.
The focused regressions, example, wrapper and stubs rebuild successfully, and
all plotting notebook cells execute. Before/after runs of all five methods at
8 and 32 samples have exactly equal recorded objectives, residuals, dimensions,
and iteration fields; all exported trajectory values are exactly equal too.
Wall-clock timings are excluded from this comparison.

## Mathematical problem

For N samples, there are 62N scalar state variables and 46N + 12 equality rows.
The manifold dimension is 16N - 12. With a pose waypoint at each sample, the cost
has 22N - 12 residual rows.

The equalities fix initial pose and zero initial twist and enforce cable length,
velocity and acceleration, wrench balance, tension-to-wrench mapping, and winch
dynamics at each sample. Equality factors retain unit sigmas. The state uses full
Pose3 and six-dimensional twist/acceleration variables, with coplanar cable
mounts. Gravity is explicitly zero, as in the source. No tension positivity,
torque bounds or extra planarity constraints are introduced.

The objective is the sum of:

- Fixed-time trapezoidal pose and twist collocation costs, sigma 0.001. These
  remain costs, not hard temporal constraints.
- Pose waypoint priors with weight Q = 100.
- Four torque priors per sample with weight R = 1, centered at 0.6.

The example retains dt = 0.01 and a default of 1000 samples. Waypoints interpolate
from (1.5, 1.0, 0) to (1.7, 1.1, 0), with identity rotation. `--num-steps` changes
the horizon while retaining dt and endpoints. The initial guess is unchanged:
cable length 1.66, tension 47, torque 0.6, zero wrenches/velocities/accelerations,
and the constant initial pose.

## Solver mapping

| Original behavior | Current implementation |
| --- | --- |
| `ConstraintsFromGraph` | `NonlinearEqualityConstraints::FromCostGraph` |
| `EqConsOptProblem` | `EConsOptProblem` |
| `ManifoldOptimizerType1` | `NonlinearManifoldOptimizer` through `ConstrainedOptBenchmark` |
| Dense `MatrixBasis` | `OrthonormalBasisCreator`, still using `FullPivLU(A).kernel()` |
| `UoptRetractor` | `UnconstrainedOptimizationRetractorCreator` |
| Initial projection | Enabled, preserving the infeasible initial guess and its retraction |
| CM(F) inner LM budget | Default LM maximum iterations, explicitly retained instead of the runner's 10 |
| CM(I) inner LM budget | One iteration; no final projection, overriding the runner's newer default |
| Soft constraints | Penalty weight 100, instead of the runner default of 1 |
| Penalty method | Exactly 15 subproblems, mu = 1, 2, ..., 2^14; outer stopping tolerances zero |
| Augmented Lagrangian | Exactly 12 subproblems; initial mu = 1 and zero multipliers; Aggressive policy with uncapped lambda += mu h; double mu when violation does not contract below 0.25 of its previous value |
| Outer LM | Soft and CM retain minModelFidelity = 0.3 and lambdaUpperBound = 1e10; penalty/AL retain default LM settings |

GTSAM's current BCL default would change the source algorithm, so the example
explicitly selects Aggressive AL updates. All source equality sigmas are one,
so raw and whitened multiplier coordinates agree. The analytical regression uses
an objective Hessian of 100 to force mu beyond the modern default dual-step cap
of 10; it would detect accidentally keeping that cap or changing the schedule.

One implementation timing change is necessary on TBB builds: construct the same
dense tangent basis eagerly. With lazy construction, an eight-sample run hung
while `ConstraintManifold::makeSureBasisConstructed` held its mutex and nested
GTSAM factor linearization stole another task that requested the same mutex.
A process stack sample confirmed that lock cycle. Eager construction avoids
parallel on-demand initialization without changing the basis or retraction
formulation. The eight-sample regression covers this path. The general lazy
basis implementation is unchanged.

## Deliberate omissions and replacements

- The 37 historical cable files listed below are #331 dependencies, not a reason
  to overwrite today's maintained factors, controllers, tests or datasets.
- `4722a210` made `predictPose` inline: current `CollocationFactors.h` already has
  that fix. Its portable Boost include edit targeted the deleted
  `ManifoldOptimizerType1.cpp`; the replacement uses current standard headers.
- The commented-out custom basis-key experiments remain inactive. The utility's
  selector is retained with the current callback signature, without debug prints;
  it is not a validated alternative benchmark mode.
- Gerry's absolute Downloads paths are replaced by the standard repository data
  directory, benchmark IDs and method-specific filenames. CSV columns retain
  their order, with corrected unique headers and full numeric precision. Output
  file-open errors are reported. The notebook uses relative paths.
- The wrapper declares `const T& prior()` to match the inherited GTSAM API.
  Python compilation uses this checkout's headers, and stub generation loads its
  built library before any old GTDynamics installation sharing GTSAM's prefix.
  Master's pybind11 provider fix is preserved.
- No old `gtdynamics/manifold`, legacy optimizer implementation, `scripts/`
  experiment copy, FK implementation or naming-refactor reversal is introduced.
  No GTSAM source or PR #2879 is modified.

## Validation

Local environment: macOS arm64, AppleClang 21, Release, installed GTSAM 4.3.1
with TBB, sdformat 15.4, Python 3.12.6 in the py312 Conda environment. Cable and
Python support were enabled; Boost serialization was disabled locally.

The library, cable kinodynamics example, generated cable/Python wrappers and
Python stubs build successfully. `gtsam` and the newly built `gtdynamics` import
together. All six cable prior template bindings return the expected values.

These focused C++ targets pass via `make -j6 testNAME.run`:
`testCableRobotUtils`, `testConstrainedOptBenchmark`, `testCableAccelerationFactor`,
`testWinchFactor`, `testCableTensionFactor`, `testCollocationFactor`,
`testConstraintManifold`, and `testCustomWrap`.

The bounded smoke command is:

```sh
build/examples/example_constraint_manifold/example_constraint_manifold_cable_kinodynamics \
  --num-steps 8 --methods all --benchmark-id cable_smoke \
  --benchmark-csv /tmp/cable_smoke.csv
```

It completed in about 1.1 seconds locally. Timing is informational; mathematical
regressions use residuals and dimensions, not wall-clock thresholds.

| Method | Equality L2 residual | Original objective |
| --- | ---: | ---: |
| Soft constraint | 5.4156e-2 | 1.036703 |
| Penalty | 1.7279e-3 | 1.900154 |
| Augmented Lagrangian | 3.4225e-3 | 1.853503 |
| CM(F) | 6.7685e-11 | 4.176597 |
| CM(I), no final projection | 6.7683e-11 | 4.176598 |

The unconstrained initial objective is 7.142857. Lower objective values from the
soft/penalty/AL rows come with larger equality violations and are not evidence
of equivalent feasible solutions. The shared benchmark reports iteration counts
as zero for penalty/AL/CM because those counters are not currently collected;
zero does not mean no iterations occurred.

The existing Python cable suite ran 11 tests: three passed, one failed, and seven
were already skipped. `test_cdpr_planar.TestCdprPlanar.testKinematics` fails its
FK Values comparison at tolerance 1e-5 (pose/twist discrepancies around 1e-4).
Those legacy Python tests and controllers are unchanged. The pre-existing skips
cover unfinished controller/simulator/parser APIs; this migration does not claim
to repair or validate them. Clang Debug CI covers the restored C++ experiment;
standard Release CI retains its existing Python configuration.

The default 1000-sample benchmark has not been run to completion. This is a
migration of a numerical experiment, not a hardware controller: the existing
cable acceleration model omits rotational effects, gravity is zero, tensions are
unbounded, and no physical feasibility or global optimum is claimed.

## Historical #331 file inventory

All paths below are relative to `gtdynamics/cablerobot/`. Their source net changes
are deliberately not replayed over current master. The wrapper receives only the
small compatibility correction described above.

```text
CMakeLists.txt
cablerobot.i
factors/CableAccelerationFactor.cpp
factors/CableAccelerationFactor.h
factors/WinchFactor.h
src/cdpr_controller.py
src/cdpr_controller_ilqr.py
src/cdpr_controller_tension_dist.py
src/cdpr_planar.py
src/cdpr_planar_controller.py
src/cdpr_planar_sim.py
src/data/.gitattributes
src/data/ATL.h
src/data/test_traj.h
src/draw_cdpr.py
src/draw_controller.py
src/gerry00_planar_sim.py
src/gerry01_planar_tracking.ipynb
src/gerry01_planar_tracking.py
src/gerry02_traj_tracking.ipynb
src/gerry02_traj_tracking.py
src/gerry03_stroke_tracking.ipynb
src/gerry03_stroke_tracking.py
src/gerry04_lqr_style.py
src/paint_parse.py
src/test_cdpr_controller_ilqr.py
src/test_cdpr_controller_tension_dist.py
src/test_cdpr_planar.py
src/test_cdpr_planar_sim.py
src/test_paint_parse.py
src/utils.py
tests/testCableAccelerationFactor.cpp
tests/testCustomWrap.cpp
tests/testWinchFactor.cpp
utils/CMakeLists.txt
utils/CustomWrap.cpp
utils/CustomWrap.h
```
