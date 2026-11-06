# Kinodynamics

The app replays a coupled CMC-Opt solve on sparse support knots from the SSIO
G1 mid-clip turns (78–98 s). It estimates configurations, body/world velocities,
accelerations, sole wrenches, internal reaction wrenches, and all 29 actuator
torques. The default `--initialization contact-gp-dynamics` runs the native
contact → GP → affine linear dynamics seed pipeline. `--initialization contact`
retains contact poses with zero motion and forces; `--initialization fk` uses
measured-joint FK at a fixed upright pelvis and gravity-support forces.

Initialization and coupled optimization both retain 171 source slices: 114 in
single support and 57 in double support. Short phases contribute a central slice;
long phases use
one-sample-buffered slices at most 200 ms apart. Contact poses supply GP knots,
retained encoders constrain velocities, and the dynamics initializer estimates
accelerations, wrenches and torques at those same knots. Selection occurs before
variable and factor assembly. The 1,000 source samples never become a dense
kinodynamic optimization problem. Original irregular timestamps determine IMU
preintegration, collocation, and rate costs. Every supported phase is represented,
so a stance landmark persists across phase boundaries and resets after lift-off.
Raw and optimized replay show the retained knots without synthesizing dense
force or torque estimates.

The component coordinate count falls from 106,584 to 18,297. Shared contact poses
and bias add 252 coordinates to each complete QP (18,549 in the sparse solve).

The computation is the student-facing example
[`kinodynamic_walk.py`](../../examples/g1_mjlab/kinodynamic_walk.py). Run from the
repository root with the built GTDynamics/GTSAM Python packages and pinned G1
model packages on `PYTHONPATH`. Enable `GTDYNAMICS_BUILD_G1_MJLAB` and
`GTDYNAMICS_BUILD_PYTHON`, then build `_g1_kinodynamics`. Add
`build/examples/g1_mjlab` alongside `build/python` and the matching GTSAM Python
path. The private extension is built with the standard wrapper’s pybind11
provider and is not installed into the GTDynamics package:

```bash
python examples/g1_mjlab/kinodynamic_walk.py --output /tmp/kinodynamics
python examples/g1_mjlab/kinodynamic_walk.py --initialization contact --output /tmp/kinodynamics-contact
python examples/g1_mjlab/kinodynamic_walk.py --initialization fk --output /tmp/kinodynamics-fk
python apps/kinodynamics/scripts/export_recording.py --result /tmp/kinodynamics \
  --comparison /tmp/kinodynamics-contact/summary.json \
  --comparison /tmp/kinodynamics-fk/summary.json
cd apps
npm run build
```

The example defaults to five iterations and accepts
`--initialization fk|contact|contact-gp-dynamics`, `--max-iterations`, `--start`, `--end`, `--data`, `--labels`, and
`--retraction-threads` (8 by default; 1 runs projections serially).
It defaults to GTSAM's feasible-step API with projected-gradient initialization
disabled. Use `--qp-step full` for a converged QP and
`--projected-gradient-initialization` to enable seeding in either mode.
`--qp-ordering temporal` passes our chronological component ordering, retiring
each stance landmark at its last observation and keeping the bias until the end.
`--qp-ordering automatic` (default) lets GTSAM use COLAMD. The supplied order
is recorded as decimal key strings in `qpOrderingKeys`.
The selected options are recorded in `solverSettings`; `qpMaxIterations` remains
1,000. The linked GTSAM build must provide `computeFeasibleStep()` and
`useProjectedGradientInitialization`, and `setOrdering()`, including matching
Python bindings.
Its main saves numerical arrays and a JSON report. The exporter converts those
arrays to offline browser assets and adds independently compiled model geometry.
No optimization runs in the browser. All recording/model gzip files use Git LFS.

Articulated q/v/a kinematics, Newton–Euler balance, motor armature, reaction
wrench transmission, sole transforms, zero swing wrenches, and rectangular wrench
cones are exact constraints. Encoder, stance-pose landmark, Gal3 IMU, stance
velocity, trapezoidal configuration/velocity collocation, effort/rate and broad
acceleration terms are costs. The default warm start uses the native floating-base GP.
No upright or ground-height costs remain in the coupled solve. The contact initializer
uses its shared upright/motion/height priors. At the first selected supported
slice, translation and wrapped-yaw costs choose the world frame with
standard deviations of 1 m and approximately 57.3°. All six root-pose coordinates remain free
in the articulated tangent basis, allowing gravity/IMU observations to correct
the initial roll and pitch.
The initialized configurations and velocities become free variables in the
coupled solve.

CMC-Opt eliminates each constraint-connected component's linearized equalities
through its basis B. The coupled linear step solves the reduced objective and
active-boundary inequalities with GTSAM's sparse QP solver. Nonlinear retractions
remain local to each component.
CMC-Opt relinearizes the full nonlinear equality basis. Each retraction recovers
link poses/twists with native FK, fixes those q/v values through ConstVarFactor,
and projects the remaining affine acceleration/wrench equations with Gaussian QR
and ReducedALRetractor. Time blocks are eliminated chronologically, retiring each
stance landmark after its last observation and the shared bias at the end.
Per-slice projections run in a bounded TBB arena when available; serial and
parallel results are checked against each other. One previous iteration is retained for diagnostics. Compact boundary residuals
and active sets track the mode independently of diagnostic-history retention. The app stores scalar per-factor error
histories instead.

Timing separates contact fitting, GP interpolation, the linear dynamics seed,
graph assembly, initial feasibility,
and coupled optimization. Input/model loading, array export,
compression and browser rendering are excluded from the reported solver total.
Measurement construction and IMU preintegration are included in the total.
Feasibility restoration changes the raw initial dynamics and is included in total.
A feasible result or an objective-change stopping condition does not establish
that the objective is globally optimal.

The saved sparse replay predates the projected-gradient and feasible-step APIs.
It is one five-iteration run with TBB enabled and eight
retraction workers. Initialization took 0.226 s, coupled optimization took
204.172 s, and initialization through optimization took 206.057 s. The objective
decreased from 7,341,612.101 to 8,931.760. Equality and inequality residual norms
are 1.76e-12 and 3.70e-8, respectively. The expensive second QP required 173
active-set iterations. The smaller variable count does not resolve the GTSAM
solver performance issue. The native binary/revision and modified Python
source fingerprints are included in the replay provenance.

The latest comparison uses GTSAM `e0fb6ad4f`, feasible steps, and projected-gradient
initialization disabled. Only `contact-gp-dynamics` was run, on the same 78–98 s
clip with 171 support slices and five outer iterations. Both modes use identical
Release binaries, TBB with CPUs 0–7, eight retraction workers,
single-thread BLAS/OpenMP, explicit LM damping and zero extra QP regularization.
Three fresh processes per ordering ran sequentially in automatic/temporal,
temporal/automatic, automatic/temporal order, after all builds and tests.

| Ordering | Optimization runs (s) | Median optimization (s) | Median initialization through optimization (s) | Final objective |
| --- | --- | ---: | ---: | ---: |
| Automatic COLAMD | 9.1473, 9.1284, 9.0903 | 9.1284 | 11.1249 | 9,272.533830 |
| Temporal, passed to GTSAM | 8.8833, 8.8170, 8.8250 | 8.8250 | 10.8266 | 9,272.533830 |

Passing the temporal ordering saves 3.32% of optimization time (1.034x).
Each run attempts five QP working-face solves total, with no rejected LM trials.
All runs are feasible: equality norms are at most 2.00e-12 and inequality norms
at most 5.09e-8. Independent MuJoCo inverse dynamics, sole-cone and swing-wrench
checks pass. Ordering changes the objective by only 4.0e-11; the maximum pose
matrix difference is 8.53e-14, wrench difference 1.24e-11 and joint-torque
difference 2.85e-12. The temporal order contains all 213 reduced/free keys,
with landmarks retiring at their last observation and bias retained until the end.

The initializer change affects the optimization policy: disabling projected
gradients keeps all tight rows on the starting face, and feasible-step mode
returns without multiplier-based releases. The earlier projected-gradient run
below reached objective 9,000.94 after five iterations, whereas these runs reach
9,272.53. That historical run also used an older GTSAM revision; it is not a
matched measurement of the initializer alone. The ordering comparison above
has identical settings apart from `qpUseTemporalOrdering`.

The recorded Python processes loaded Conda's `libtbb.so.12.13` and
`libtbbmalloc.so.2.13`, despite the build selecting system TBB 2021.11. Their
actual paths and hashes match across both orderings. The master audit below
pins system TBB explicitly, avoiding this loader-dependent difference.

Raw reports, arrays, actual ordered keys, library/model hashes, validation and
`comparison.json` are local in `build/temporal-qp-comparison/`. Reproduce the two
modes using `--qp-step feasible --no-projected-gradient-initialization` and
`--qp-ordering automatic` or `--qp-ordering temporal`. GTDynamics forwards
`lmParams.ordering` after dropping zero-dimensional component keys, with GTSAM
validating the remaining permutation. `testIELMQp`, `testKinodynamicGauge`, the
four affected manifold/retractor suites, and seven G1 Python tests pass. The
GTSAM options, feasible-step, QP, LP and QP-problem suites also pass.

Earlier matched local runs on 2026-11-05 link GTSAM `e7f6b7774` with incremental sparse
elimination and projected-gradient initialization. Each run uses the same
78–98 s clip, 171 retained support slices, five outer iterations, LM settings,
and zero additional QP regularization. Both libraries are Release TBB builds with
affinity CPUs 0–7, eight retraction workers, and single-thread
BLAS/OpenMP. Runs use fresh processes sequentially, without concurrent builds.

| Initialization | Full QP optimization (s) | Feasible-step optimization (s) | Full QP objective | Feasible-step objective |
| --- | ---: | ---: | ---: | ---: |
| `contact-gp-dynamics` | 87.78 | 67.47 | 8,931.76 | 9,000.94 |
| `contact` | 170.01 | 108.74 | 8,931.49 | 8,948.17 |
| `fk` | 274.26 | 192.15 | 26,482.39 | 26,469.03 |

These are complete nonlinear optimization times, including QP initialization,
relinearization, retraction, LM retries and error-history recording. Initialization
through optimization totals are 89.78/69.48 s, 171.70/110.41 s and
275.92/193.84 s respectively (full/feasible). All six results are feasible:
equality norms are at most 1.87e-12 and inequality norms at most 9.44e-8.
They also pass the independent 21-sample MuJoCo inverse-dynamics check and
sole-cone/swing-wrench checks at every retained slice.

Feasible steps save 23%, 36% and 30% in these single matched comparisons.
Their five-iteration objectives change by +0.77%, +0.19% and -0.05%.
Neither stopping at five iterations nor obtaining a feasible QP step establishes
nonlinear optimality. Both FK runs have two rejected trials and damping retries.
Main QP iteration totals (full/feasible) are 243/167, 488/288 and 839/554;
they exclude auxiliary projection iterations. The default's second QP needs
111/41 main iterations, so feasible-step mode still spends time updating its
working set before returning useful descent.

Raw summaries, arrays, logs, matching library/model hashes and validation reports
are local in `build/new-qp-comparison/`; `comparison.json` checks that each pair
differs only in `qpUseFeasibleStep`. These runs do not update the saved browser
recording. The historical dense PR 472 timings below use different sampling.

## Matched audit against master

The 28.01 s optimization / 39.64 s total quoted from master used 1,000 optimized
samples. The current 8.83 s run uses 171 support knots, so those numbers do not
measure a regression on the same problem. A new audit builds master's native
solver from `ad64754e` (tree-identical to merged master `c88f3699`) in an isolated
build directory, using the current Python support-slice preparation. Both native
versions use the same GTSAM `e0fb6ad4f` binary, Clang Release flags, 171 slices,
five outer iterations, temporal ordering, measurements and damping. This audit
explicitly preloads system `libtbb.so.12.11` / `libtbbmalloc.so.2.11`, pins CPUs
0–7, uses eight retraction workers, and limits BLAS/OpenMP to one thread.

| Solver / diagnostic | Optimization (s) | Final objective |
| --- | ---: | ---: |
| Unchanged master algorithm on sparse inputs | 8.6581 | 8,980.796615 |
| Current generic QP, no projected initialization | 8.8808 | 9,272.533830 |
| Master with diagnostic all-tight face selection, sequential Cholesky | 5.1008 | 9,272.533830 |
| Same master diagnostic, sequential QR | 6.6905 | 9,272.533830 |
| Generic QP diagnostic, sequential QR | 8.6463 | 9,272.533830 |
| Generic QP diagnostic, sequential mixed QR/Cholesky | 8.2031 | 9,272.533830 |

These are single bounded runs, checked by a separate timing-probe pass. All
return feasible trajectories and pass independent MuJoCo dynamics and sole
checks. For the all-tight Cholesky and current generic paths, saved trajectories
agree to numerical precision; the original master row selects a different face
and consequently produces a lower objective. The diagnostics alter only face
selection or elimination in temporary copies/interposition libraries. They do
not restore a working-face implementation or change GTSAM's production backend.

The identical-face comparison locates the remaining overhead. In the instrumented
pass, master takes 5.0943 s and the generic path 8.8721 s. Total linear-step work
is 0.5734 s for master's local-face reduction/recovery plus unconstrained
Cholesky, versus 3.4416 s inside GTSAM's feasible-step calls (3.3446 s in sparse
subproblems). Six state constructions take 0.3416 s on master versus 1.2225 s
with global QP assembly. These two differences account for approximately 3.75 s
of the 3.78 s total gap.

Passing an ordering preserves variable order, but does not reproduce master's
linear algebra. Master removes active-face directions locally and then uses
sequential unconstrained Cholesky. The generic path retains exact active rows
and performs constrained QR through a multifrontal/Bayes-tree solve. Switching
that generic solve to sequential QR saves only about 0.23 s here; ordering or
sequential elimination alone does not recover the old cost. The next backend
optimization target is efficient exact-constraint elimination / an unconstrained
factorization after reduction inside GTSAM, plus reduced QP assembly overhead.
The diagnostic mixed QR/Cholesky result is not a robustness validation for other
problems and has not been enabled in production.

The step policy also differs. Master projects the gradient to select its first
face, then adds all violated blocks together until feasible; optional full QP
refinement is disabled. The current no-projection method retains all tight rows
on its initial face and returns feasible descent without multiplier-based row
release. Having a feasible-step API and matching ordering therefore does not
make the two algorithms equivalent. Matching master quality needs an appropriate
face-selection/update policy as well as a faster linear subproblem.

Raw pinned runs, excluded preliminary runs, timing probes, diagnostic source
copies, library hashes, arrays and validation are local in
`build/master-qp-audit/`. `comparison.json` contains only the pinned successful
runs and verifies matching input objective, sample count, GTSAM and TBB hashes.

## Historical dense runs

The following historical PR 472 dense runs (2026-11-05, Release build, eight
retraction workers) used
five coupled iterations on identical 1,000 source slices at 50 Hz. Contact fitting
retains 171 slices: 114 in single support and 57 in double support. Each strategy
ran once in a separate process with `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`.
All three runs are feasible.

| Initialization | Initialization (s) | Coupled optimization (s) | Total (s) | Final objective |
| --- | ---: | ---: | ---: | ---: |
| `contact-gp-dynamics (default)` | 1.245 | 28.01 | 39.64 | 9,074.082857 |
| `contact` | 0.074 | 32.97 | 42.75 | 9,047.406663 |
| `fk` | 0.034 | 37.36 | 47.19 | 67,966.670103 |

The default pipeline takes **1.245 s** to initialize and **39.64 s** through the
coupled solve. Its final objective is **9,074.082857** at the five-iteration limit.

| Initialization stage | Wall time (s) |
| --- | ---: |
| Contact-pose fit and sparse output | 0.045 |
| GP assembly, fitting and dense FK | 0.075 |
| Linear dynamics stage, including temporary-state cleanup | 1.123 |
| ↳ Mechanics graph assembly | 0.346 |
| ↳ Equality-system linearization and assembly | 0.142 |
| ↳ Constrained Gaussian QR | 0.330 |
| ↳ Dynamics-variable reconstruction | 0.064 |
| ↳ Mechanics/wrench checks and acceleration-prior error | 0.108 |

The indented rows are substages of linear dynamics. The stage total also includes
GP-velocity differentiation, merging the kinematic state, world-velocity
construction and destruction of temporary graphs/maps. QR time excludes those
operations. Dynamics initialization performs zero CMC iterations or manifold
projections. Effort and temporal-cost graphs are constructed only for full
optimization; the seed reports acceleration-prior error. The combined pipeline
owns one state map and separate stage diagnostics. Contact root poses feed GP
knots directly, and dense FK is evaluated after the GP fit. The GP uses no
external root-velocity observations.

`ConstVarFactor::setFixedValues` visits only its own fixed keys. Equality assembly
reuses freshly linearized Jacobians with constrained noise models. The later
coupled solve optimizes configurations, velocities, accelerations, forces and
torques. These bounded runs do not establish convergence or cross-machine speedups.

`python examples/g1_mjlab/validate_kinodynamics.py --result /tmp/kinodynamics`
checks independent MuJoCo inverse dynamics on 21 slices and sole feasibility
at every slice. The saved replay includes these diagnostics and records the
configured five-iteration limit; feasibility does not establish convergence.

The replay includes source/binary fingerprints, projection settings and gauge
metadata. The [screenshot](docs/kinodynamics.png) shows this replay.

The historical table above refers to the dense formulation; it is not a runtime
claim for the sparse QP solver. The app's three-strategy table uses the summaries
embedded in its current recording, when matching comparison runs are supplied.
The current sparse replay contains only the default run; the historical dense
comparison remains above. Only the default's
full initial/final replay is stored; the contact-only and FK rows contain summary
statistics. Totals include initialization, measurement construction, IMU
preintegration, graph assembly, initial feasibility and coupled optimization.
Input/model loading, output serialization, compression and browser rendering
are excluded.

The page follows the shared technical-paper layout. Its abstract and complete
replay controls fit together on a laptop screen. Motivation, a synchronized
factor-graph schematic and native factor definitions precede the recorded
timings, force/velocity/torque plots, error budget and numerical settings.
Equations use the shared build-time MathJax plugin and work offline. Parameter
tables read the selected recording, including the sole support rectangles,
collocation weights, IMU assumptions and gauge. The Discussion links to Motion
Planning, which remains the final tab.

The support schedule is privileged: it uses saved max-product labels derived
from SuperOdom LiDAR-inertial foot motion and measured joint velocities. Reference
poses after initialization do not enter the coupled estimation graph. The inferred
forces and torques are model-based estimates. Dataset/model attribution is
available in the app and accompanying notices.
