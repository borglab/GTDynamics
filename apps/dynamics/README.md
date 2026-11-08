# Dynamics

The Dynamics app displays estimated accelerations, ground reaction wrenches, and
all 29 G1 joint torques for the SSIO 78–98 s walking/turn snippet. Native CMC-Opt
solves the estimation problem offline. The browser replays body transforms,
forces, and torques at the saved timestamps.

The development window contains **57 original double-support knots** from the
best 715-state Kinematics trajectory. The Gaussian-process fit uses all those
knots and 10,996 dense encoder samples before cropping the window. Knot poses
and joint configurations remain exact. The Pose3/vector WNOA model in
`FloatingBaseInterpolator` fits body twists and joint rates; saved world COM
velocities provide additional observations. The same reusable GP implementation
supplies the Interpolation app.

Dynamics keeps these configurations and velocities fixed through
`ConstVarFactor`. Its variables include every link's body-twist derivative,
every joint acceleration, reaction wrenches, one external wrench per sole, and
actuator torques. Native `TwistAccelFactor` constraints enforce articulated
acceleration kinematics exactly. Rotor armature multiplies the **estimated**
joint acceleration in the motor-torque equality. All remaining equations are
affine, so CMC-Opt retains the elimination basis and cached affine
`ReducedALRetractor`.

Acceleration observations constrain the fitted velocity signal locally. At a
knot, the prior mean is

```text
a_hat(t) = [v_GP(t + h) - v_GP(t - h)] / (2 h),  h = 20 ms by default.
```

The stencil uses the `[-1, 0, 1]` differentiation mask. Its queries stay inside
the complete GP fit; the full trajectory's endpoints use one-sided differences
and the actual elapsed time. The development-window endpoints retain context
from the complete fit. Root and joint acceleration priors are soft; exact
force/moment balance and wrench inequalities can adjust their values.

| Acceleration prior | Per-axis standard deviation |
| --- | --- |
| Root angular body-twist derivative | 90 °/s² |
| Root linear body-twist derivative | 3 m/s² |
| Revolute-joint acceleration | 100 °/s² |

The root pose maps its inertial COM frame into world. Twists and their
derivatives are angular-first and expressed in the moving COM axes. The linear
part of a twist derivative is `d(v_body)/dt`; physical COM acceleration in body
axes is `d(v_body)/dt + omega_body × v_body`. Rotational prior scales are
specified in degrees and converted explicitly at the native factor boundary.
The recording retains both the acceleration estimates and their prior means.

The contact model uses one torque-first sole-local wrench per foot. Exact
inequalities enforce compression, an inscribed friction pyramid, rectangular
center-of-pressure bounds, and coupled yaw bounds. The original source phase
sequence preserves lift-off and recontact across unsampled intervals. Wrench-rate
priors connect observations belonging to the same uninterrupted stance.
Torque-rate costs use the actual irregular time intervals. Effort limits are
reported as diagnostics; hard effort bounds remain available in the native API.

The regenerated recording satisfies exact floating-base balance and contact
wrench inequalities at all 57 knots. CMC-Opt converged in three iterations,
taking approximately 0.75 s for native graph construction and optimization.
GP factor construction and elimination took another 0.18 s, excluding data
loading, FK sampling, and export. The result has 2,337 independent coordinates.
The error budget includes the acceleration observation costs. Per-component
RMS corrections from the GP-derived priors are approximately 1.20 °/s² for root
angular acceleration, 0.50 m/s² for root linear acceleration, and 0.39 °/s² for
joint accelerations.

Independent MuJoCo inverse dynamics agrees with the native required force,
moment, and all joint torques within 2e-12 N or Nm. Resultant-wrench and
four-corner LP feasibility checks agree at every knot. These checks establish
equation and frame consistency. Contact stationarity remains an input-quality
issue: the labeled stance feet have 0.67 m/s RMS speed in the GP fit. Rigid
contact would require further pose/velocity consistency constraints.

## Affine equality-only showcase

The conditioning boundary is deliberate: fit the full kinematic GP first,
then capture an owning q/v snapshot with `ConstVarGraph`, then create runtime
manifold types with `affineEqualities = true`. Accelerations remain free with
soft GP-derived priors. Every slice gets its own conditioned coefficients and
offset; equal graph structure alone does not permit sharing its numerical
basis. A changed q/v snapshot creates fresh types. LM damping and objective
weights never change that equality geometry.

`solveFixedKinematicsDynamics` exposes this workflow through
`fixed_dynamics::Options`. Set `equalityOnly = true` to compare equality-only
optimization, and choose `LinearSolver::Gaussian`, `GtsamQP`, or optional
`ProxQP`. `Automatic` uses Gaussian only for equality-only problems; the app's
normal inequality-constrained solve still uses GTSAM QP. The Python example
accepts the corresponding `--equality-only --linear-solver gaussian` flags.
This reuses the existing conditioning implementation; a dedicated generated
mechanics/conditioning API remains future work.

The [comparison runner](../../examples/g1_mjlab/benchmark_fixed_dynamics.py)
uses exactly this app's inputs, acceleration priors, objective, and anchor for
all solvers. Its private old-CM adapter calls the actual `LMManifoldOptimizer`
and `ConstraintManifold`, using the production problem builders. It keeps old
CM dependencies out of the public API. Run with matching built bindings:

```bash
taskset -c 8-15 python examples/g1_mjlab/benchmark_fixed_dynamics.py \
  --kinematics apps/kinematics-inspector/public \
  --dense-data apps/kinematics-inspector/.cache/openio-contact-source \
  --model-cache models/mujoco/.cache/g1-mjlab \
  --output build/affine-dynamics-comparison --include-proxqp
```

Omit `--include-proxqp` without that optional backend. Choose available CPUs
for your machine. The runner restricts each fresh process to eight inherited
CPUs, alternates case order, and saves input, source, binary, and library hashes.
Use single-thread BLAS/OpenMP and the same TBB-enabled GTSAM/GTDynamics build.
It writes benchmark artifacts only; it does not export browser recordings.

On 2026-11-08, three fresh runs per case used Release Clang 18.1.3, TBB,
ProxSuite 0.7.3, one retraction worker, CPUs 8–15, and single-thread BLAS/OpenMP.
GTSAM revision was `c940b44158d8bfa7394da233dd63aa225bc8fed2`, matching the
[nonlinear comparison](../../gtdynamics/cmcopt/EqualityOnlyOptimization.md).
Another benchmark was confined to CPUs 0–7; no builds/tests ran during timing.
These are local wall times, with some host variability.

| Solve | Median complete native call (s) | LM iterations | Final objective |
| --- | ---: | ---: | ---: |
| CMC equality-only Gaussian | 0.616 | 4 | 13.100276284397 |
| CMC equality-only GTSAM QP | 0.685 | 4 | 13.100276284397 |
| Old equality-only CM-Opt | 0.802 | 4 | 13.100276284397 |
| CMC equality-only ProxQP | 2.678 | 19 | 13.100276284398 |
| Normal CMC with wrench inequalities | 0.678 | 4 | 13.100276284397 |

The complete native call includes assembly, conditioning, equality-anchor
construction, manifold preparation, optimization, diagnostics, and cleanup.
It excludes data loading, GP fitting, fixed-input preparation, and export.
GP construction/solve takes about another 0.20 s. Old CM's native call includes
its adapter preparation, so the whole-call comparison does not isolate the LM
algorithm. New/old LM-stage timers also differ in teardown scope; use the
complete native-call column above for comparisons.

Gaussian is about 23% faster than the old baseline on this workload. Its
median linear-step time is 0.061 s versus 0.115 s for GTSAM QP, while retraction
remains about 0.21 s for both. There are 2,337 independent coordinates and
23,826 conditioned scalar dimensions. All CMC cases prepare and numerically
construct exactly 57 equality bases, including ProxQP's longer run. The
constant affine geometry is reused throughout LM.

Old CM, Gaussian, and GTSAM QP agree to 5.7e-14 N in sole wrenches and
1.3e-14 Nm in torques. Equality violation L2 stays below 2.2e-12 for all cases;
root force/moment residual norms stay below 1.5e-13 N/Nm. ProxQP reaches an
objective within 5.7e-13, with maximum wrench/torque differences of 1.27e-4 N
and 1.25e-5 Nm. Its existing settings take 35 QP attempts for 19 accepted LM
iterations, versus four Gaussian/GTSAM attempts. Its longer trajectory and
rejected trials account for much of the extra time; these measurements do not
identify the reason for each rejection. No solver tolerances or QP algorithms
were changed for this comparison.

The relaxed optimum happens to satisfy all 1,938 omitted inequality rows here,
so it agrees with the constrained reference. This is not guaranteed: regression
tests deliberately exercise infeasible omitted bounds. The equality-only mode
must be selected explicitly and always reports the original bounds separately.
An independent MuJoCo inverse-dynamics validation of the new Gaussian solution
agrees within 6.4e-13 N in force, 1.7e-12 Nm in moment, and 1.9e-12 Nm in joint
torques. Independent resultant-wrench and four-corner LP checks agree and find
all 57 slices feasible. This validation uses a temporary export, leaving the
checked-in recording intact.

The 17 affected native suites, 12 G1 Python tests, and four ProxQP-disabled
native suites pass. Raw runs, state differences, and the independent validation
are in `build/affine-dynamics-isolated/`.
The earlier overlapping-core preflight campaign is excluded.

Together with the nonlinear equality benchmark, this establishes numerical
agreement with old CM and useful direct-Gaussian performance for both kinds of
equality geometry. Old CM remains available pending broader migration coverage.
The checked-in browser recording and its historical timings above are unchanged.

## Regenerate

The computational Python example is
[`dynamics_walk.py`](../../examples/g1_mjlab/dynamics_walk.py). Start with
`solve_walk()`: it fits the full-trajectory GP, prepares fixed poses and
velocities with acceleration prior means, constructs the sole contact models,
and calls `gtd.solveFixedKinematicsDynamics()`. Its runnable `main()` prints
convergence diagnostics and optionally saves `dynamics.npz` and `summary.json`.
The [example README](../../examples/g1_mjlab/README.md#walking-dynamics-example)
documents that command and the saved arrays' units and ordering.

The browser adapter is [scripts/export_recording.py](scripts/export_recording.py).
It calls the same example and writes `recording.json.gz` (the solution and
browser diagnostics) and `model.json.gz` (display geometry) to `--output`, which
defaults to `apps/dynamics/public`.

Use the pinned G1 packages and matching GTDynamics/GTSAM Python bindings:

```bash
conda activate py312
python -m pip install -r examples/g1_mjlab/requirements-dynamics.txt
make -C build -j6 gtdynamics_py
PYTHONPATH="$PWD/build/python:$HOME/git/gtsam/build/python:$PWD/examples/g1_mjlab" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
python apps/dynamics/scripts/export_recording.py \
  --kinematics apps/kinematics-inspector/public \
  --dense-data apps/kinematics-inspector/.cache/openio-contact-source \
  --start 78 --end 98 --friction 0.8 --max-iterations 50
```

The exporter selects `comparison.json`'s winner automatically. The
`--acceleration-step-ms`, `--base-angular-acceleration-sigma-degrees`,
`--base-linear-acceleration-sigma`, and `--joint-acceleration-sigma-degrees`
arguments configure the acceleration observations. GP noise assumptions,
source fingerprints, factor errors, corrections, and timing scopes are saved
with the replay.

The default solve enforces exact floating-base balance. The explicit
`--soft-base-balance` option permits a residual wrench and publishes its force
and moment. Its per-axis scales are controlled by `--base-force-sigma` and
`--base-moment-sigma`. Configuration and velocity inputs remain fixed.

## Validate

With the same Python environment and `PYTHONPATH`:

```bash
make -C build -j6 testFixedKinematicsDynamics.run testFloatingBaseInterpolator.run
python -m unittest discover -s examples/g1_mjlab -p test_fixed_kinematics.py -v
python -m unittest discover -s examples/g1_mjlab -p test_sole_support.py -v
python apps/dynamics/scripts/validate_recording.py
cd apps
npm test --workspace dynamics
npm run build
```

The MuJoCo oracle uses the **published optimized accelerations**, disables
contact constraints and passive damping/friction, and subtracts sole-wrench
loading when comparing actuator torques. GTSAM `LpProblem` programs check
resultant-wrench and nonnegative corner-force feasibility. They keep balance
exact and minimize a nonnegative slack on the cone inequalities; zero slack
establishes feasibility. Solver failures propagate separately from infeasible
loads. The native CMC result supplies every published force and torque.

The support rectangle uses a two-dimensional monotone-chain hull of the sampled
capsule footprints. This preserves the previous rectangle and 5% margin without
a SciPy dependency. Footprint tests live alongside the example. With the native
Python environment active, run `npm run test:native --workspace dynamics` from
`apps/` for feasible/infeasible-load checks.

The browser selects the nearest retained knot during playback. Connecting lines
between force/torque samples guide the eye across unsampled intervals. Compiled
canonical-model geometry and the replay use Git LFS. Model and SSIO attribution
remain available below the viewer. Browser checks are
`node dynamics/tests/app.browser.mjs` and
`node dynamics/tests/navigation.browser.mjs`, with `PLAYWRIGHT_MODULE` set when
Playwright is installed outside the npm workspace.
