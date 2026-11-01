# Whole-walk kinematic estimation with CM-Opt

[`kinematic_walk.py`](kinematic_walk.py) is a Python example of fitting the
complete recorded G1 walking snippet. Read `load_walk`, `build_problem`,
`initialize`, `solve_cm`, and `evaluate` in that order. The default is all 800 samples at
100 Hz, covering 7.99 seconds between the first and last states.

## Build and run

Use a Python environment with NumPy, MuJoCo, and the GTSAM Python bindings
built against the same GTSAM library as GTDynamics. Build the standard
`gtdynamics` package with that interpreter; all needed APIs are declared in
`gtdynamics.i`.

From the repository root (adjust the interpreter and GTSAM build paths):

```bash
PYTHON=/path/to/python
PYTHON_VERSION=$("$PYTHON" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DGTDYNAMICS_BUILD_PYTHON=ON \
  -DWRAP_PYTHON_VERSION="$PYTHON_VERSION" -DPYTHON_EXECUTABLE="$PYTHON"
cmake --build build --target gtdynamics_py -j2

# Omit the GTSAM build path if a matching GTSAM package is already installed.
export PYTHONPATH="$PWD/build/python:/path/to/gtsam/build/python:$PYTHONPATH"

# One full solve, using contact initialization by default.
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 "$PYTHON" \
  examples/g1_mjlab/kinematic_walk.py

# Run the example with inverse-kinematics initialization.
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 "$PYTHON" \
  examples/g1_mjlab/kinematic_walk.py --initializer ik

# The separate timing driver calls the same example functions.
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 "$PYTHON" \
  examples/g1_mjlab/compare_kinematic_walk.py --repeats 3 \
  --output examples/g1_mjlab/results/kinematic-walk-timings.json

"$PYTHON" -m unittest discover -s python/tests -p test_kinematics_wrapping.py -v
"$PYTHON" examples/g1_mjlab/test_kinematic_walk.py
```

[`mujoco_io.py`](mujoco_io.py) owns recording loading, static MuJoCo geometry
extraction, conversion through the existing wrapped `Link`, `RevoluteJoint`,
and `Robot` constructors, and the MuJoCo FK oracle. `full_model.py`,
`four_way_compare.py`, and `validate_full.py` share these loading utilities.
The dynamics exporters can still run in the MJLab environment without GTSAM;
only the kinematic conversion functions import it.

The walking driver uses standard `gtsam.Values.extract()`, pose factors, noise
models, and GTDynamics key/FK utilities. `gtdynamics.i` exposes the missing
kinematic graph builders, contact initializer types, and CM-Opt interfaces;
solver configuration is visible in the example's `solve_cm` function.

The example reuses local `results/model.mjb`, `results/walk.npz`, and
`results/walk.json`; see [recording instructions](README.md#reproduce).
The contact labels come from the checked-in
[G1 contact dataset](../../apps/contact-initialization/public/g1-walk.json).
The loader verifies that its encoder samples match the recording and checks
GTDynamics COM-frame forward kinematics against MuJoCo at three samples.
A newly recorded or different walk needs a matching `--contacts` dataset.
`--max-frames 12` is a diagnostic prefix; it is never the default benchmark.

## Estimating only at contact-initializer slices

For irregular contact-selected sampling, initialize before building the graph:

```python
source = load_walk(data, contacts_path)
walk, seed, initialization = initialize_contact_slices(source, contact_buffer=1)
costs, constraints, keys = build_problem(walk)
seed = seed.extract(keys)
solution, solver = solve_cm(costs, constraints, seed, walk.root.id(), 10)
```

The native `contact_init.Result.slices()` API returns the exact source slices
used by `Initializer.InitializeSolutionContacts`, in pose order, excluding
dense interpolated outputs. Pass a `Result` as `diagnostics` and `contactBuffer=1`
to that existing call. Slice indices retain their original values; timestamps
are `slice.k * dt`, also available as `result.graph.measurements[i].time`.
Each continuous foot stance must retain two slices after trimming both ends.
The initializer also selects phase boundaries where a trusted contact remains
and supplemental slices in long stances, at intervals of at most one second.

`initialize_contact_slices` reindexes those states into contiguous solver keys
and constructs the reduced `Trajectory` from the source phases. It preserves
`walk.source_indices` for reference alignment and `walk.times` in seconds from
the recording start. The smoother keeps source contact specifications; it
removes stationary-contact edges across lift-offs skipped by sparse sampling.
The seed is the native solution at those exact slices, not another initializer
run. Its first retained pose uses the original initial gauge; no later reference
pose enters estimation.

`imu_aiding.build_imu_aiding` integrates one `pim` between each pair of actual
`walk.times`, clipping native IMU intervals at both ends. Velocity seeds use
finite differences at those same times. Stationary-contact sigma remains a
displacement sigma per selected interval, not a velocity sigma. Contact vs IK
continues using uniform snippet sampling; the Kinematics app uses this reduced
trajectory workflow.

## Graph construction

Each state contains 30 link COM poses and 29 scalar joint angles. Contact
changes partition the samples into 75 disjoint, inclusive `Phase` intervals.
Each phase carries a `FootContactConstraintSpec`; `WalkCycle` combines them and
`Trajectory` supplies the full time context. There is no duplicate boundary
slice. A continuing stance remains linked when the other foot changes contact.

`Kinematics.constraints(trajectory, robot)` creates the 29 joint pose relations
at every state. These are the hard equalities. The cost graph contains:

- Encoder joint-angle objectives, built through the trajectory API (sigma
  0.03 rad).
- Sole-height objectives built per phase from its contact specification,
  covering every slice in the phase (sigma 0.01 m).
- `Kinematics.contactEqualityFactors(trajectory, model)` penalizes each continuing
  ground contact point's world displacement between successive samples, including
  across phase boundaries (isotropic sigma 0.005 m per point). These reuse
  `ContactEqualityFactor`; a single point permits rotation about that point,
  while multiple sole points collectively resist rotation. Non-ground contacts
  contribute no factors, and non-ground intervals break ground-contact continuity.
- Weak adjacent-root smoothness costs (0.3 rad and 0.1 m), including contact
  transitions and the initially unsupported sample.
- A common first-root-pose prior to fix the world gauge (sigma 1e-5).

Contact observations are soft because the compliant recorded walk has moving,
rolling soles. The hard equalities encode the articulated robot geometry;
there are no inequality constraints, friction cones, force variables, or
acceleration variables in this example.

Both comparisons optimize this **same whole-trajectory graph** with CM-Opt.
Its equality-connected components are individual articulated configurations;
the inter-slice cost factors couple them into one optimization. CM-Opt uses an
elimination basis with root pose and joint angles as coordinates and the native
`BasisRetractor`. It does not replace the graph with hand-written FK Jacobians.

## What the two initializers know

The contact path calls `Initializer.InitializeSolutionContacts` with measured
joint angles and the same `Trajectory` used by the final graph. The initializer
reads each phase's contact specification. The same point on the same link shares
a landmark across consecutive phases; a point that returns after contact loss
gets a fresh landmark. The example
keeps no separate episode schedule or `(phase, spec)` pairs; `Phase.constraintSpec()`
provides the contacts for IK and graph evaluation as well.

The initializer solves supported phase boundaries and supplemental frames,
then expands every trajectory slice through measured-joint FK. The first
recorded sample has no contact: the supplied initial root pose is held through
this prefix and anchors the first supported frame. Internal flight uses
interpolation; a trailing unsupported span holds the last solved root pose.
These are initialization assumptions, not contact observations. The contact
initializer no longer uses the commanded forward step for its initial gauge.

The IK path calls `Initializer.InitializeSolutionInverseKinematics` separately
for each phase because that API accepts one fixed contact set. It uses the
recorded command speed, 0.5 m/s, to specify a straight-line root target. Native
phase-local keys are remapped to global trajectory keys. A one-sample phase uses
a one-interval call and keeps only its first state. This avoids changing or
inventing a second IK algorithm for the comparison.

These initializer APIs have different input requirements: contact initialization
uses measured joints and phase contacts, while legacy IK starts from zero joint angles and a
commanded root path. Both final solves use all the same encoder and contact
objectives. Neither initializer or objective receives the later recorded root
poses; those are used only for model validation and error evaluation. The
comparison is of the two existing initialization workflows, including those
input differences.

## Reading the output

[`compare_kinematic_walk.py`](compare_kinematic_walk.py) alternates initializer
order across repetitions (`--repeats`, default 3). Every run creates
fresh seeds and a fresh CM-Opt optimizer. The same prebuilt graph is shared.
There is no warm-start between repetitions. Medians include all requested runs;
an infeasible result causes a nonzero exit and remains in the saved diagnostics.
Iteration-limit flags are also recorded; small equality error alone does not
prove convergence to an optimum.

Initialization timing includes Python assembly, native IK/contact optimization,
key remapping, and dense FK expansion. Solve timing includes the CM-Opt problem
transformation, initial feasibility projection, LM, and recovery of the original
variables. Loading, shared graph construction, and residual evaluation are
reported separately or excluded. All wall-clock timers, repeated-run logic,
and benchmark provenance live in `compare_kinematic_walk.py`, which calls the
example's `build_problem`, `initialize`, `solve_cm`, and `evaluate` functions.

Inspect objective, equality residual, joint residual, stance height/slip, and
reference-root error alongside timings. This kinematics-only estimator has no
IMU, global position observations, or absolute heading measurements after its
initial gauge. Accumulated root drift and imperfect stance fit are expected;
the reference-root error is not used to tune either run.

The detailed JSON is an ignored local output. Compact measured results are
recorded below so the example stays small.

The soft contact displacement cost is configurable with
`--contact-slip-sigma` (metres per axis per adjacent slice interval; default
0.005). `build_problem(walk, contact_slip_sigma=...)` exposes the same setting.
This is a displacement sigma, not a velocity sigma, and is not scaled with sample
rate. The Open-IO app uses 0.05 to allow more stance slip during turns; its
[comparison](../../apps/kinematics-inspector/README.md#contact-slip) retains the
5/20/50 mm experiment. The MJLab timing results below use the original 0.005.

## Optional Gal3 IMU aiding

For recordings containing native IMU timestamps, `imu_aiding.py` constructs
GTSAM `PreintegratedImuMeasurementsG` objects (`pim`, or `pims` for a collection)
and calls the wrapped `Kinematics.imuFactors` trajectory builder. It adds one
world-frame COM velocity per slice and one shared sensor-frame bias, with explicit
velocity and bias priors. Existing `solve_cm` handles these additional variables.

Poses map COM coordinates into world coordinates. Velocity is the world-frame
linear velocity of that COM, not the six-dimensional body twist. The model's
`COM_T_IMU` transform is passed as `body_P_sensor`; specific force includes gravity
at rest. GTSAM applies rotation and centripetal lever-arm corrections but omits
the angular-acceleration lever-arm term. Native timestamps are integrated over
exact slice intervals, including phase boundaries and flight.

The [Kinematics app](../../apps/kinematics-inspector/README.md#imu-aiding) exports
and displays both aided and unaided solves for Open-IO. See its documentation for
noise assumptions, priors, regeneration commands, and measured limitations.
The timing comparison above continues to measure the kinematics-only problem.

## Measured whole-walk comparison (October 31, 2026)

### Current ground-contact equality factors

Three paired repetitions with the point-displacement objective, all 800 samples
and 75 phases, Release build, one OpenMP/OpenBLAS thread:

| Initialization | Initialization median (s) | CM-Opt median (s) | Total median (s) |
|---|---:|---:|---:|
| Contact initializer | 0.315 | 7.285 | 7.601 |
| Inverse kinematics initializer | 1.610 | 24.318 | 25.927 |

Contact initialization took 11 CM-Opt iterations; IK took 42. No run reached the
50-iteration limit. Their final objectives were 607.9554 and 607.9556,
respectively, with equality L2 below 1.9e-9. Both fits had approximately
0.00410 rad encoder RMS, 0.00355 m stance-height RMS, 0.00158 m per-step contact
slip RMS, and 0.346 m reference-root translation RMS.

The graph has 30,648 cost factors and 23,200 equalities; its 800 equality
components still reduce 167,200 ambient dimensions to 28,000 coordinates.
These results use a 0.005 m sigma on each ground point's displacement, replacing
the previous independently weighted foot rotation/translation costs. This is
a changed objective, not just a refactor, and the historical timings below
are not directly comparable. Detailed local output:
`results/kinematic-walk-contact-equality.json`.

### Historical whole-foot pose objectives

The following historical runs used whole-foot pose-between factors. Their
objectives and timings are not measurements of the current point-contact graph.

Historical result before migrating to the standard Python package: three
paired repetitions on this machine, Release build, one OpenMP/OpenBLAS thread,
all 800 samples and 75 contact-aligned phases:

| Initialization | Initialization median (s) | CM-Opt median (s) | Total median (s) |
|---|---:|---:|---:|
| Contact initializer | 0.301 | 6.908 | 7.209 |
| Inverse kinematics initializer | 1.156 | 3.297 | 4.452 |

Both methods took five CM-Opt iterations and reached objective 624.3698;
the relative objective difference was below 4e-8. The kinematic equality L2
residual stayed below 2e-9. There were 23,200 equality factors and 28,470 cost
factors. CM-Opt reduced 167,200 ambient dimensions to 28,000 coordinates,
organized into 800 components coupled by the temporal costs.

Both fits have approximately 0.00422 rad joint-measurement RMS, 0.00339 m
stance-height RMS, 0.00177 m per-step sole slip RMS, and 0.381 m reference-root
translation RMS. Contact initialization is faster, but its subsequent CM-Opt
solve is slower here, so the IK workflow has the lower total time. These are
measurements of this graph and recording, not a general initializer ranking.

Repeating the same three-pair benchmark through the standard `gtdynamics`
package after the refactor gave these medians (same machine and graph):

| Initialization | Initialization median (s) | CM-Opt median (s) | Total median (s) |
|---|---:|---:|---:|
| Contact initializer | 0.255 | 6.892 | 7.153 |
| Inverse kinematics initializer | 1.532 | 3.872 | 5.406 |

All six solves again took five iterations, reached objective 624.3698, and had
equality L2 below 1.3e-9. These timings include Python basis-key callbacks and
assembly through the public API; the objective and factor counts are unchanged.
The local detailed output is `results/kinematic-walk-public-bindings.json`.

With the trajectory-based initializer, three paired repetitions gave:

| Initialization | Initialization median (s) | CM-Opt median (s) | Total median (s) |
|---|---:|---:|---:|
| Contact initializer | 0.272 | 2.460 | 2.735 |
| Inverse kinematics initializer | 1.277 | 3.200 | 4.474 |

The contact initializer now selects 148 supported phase-boundary frames and
holds the supplied root pose through the unsupported prefix. Its CM-Opt solve
took four iterations; IK took five. Both reached objective 624.3698 with equality
L2 below 1.4e-9. The final graph and its factor counts are unchanged; the changed
sparse sampling and initial gauge produce a different contact seed. These
results therefore measure the new initialization policy as well as the API
change. The detailed local output is `results/kinematic-walk-phase-initializer.json`.

For interactive use, the same functions expose the actual optimized Values:

```python
from pathlib import Path
import gtdynamics as gtd
from kinematic_walk import load_walk, build_problem, initialize, solve_cm

walk = load_walk(Path("examples/g1_mjlab/results"),
                 Path("apps/contact-initialization/public/g1-walk.json"))
costs, equalities, keys = build_problem(walk)
seed, initialization_info = initialize(walk, "contact", keys)
solution, solver_info = solve_cm(costs, equalities, seed, walk.root.id(), 50)
last_root = solution.atPose3(gtd.PoseKey(walk.root.id(), walk.count - 1))
```

Run that snippet from the repository root with `examples/g1_mjlab` on
`PYTHONPATH`. `solution` holds all link poses and joint angles, using ordinary
GTDynamics keys, in a standard `gtsam.Values` object.

## Inspect solver iterations

The [Contact vs IK app](../../apps/contact-vs-ik/README.md) uses the same example
on three ten-second SSIO G1 walking snippets. It replays up to ten outer CM-Opt
iterations from both initializations, the sparse contact initializer history,
and IK history for each motion frame. Its offline exporter calls this example
with optional observers. The timing driver leaves those observers disabled.

The [Kinematics app](../../apps/kinematics-inspector/README.md) estimates the
entire SSIO clip on buffered contact-selected slices, with optional Gal3 IMU
aiding and a return-to-start position prior. It stores only initial and final
snapshots, factor errors, slip diagnostics, and recorded optimization timings.
