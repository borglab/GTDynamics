# Contact Initialization

Inspect the native `PoseContactInitializer.run` solve on the same
three SSIO snippets as Contact Labels: 24–44 s, 78–98 s, and 198–218 s.
The policy walk and retargeted step-touch use the same sampling policy. Each foot has one fixed sole frame; each uninterrupted stance has one
world `Pose3` landmark. The app has no point/patch selector or point-landmark solve.

The browser replays native iterations, timings, factor budgets and reference
errors. It performs no optimization. Its MuJoCo FK positions the canonical model
from measured joint angles; saved root COM poses are converted to pelvis body
poses explicitly for display. RGB axes show estimated sole frames. Origin dots
are visual markers, not separate point variables.

## Preview locally

From the repository root, use Node.js 22 or later and hydrate the recordings with
Git LFS. If the G1 assets are already present, start the app with:

```sh
git lfs pull --include="apps/contact-initialization/public/**" --exclude=""
cd apps
npm ci
npm run dev --workspace contact-initialization
```

Open [http://127.0.0.1:3003/](http://127.0.0.1:3003/) and scroll to **Factor graph
formulation**. Select a root pose, then move the **Contact event** slider to
select a different pose from the viewer.

On a fresh checkout, generate the G1 assets once before starting Vite. From the
repository root, in a Python 3.12 environment with a suitable `torch>=2.7` build:

```sh
python -m pip install -r examples/g1_mjlab/requirements-model.txt
python scripts/bootstrap_mujoco_models.py
```

## SSIO experiments

`export_ssio_recordings.py` consumes the exact saved max-product `supportGraph.states`
from Contact Labels, including joint-velocity unaries, duration priors, and hard
walking constraints. It does not infer another schedule. These are privileged
labels derived partly from SuperOdom LiDAR-inertial motion, not independent contact
measurements. The first selected root pose also fixes the gauge from that reference;
subsequent reference poses are used only to evaluate the result.

`initialization_slices` calls the shared native `contactInitializationSlices`
sampler for both single- and double-support phases: one central slice
from phases up to 200 ms, or evenly spaced slices between one-sample-buffered
endpoints for longer phases, with no within-phase gap above 200 ms. This is the
default for regenerating all five recordings; `--max-slice-interval-ms` sets the limit. Only these slices
are optimized; there is no dense expansion. The full snippet phases still determine
stance continuity. The strip shows the saved support states and selected slices.

Before solving, one constant vertical translation is applied to the gauge and
reference poses so the mean sole height at the first selected slice is zero. The
translation is recorded in `metadata.snippetWorldTranslation`; joint FK and labels
are unchanged. This avoids imposing a zero-height prior in the full recording's
unrelated vertical frame.

All configurations use `initializeUpright=true`. Starting values propagate only
the yaw component of relative-root observations, preserve the anchor's initial
tilt, and translate each root to match the continuing contact origin. This option
is independent of the upright prior: it adds no factors or sensors, and the solve
still estimates unrestricted six-dimensional poses. Original full-pose starting
values remain the native default for other clients.

All eight combinations of three priors are saved; all three are enabled by default:

- Pelvis upright: **15°** small-angle scale; uses the existing point-contact
  initializer's `contact_init::UprightFactor`. Position and yaw are free.
- Weak zero-motion: **10 m / 180°** per selected-pose interval, using the existing
  identity `BetweenFactor<Pose3>`. These are fixed per-edge sigmas, not measured
  velocity or IMU factors, and are not scaled by elapsed time.
- Contact height: **zero ± 10 cm**, using `ContactHeightFactor` on each sole landmark.
  Sole-normal alignment is disabled independently; this prior constrains only Z.

Contact observation standard deviations are **1 cm / 90°**. Rotation configuration
is specified in degrees and explicitly converted at the native API boundary. The
fixed gauge has no upright factor because its contribution would be constant.
Each configuration records up to 200 LM iterations, convergence status, factor
errors, reference errors, and timings (11 calls after 2 warmups; history recorded
in a separate untimed call). A no-decrease LM return is labeled **stalled**, rather
than converged, unless the gradient norm is at most 1e-6. Positive but small
objective improvements retain the usual stopping criterion. A smaller objective is not necessarily a more accurate
reference trajectory. The full-trajectory Kinematics recordings are unchanged by
this experiment.

With the model packages and native bindings available:

```sh
python apps/contact-initialization/scripts/export_ssio_recordings.py
```

The input defaults to the existing ignored `openio-contact-source` cache. The
exporter records source indices, saved-label fingerprints, and native/exporter
source hashes. The app loads only the selected snippet's compressed data and runs.
All calculations are offline; the browser replays iterations and compares priors.
The dataset attribution is linked alongside the label interpretation.

## Legacy model and graph

The shared Python pipeline uses pinned Menagerie `g1_mjlab`, MJLab and MuJoCo
packages. It defines +X forward, +Y left, +Z up sole frames on the foot capsule
lower envelope. `mujoco_io.contact_frames` converts these body-frame transforms
to the link COM frames used by GTDynamics. All FK and model geometry are derived
from that pipeline, independent of another recording's embedded geometry.

The existing contact schedules are preserved. Phases carry
`PoseContactConstraintSpec`; the initializer returns the selected source slices,
actual times, frozen root-relative contact poses, and native stance indices.
The app does not maintain a second association or graph-building algorithm.
The current policy recording selects 148 slices and 38 stance poses; the capture
selects 86 slices and 19 stance poses. Supported phase boundaries and supplemental
samples use the shared native sampler with a one-slice phase buffer. Explicit
selected slices then enter the initializer with `contactBuffer=0`, preventing
double buffering. Existing saved recordings retain their historical selection.

For frozen FK `Z_ij = rootCOM_T_sole(q_i)`, the pose observation relates root pose
`X_i` to stance pose `C_j` through `X_i Z_ij = C_j`. The native six-dimensional
pose residual has rotation sigma 90° and translation sigma 0.01 m. The first
root COM pose is fixed exactly; its corresponding pelvis body pose is identity.
No reference or IMU factors enter this graph. Continuing stance landmarks persist
across phases; lift-off creates a fresh landmark on recontact.

## Shared priors and evaluation

Both exporters use `prior_configurations()` for the same three checkboxes and
noise settings: weak motion **10 m / 180°**, pelvis upright **15°**, and contact
height **10 cm**. Contact observations use **1 cm / 90°**, upright starting values
are enabled, and sole-normal alignment is disabled. The non-SSIO ground level is
derived from initial-gauge FK; it need not be zero in the solve frame. The viewer
subtracts that level for display. Reference poses do not determine this level.

Policy walk has eight saved combinations. Step-touch has four: its hop separates
support components, so the weak motion prior stays on to connect them under one
fixed gauge. Height and upright remain independently selectable. All three priors
are enabled by default, without choosing defaults using reference accuracy.

Reference poses are held out until evaluation and aligned to the fixed first
pose. The app reports translation/orientation RMSE, unwhitened contact RMS, and
per-family factor errors. These are different motion sources, not an initializer
accuracy benchmark. Capture provenance and the retained candidate-support
heuristic are documented in [DATA.md](DATA.md).

Recordings are compressed `.json.gz` bundles stored in Git LFS. Regeneration removes
the superseded uncompressed recording for each selected clip. The SSIO bundles
were retained byte-for-byte when the legacy configurations were unified; their
source fingerprints still describe their original export, before the shared-settings
refactor. Native code and input-data fingerprints remain checked.

## Timings and reproduction

Every valid configuration has two warmups and eleven timed calls, with history
disabled. A separate call records native LM snapshots. The cap is 200 iterations. The app reports all timing samples plus
median, minimum and nearest-rank p95.

Native `graphBuildMs` includes measured-joint FK, seed propagation and graph
construction; `optimizationMs` measures LM. Native total excludes dense FK
expansion. The separately measured full-call wall time includes dense expansion
and binding overhead. Reference evaluation and JSON packaging are outside all
measured scopes. The pose initializer is the same native API used by Kinematics;
there is no separate C++ JSON solver or Python optimizer.

Install `examples/g1_mjlab/requirements-model.txt` in a Python 3.12 environment
with matching GTSAM/GTDynamics bindings. From the repository root:

```bash
cmake --build build --target gtdynamics_py testPoseContacts.run -j6
export PYTHONPATH="$PWD/build/python:/path/to/gtsam/build/python"
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python \
  apps/contact-initialization/scripts/export_native_recordings.py \
  --sequence g1-policy-8s --sequence g1-steptouch-30s
python apps/contact-initialization/backend/test_exporter.py
cd apps
npm ci
npm run models
npm test --workspace contact-initialization
npm run build
npm run preview
```

Open `/#contact-initialization` in the app hub. The exporter preserves joint
measurements, references and schedules, refreshes canonical display FK in the
datasets, and replaces old point recordings with pose-only archives. Only the
selected sequence is downloaded. Export initialization JSON includes full pose
landmarks, native observations, frame conventions and timing provenance.

The original point-contact library APIs and their C++ regression tests remain
available to other clients. The legacy `backend/CMakeLists.txt` now builds only
those point-core tests; it is not an app backend. See
[the pose API contract](docs/initializer-api.md) for library integration.
