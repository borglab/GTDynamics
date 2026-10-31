# Kinematics

Inspect two physical G1 recordings with contact initialization and
inverse kinematics, followed by 10 CM-Opt iterations on the same smoothing problem.
This app is entirely static: optimization, FK, residuals, and body transforms are computed
offline. The browser only renders stored poses and plots recorded numbers.

From `apps`, run `npm run dev` and select **Kinematics**, or run
`npm run dev --workspace kinematics-inspector` for http://127.0.0.1:3004/.
The shared `npm run build` includes this app in the static site.

## Inspect a result

The selector next to the stage switch offers:

| Recording | Sample rate | Samples |
| --- | --- | --- |
| Turn, source seconds 204–214 | 50 Hz | 500 |
| Entire trajectory, approximately 220 seconds | 5 Hz | 1,100 |

Both have all initializer histories and ten CM-Opt iterations per method.
Changing recordings resets time and disposes the previous scenes. The camera
follows the selected robot pose while preserving the orbit and zoom.

- **Walking time** synchronizes both views. Contact phase strips jump to a phase.
- **CM-Opt smoothing** shows the feasible initial guess as iteration 0, followed
  by iterations 1–10 for each initialization. Each method has its own slider,
  objective plot, step buttons, and iteration replay.
- **Initialization** shows the contact initializer's joint sparse solve
  and IK's independent per-frame solves. Change walking time to choose the IK
  problem; its iteration count can differ from neighboring frames.
- **SuperOdom reference** shows the LIO root trajectory with measured-joint MuJoCo FK. The selected joint
  plot compares estimated angles with recorded joint angles. Ground height and slip
  summaries are residuals of the actual saved values.

Contact initialization uses frozen measured-joint FK. Its sparse root poses are
expanded for display with the same SE(3) interpolation as the native adapter.
IK snapshots preserve their actual link poses even before they are kinematically
feasible. The viewer does not replace these with FK from joint angles.

Iteration 0 is the initial guess. Subsequent indices identify completed outer LM
iterations, including an unchanged iterate when an optimizer terminates that way.
Internal damping trials and the inner CM-Opt feasibility retraction solves are not
separate iterations.
Contact objectives and IK objectives belong to different problems; use the
**smoothing objective** to compare the two methods on a common objective.

## Motion source and attribution

The recording is **10871_humanoid_20260520_223451**, from
[Open-IO's Unitree G1 Inertial Odometry Dataset](https://huggingface.co/datasets/Open-IO/humanoid-g1-odometry),
released with [SSIO](https://ssio-imu.github.io/) by Yuheng Qiu and collaborators.
It contains 43,991 samples over 219.96 seconds from a physical G1: the pelvis
accelerometer and gyroscope, all 29 joint angles/velocities/estimated torques,
and a SuperOdom LiDAR-inertial reference trajectory. The reference is an odometry
estimate, not independent motion-capture ground truth.

The app uses seconds **204–214**, containing sustained walking and a roughly 148° right turn over 2.9 m. Encoders
and reference poses are resampled at 50 Hz (500 samples) using linear
interpolation and quaternion SLERP. The 2,000 physical pelvis IMU samples in
that interval retain their original measurements and nonuniform timestamps in
`public/imu.json`. The other 13 body-part IMU-format streams are derived and
are excluded. **This kinematic smoother does not yet use IMU factors.**

The dataset's pelvis pose origin is the pelvis IMU site. The importer applies
the model's `[0.04525, 0, -0.08339]` m mounting offset to recover the robot root.
It validates Unitree SDK joint order and this transform against both recorded
foot poses at 257 samples across the full recording (maximum position difference
3.1 micrometres). A constant world translation puts the initial XY near zero and
the estimated floor at Z=0; it does not remove time-varying LIO drift.

The publisher licenses the dataset under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), permitting redistribution
and adaptation with credit and an indication of changes. The website displays
the authors, source and license links, and modifications. A separate
[attribution notice](public/OPENIO_ATTRIBUTION.txt) records the pinned revision
and source checksums. Robot geometry retains its separate license.

The shared contact detector infers 9 stance intervals and 15 phases from sole
height and speed. These form one connected contact graph, with no unsupported
samples. They are heuristic labels, not force measurements or evidence of no slip.
Only the initial root pose enters estimation; later reference root poses are used
for labeling and evaluation. The legacy IK initializer retains its existing
height-only contact model and uses a stationary root command for this clip;
the dataset supplies no commanded speed. The contact initializer converges after
27 iterations. Both
subsequent CM-Opt runs retain exactly ten iterations and satisfy their kinematic
equalities to less than 1e-5 in L2 norm.

## Regenerate offline

Build the standard bindings with their GTSAM dependency and install MuJoCo in the
same Python environment. The G1 example's local model must be available at
`examples/g1_mjlab/results/model.mjb`; see the
[example walkthrough](../../examples/g1_mjlab/KINEMATIC_WALK.md) for setup.
From the repository root, using the Python interpreter selected by CMake:

```bash
cmake --build build --target gtdynamics_py -j2
python apps/kinematics-inspector/scripts/import_openio.py
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_recording.py
```


To generate the entire recording at 5 Hz as the second selector option:

```bash
python apps/kinematics-inspector/scripts/import_openio.py \
  --full-trajectory --fps 5 --output apps/kinematics-inspector/.cache/openio-full
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_recording.py \
  --data apps/kinematics-inspector/.cache/openio-full \
  --contacts apps/kinematics-inspector/.cache/openio-full/contacts.json \
  --output apps/kinematics-inspector/public/full
```

The full trajectory detects contacts at 50 Hz before sampling labels at 5 Hz.
A rolling 1.02-second fifth-percentile floor accommodates LIO vertical drift
for contact labeling only; reference poses receive only one constant world
translation. Missed support and disconnected contact components are retained,
not filled with invented contacts. The native contact initializer uses its
existing weak identity-motion factors (translation sigma 10 m, rotation sigma
pi rad) to connect these components. The smoother uses the same objective for
both initializers, with a fixed ground plane. Neither solve uses IMU factors
or later reference poses as measurements. The full result is consequently a
coarse kinematic estimate, not an inertial odometry reconstruction. Its original
43,991 IMU samples are available in `public/full/imu.json`.

The `--max-iterations` option defaults to 10 and limits both CM-Opt runs.
Initializer histories retain their native stopping criteria.
The importer requires NumPy, SciPy, and MuJoCo. It downloads all six source files
for the full recording from pinned revision `db621c57833e08a812b2ec8491aa0ca81f0ecb33`,
checks SHA-256, and retains them under the ignored
`.cache/openio/source/10871_humanoid_20260520_223451` directory. The prepared
excerpt is under `.cache/openio`. Use `--start-seconds`, `--seconds`, and `--fps`
to select another window; unsupported or disconnected contact schedules fail
explicitly for excerpts. The full-trajectory option retains them and enables
the native weak motion priors. Native IMU timestamps are preserved regardless of the pose sample rate. The
original MJLab comparison inputs are unchanged. To export those instead, pass
`--data examples/g1_mjlab/results --contacts apps/contact-initialization/public/g1-walk.json`.

The exporter calls `kinematic_walk.initialize`, `solve_cm`, `build_problem`, and
`evaluate`. Its optional observers capture real native iterations. For a quick
check, pass `--max-frames 12 --output /tmp/kinematics-inspector-smoke`; keep the
full recording in `public` for the app and its data tests.

The exporter checks final history states against native output, verifies dense
contact interpolation, and refuses nonfinite or infeasible final results. History
capture adds overhead: this export is **not a timing benchmark**. Continue using
`compare_kinematic_walk.py` for timings without history capture.

Both selected results are retained: the turn in `public/`, and the full 5 Hz
trajectory in `public/full/`. Each directory contains:

- `public/recording.json`: metrics, phase schedule, iteration indices, float
  offsets, and input/source/native-module fingerprints.
- `public/states.bin.gz`: delta-coded, byte-shuffled int32 body poses and joint angles,
  scaled by 1e6 and decoded into float32 for rendering. Display coordinates are
  rounded to 1e-6 (at most 0.5 micrometre position error before float32 conversion).
  Poses are `[tx, ty, tz, qx, qy, qz, qw]`, in MuJoCo body frames, Z-up.
  Offsets count decoded floats. `storage.blocks` lists consecutive block sizes;
  each block is a delta from the preceding block of the same size. Each snapshot has all walking frames, except per-frame
  IK snapshots, which contain one frame. Metrics retain float64 precision in JSON.
- `public/model.json.gz`: compiled visual geometry using the same model contract
  as the shared `mujocoRobot.js` renderer. No model runtime is loaded by this app.
- `public/MODEL_LICENSE.txt`: Unitree model license; geometry is exported from
  the G1 MJLab recording. The source model is from Unitree Robotics / MuJoCo Menagerie.
- `public/imu.json`: original pelvis accelerometer and gyroscope measurements
  for the selected window; nanosecond origin is a decimal string to avoid
  JavaScript integer precision loss, with exact integer nanosecond offsets.
- `public/OPENIO_ATTRIBUTION.txt`: dataset citation, license, modifications, and
  checksums. Full source arrays remain cached locally, outside version control.

A modern browser with `DecompressionStream('gzip')` and WebGL is required.

## Checks

```bash
cd apps
npm test --workspace kinematics-inspector
npm run build
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs \
  node kinematics-inspector/tests/app.browser.mjs
```

The browser check exercises both stages, time and iteration scrubbing, replay,
reference overlays, joint selection, mobile layout, and a project-site prefix.
