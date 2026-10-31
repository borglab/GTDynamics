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
**IMU aiding** switches between precomputed solves with and without Gal3 IMU
factors; changing this option preserves walking time. Initializers are identical
in both variants. Shared accelerometer and gyroscope biases are shown for every
smoothing iteration.

Changing recordings resets time and disposes the previous scenes. The camera
follows the selected robot pose while preserving the orbit and zoom.

- **Walking time** synchronizes both views. Contact phase strips jump to a phase.
- **CM-Opt smoothing** shows the feasible initial guess as iteration 0, followed
  by iterations 1–10 for each initialization. Each method has its own slider,
  objective plot, step buttons, and iteration replay.
- **Initialization** shows the contact initializer's joint sparse solve
  and IK's independent per-frame solves. Change walking time to choose the IK
  problem; its iteration count can differ from neighboring frames.
- **SuperOdom reference**, enabled by default, shows the LIO root trajectory as
  a dashed blue-gray line and its measured-joint MuJoCo FK robot. The estimated
  root path is solid. Both paths are XY ground projections in the same recorded
  world frame; no additional trajectory alignment is applied. **Fit paths** frames
  both trajectories, including when the estimates drift apart. **Reset view**
  returns to the selected robot pose. The selected joint
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
are excluded. The **IMU aiding** option uses only the physical pelvis stream.

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
both initializers, with a fixed ground plane. Neither variant uses later reference
poses as measurements. IMU aiding is optional. Its original
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

Four results are retained: the turn in `public/`, the full 5 Hz trajectory in
`public/full/`, and their aided variants in `public/imu/` and `public/full-imu/`.
Each directory contains:

- `public/recording.json`: metrics, phase schedule, iteration indices, float
  offsets, and input/source/native-module fingerprints.
- `public/states.bin.gz`: delta-coded, byte-shuffled int32 body poses and joint angles,
  scaled by 1e6 and decoded into float32 for rendering. Display coordinates are
  rounded to 1e-6 (at most 0.5 micrometre position error before float32 conversion).
  Poses are `[tx, ty, tz, qx, qy, qz, qw]`, in MuJoCo body frames, Z-up.
  Aided smoothing snapshots also store world-frame COM velocities at
  `velocityOffset` (three floats per frame) and the shared six-vector `bias`
  (accelerometer then gyroscope, in sensor axes).
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

## Contact slip

The app exporter uses `--contact-slip-sigma 0.05`: each continuing ground contact
point has a soft 3D displacement cost with sigma **50 mm per axis per adjacent
slice interval**, replacing 5 mm. This reduces that factor's precision by 100×.
The sigma is a displacement tolerance, not a velocity tolerance, and is applied
at both 50 Hz and 5 Hz without sample-rate scaling. Height costs, joint costs,
root priors, initializers, IMU settings, and the ten-iteration limit are unchanged.
The generic example retains its previous 5 mm default; both drivers expose
`--contact-slip-sigma` for explicit experiments. The selected value and interval
are recorded in metadata and displayed below the viewer.

On the IMU-aided turn, the measured heading change is −147.5° in SuperOdom:

| Sigma | Contact-initialized turn | IK-initialized turn | RMS slip, contact / IK |
| --- | ---: | ---: | ---: |
| 5 mm | −37.6° | −29.8° | 2.83 / 2.83 mm |
| 20 mm | −71.2° | −110.2° | 4.93 / 4.94 mm |
| 50 mm | −101.7° | −108.2° | 5.75 / 5.75 mm |

The 50 mm runs turn more, but still underestimate the turn. Their position RMS
errors are 0.947/0.941 m, versus 0.903/0.908 m previously (contact/IK): the heading
improvement is not an improvement in every metric. These are ten-iteration
sensitivity experiments evaluated against this same reference, not held-out
validation. Compact previous results and all three trials are retained in
[`public/slip-comparison.json`](public/slip-comparison.json).

## IMU aiding

The native `Kinematics.imuFactors(trajectory, link_id, pims, bias_key)` builder
connects consecutive unique slices, including across contact phase boundaries
and flight. Python uses GTSAM's `PreintegratedImuMeasurementsG` directly. One
`pim` covers each slice interval; the collection is `pims`. Measurements retain
native nonuniform timestamps and are linearly interpolated within native sample
intervals, then integrated at subinterval midpoints with exact slice boundaries.

`PoseKey` is `world_T_COM`, mapping the link COM principal frame into the world.
`LinearVelocityKey` stores a three-vector in world axes, the velocity of that COM;
it is distinct from the body-frame six-dimensional `TwistKey`. Gravity is
`[0, 0, -9.81]` m/s². The physical pelvis IMU measures specific force, including
gravity at rest, and angular rate in pelvis axes. The model supplies
`COM_T_IMU = pelvis_T_COM.inverse() * pelvis_T_IMU`, used as `body_P_sensor`.
GTSAM corrects rotation and centripetal acceleration at the sensor lever arm;
its correction omits the angular-acceleration term. This remains an approximation.

A single shared `ConstantBias` is expressed in IMU measurement axes. It starts
at zero with prior sigmas 0.5 m/s² and 0.05 rad/s. Velocities are initialized by
finite differences of each initializer's COM positions. The initial world
velocity has a zero-mean prior with sigma 3 m/s, identical for both methods;
reference velocities are never used. Noise densities are provisionally
0.1 m/s²/√Hz and 0.01 rad/s/√Hz, with integration covariance 1e-8.
These are example settings, not a sensor calibration. The poses retain the same
initial gauge and kinematic constraints as the unaided problem.

Aided and unaided objectives contain different factors and are not directly
comparable. Inspect reference position errors and contact/joint residuals instead.
The loose-contact experiment below improves the turn but does not establish
improved position accuracy or calibrated inertial odometry.

After importing both recordings, generate the aided variants with:

```bash
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_recording.py --imu \
  --output apps/kinematics-inspector/public/imu
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_recording.py --imu \
  --data apps/kinematics-inspector/.cache/openio-full \
  --contacts apps/kinematics-inspector/.cache/openio-full/contacts.json \
  --output apps/kinematics-inspector/public/full-imu
```

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
