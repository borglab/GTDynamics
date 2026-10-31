# Kinematics

Inspect the full 637-sample G1 JazzWalk clip with contact initialization and
inverse kinematics, followed by 10 CM-Opt iterations on the same smoothing problem.
This app is entirely static: optimization, FK, residuals, and body transforms are computed
offline. The browser only renders stored poses and plots recorded numbers.

From `apps`, run `npm run dev` and select **Kinematics**, or run
`npm run dev --workspace kinematics-inspector` for http://127.0.0.1:3004/.
The shared `npm run build` includes this app in the static site.

## Inspect a result

- **Walking time** synchronizes both views. Contact phase strips jump to a phase.
- **CM-Opt smoothing** shows the feasible initial guess as iteration 0, followed
  by iterations 1–10 for each initialization. Each method has its own slider,
  objective plot, step buttons, and iteration replay.
- **Initialization** shows the contact initializer's joint sparse solve
  and IK's independent per-frame solves. Change walking time to choose the IK
  problem; its iteration count can differ from neighboring frames.
- **Reference overlay** shows the original retargeted motion using MuJoCo FK. The selected joint
  plot compares estimated angles with source joint angles. Ground height and slip
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

The clip is **J_ShortDance16_JazzWalk**, from
[G1 Moves](https://huggingface.co/datasets/exptech/g1-moves) by Mitch Chaiet /
Experiential Technologies, performed by **Jasmine Coro** and captured with MOVIN
TRACIN. All 637 source frames are retained at 60 Hz (10.6 seconds), with no looping
or synthesized source motion. The motion includes stepping and substantial arm
movement. It is retargeted human motion, not a measured robot rollout.

The publisher licenses the dataset under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), permitting redistribution
and adaptation with credit and an indication of changes. The website displays
the creator, performer, source and license links, and modifications. A separate
[attribution notice](public/G1_MOVES_ATTRIBUTION.txt) records the pinned revision
and source checksum. Robot geometry retains its separate license.

The shared contact detector infers 12 stance intervals and 21 phases from sole
height and speed. These form one connected contact graph, with no unsupported
samples. They are heuristic labels, not force measurements or evidence of no slip.
Only the initial root pose enters estimation; later reference root poses are used
for labeling and evaluation. The legacy IK initializer retains its existing
height-only contact model and uses a stationary root command for this clip.
The contact initializer reaches its 100-iteration cap without satisfying its
convergence criterion; that status is shown in the initialization view. Both
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
python apps/contact-initialization/scripts/export_g1_moves.py \
  --clip J_ShortDance16_JazzWalk \
  --model examples/g1_mjlab/results/model.mjb \
  --output apps/kinematics-inspector/.cache/jazzwalk/contacts.json \
  --recording-output apps/kinematics-inspector/.cache/jazzwalk
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_recording.py
```

The `--max-iterations` option defaults to 10 and limits both CM-Opt runs.
Initializer histories retain their native stopping criteria.
The shared importer downloads the CSV from a pinned dataset revision, checks
SHA-256, and writes local inputs under the ignored `.cache` directory. The
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

Only the latest full recording is retained:

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
