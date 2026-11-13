# SCATE UR10e Base

Reactive SCATE on a UR10e among two moving obstacles, comparing costs on the
disturbance the arm puts on its base. A VICON-like system measures markers on
the arm and the obstacles' world positions every second; the planner re-solves
after each one (the reactive variant of King-Smith et al.). One noise draw is
replayed four times: with the base-disturbance cost off and at sigma 3 (the
default), 0.3 and 0.1 N m / N.

- Toggle each run in the **Runs** panel; each keeps its own color for the
  ghost arm, collision spheres, tool trail and plot lines. The σ = 3 run is
  the solid arm with its plan and obstacle estimates while it is shown.
- **So far** draws the base reaction torque ‖τ‖ and the obstacle clearance of
  the shown runs up to the playback time; click or drag on either to scrub.
- The results add base torque and force plots and RMS, peak and integral of
  the torque per run.

The base reaction is the gravity-free wrench of the arm on its mount, in the
base frame about its origin (`examples/example_scate_ur5/base_wrench.h`): the
part an attitude controller in orbit would have to cancel. The arm is drawn
with the UR10e's visual COLLADA meshes.

## Live simulation

With the development server, the **Live simulation** panel solves your own
setup with the native solver (`scate_live`, built with GTDynamics):

- **UR10e / UR5** picks the robot (it resets the setup to that robot's
  version of the experiment).
- **Set start and goal position** shows a green start arm and a purple goal
  arm. Choose Start or Goal, then grab any link and drag it: inverse
  kinematics pulls that point of the arm toward the pointer. The sliders set
  each joint exactly; *Show collision spheres* draws the planner's spheres.
- **Set obstacles**: drag a red sphere to where the obstacle starts and its
  orange marker to where it is at the end of the run (the same place = still).
  Set the radius, add up to four obstacles or remove one.
- **Runs to solve** picks the base-cost weights (each is one closed-loop run
  on the same noise), and **Solve and run** solves and plays the result.
  A UR10e run takes about 1.5 s.

Anything you don't set comes from the recorded experiment, so solving without
edits reproduces it. The hosted page has no solver and keeps the recording.

Build the solver once, from the GTDynamics build folder:

```bash
cmake --build . --target scate_live
```

The dev server looks for `build/examples/example_scate_ur10e/scate_live` in the
GTDynamics checkout; set `SCATE_LIVE_BIN=/path/to/scate_live` for another
build folder. The dragging uses `public/robots/*.json` (link frames, joint
screw axes, spheres and the default experiment), written by
`cmake --build . --target scate_live.describe`.

## Running

With the rest of the apps (see [apps/README.md](../../README.md)), it is the
SCATE UR10e Base tab. On its own, from `apps`:

```bash
npm ci
npm run dev --workspace scate/ur10e-reactive     # http://127.0.0.1:3012/
npm test --workspace scate/ur10e-reactive        # recording, meshes, kinematics, solve endpoint
```

The recording and meshes are in Git LFS: run `git lfs pull --include="apps/**"`
first if `replay.json` starts with `version https://git-lfs`.

## Refreshing the recording

From the GTDynamics build folder:

```bash
cmake --build . --target scate_ur10e_base_wrench.run      # writes base_wrench_replay.json
cmake --build . --target scate_ur10e_base_wrench.export   # copies it to public/replay.json
```

If the link names change, repack the meshes, from `apps/scate`:

```bash
python3 shared/scripts/pack_meshes.py ../../models/urdfs/ur10e/ur10e.urdf \
    ur10e-reactive/public/replay.json ur10e-reactive/public/meshes.json --visual
```

Browser checks from `apps/` (the first after `npm run build`; the second
drags, solves live and needs `scate_live`):

```bash
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs node scate/ur10e-reactive/tests/app.browser.mjs
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs node scate/ur10e-reactive/tests/live.browser.mjs
```
