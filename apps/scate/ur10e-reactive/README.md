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

## Running

With the rest of the apps (see [apps/README.md](../../README.md)), it is the
SCATE UR10e Base tab. On its own, from `apps`:

```bash
npm ci
npm run dev --workspace scate/ur10e-reactive     # http://127.0.0.1:3012/
npm test --workspace scate/ur10e-reactive        # checks the recording and meshes
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

Browser checks from `apps/` (after `npm run build`):

```bash
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs node scate/ur10e-reactive/tests/app.browser.mjs
```
