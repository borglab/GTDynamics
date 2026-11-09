# SCATE UR10e

The [SCATE UR5](../ur5/README.md) replay, on a UR10e: the same scenario, planner
and online loop, with the UR10e's geometry and collision spheres. The arm
sweeps toward its goal; an obstacle placed on its offline plan comes into
sensor range partway through, and the planner re-solves after every executed
step to avoid it. A blind run with the same noise, which never sees the
obstacle, hits it.

![SCATE UR10e dashboard](docs/dashboard.png)

The recording is `public/replay.json`, written by the C++ example
[`examples/example_scate_ur10e`](../../../examples/example_scate_ur10e/README.md).
The arm meshes in `public/meshes.json` are the UR10e collision STLs in
`models/urdfs/ur10e`, moved into the link frames the replay records by
`../shared/scripts/pack_meshes.py`. The page, viewer and plot script are
shared with the UR5 app (`apps/scate/shared`).

## Running

With the rest of the apps (see [apps/README.md](../../README.md)), it is the
SCATE UR10e tab. On its own, from `apps`:

```bash
npm ci
npm run dev --workspace scate/ur10e     # http://127.0.0.1:3011/
npm test --workspace scate/ur10e        # checks the recording and meshes
```

The recording and meshes are in Git LFS: run `git lfs pull --include="apps/**"`
first if `replay.json` starts with `version https://git-lfs`.

## Refreshing the recording

After changing the planner, from the GTDynamics build folder:

```bash
cmake --build . --target scate_ur10e_online.run      # writes replay.json
cmake --build . --target scate_ur10e_online.export   # copies it to public/replay.json
cmake --build . --target scate_ur10e_online.plot     # PNGs and a standalone page
```

Commit the new `public/replay.json` (and `docs/dashboard.png` if it changed,
from `build/examples/example_scate_ur10e/scate_dashboard.png`). If the link
names in the replay change, repack the meshes, from `apps/scate`:

```bash
python3 shared/scripts/pack_meshes.py ../../models/urdfs/ur10e/ur10e.urdf \
    ur10e/public/replay.json ur10e/public/meshes.json
```

Browser checks from `apps/` (after `npm run build`):

```bash
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs node scate/ur10e/tests/app.browser.mjs
```
