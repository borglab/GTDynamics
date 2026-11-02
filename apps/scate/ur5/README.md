# SCATE UR5

Replay of SCATE (simultaneous control and trajectory estimation) on a UR5. The
arm sweeps toward its goal; an obstacle on its path comes into sensor range at
step 5, and the planner re-solves after every executed step to avoid it. A
blind run with the same noise, which never sees the obstacle, hits it.

![SCATE dashboard](docs/dashboard.png)

The recording is `public/replay.json`, written by the C++ example
[`examples/example_scate_ur5`](../../../examples/example_scate_ur5/README.md)
with GTDynamics' `ObstacleSDFFactorGP` and `SelfCollisionSphereFactorGP`. The
arm meshes in `public/meshes.json` come from the UR5 collision STLs in
`models/urdfs/ur5`. The viewer can also open any other `replay.json`
(Open replay.json, or drag and drop).

## Running

With the rest of the apps (see [apps/README.md](../../README.md)), it is the
SCATE UR5 tab. On its own, from `apps`:

```bash
npm ci
npm run dev --workspace scate/ur5     # http://127.0.0.1:3006/
npm test --workspace scate/ur5        # checks the recording and meshes
```

The recording and meshes are in Git LFS: run `git lfs pull --include="apps/**"`
first if `replay.json` starts with `version https://git-lfs`.

## Refreshing the recording

After changing the planner, from the GTDynamics build folder:

```bash
cmake --build . --target scate_online.run      # writes replay.json
cmake --build . --target scate_online.export   # copies it to public/replay.json
cmake --build . --target scate_online.plot     # PNGs and a standalone page
```

Commit the new `public/replay.json` (and `docs/dashboard.png` if it changed,
from `build/examples/example_scate_ur5/scate_dashboard.png`).

`scripts/plot_scate.py replay.json` writes the dashboard, joint and control
plots, and `scate_replay.html`: this viewer as one file with the run built in,
which opens without a server. It needs numpy and matplotlib.
