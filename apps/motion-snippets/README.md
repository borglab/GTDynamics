# Motion Snippets

Vite + Vue app for MuJoCo trajectory playback, available in the shared
[GTDynamics apps site](../README.md) and on GitHub Pages.

## Prerequisites

- Node.js 22+
- Git (for bootstrap script)

## First-time setup

From repo root:

```bash
python scripts/bootstrap_mujoco_models.py
```

This downloads model assets on demand (sparse checkout from MuJoCo Menagerie), creates symlinks, and regenerates:
- `apps/motion-snippets/public/examples/scenes/files.json`

Model provenance is documented in:
- `models/mujoco/README.md`

## Run locally

```bash
cd apps
npm ci
npm run dev --workspace motion-snippets
```

## Control model

This demo runs MuJoCo dynamics; it is not pure kinematic teleporting.

- Default behavior in this GTD fork: `control_mode: "position"`
- Meaning: `ctrl` is sent as desired joint positions for MuJoCo `<position>` actuators
- MuJoCo still resolves forces/torques, contacts, and friction physically

Optional fallback for custom models:

- `control_mode: "torque_pd"` uses explicit torque-like PD in JS before writing `ctrl`

Per-robot controller mode is configured in:
- `apps/motion-snippets/public/examples/configs/<robot>.json`

Note:
- If a clip includes `root_pos` / `root_quat`, the root can be replayed directly from the clip.
- For fully dynamic locomotion evaluation, use clips/controllers that do not hard-impose root motion every tick.

## File layout

- Scene assets (symlinked by bootstrap script):
  - `apps/motion-snippets/public/examples/scenes/<model>/...`
- Scene preload list:
  - `apps/motion-snippets/public/examples/scenes/files.json`
- Robot configs:
  - `apps/motion-snippets/public/examples/configs/<robot>.json`
- Motion assets:
  - `apps/motion-snippets/public/examples/motions/<robot>/motions.json`
  - `apps/motion-snippets/public/examples/motions/<robot>/motions/<clip>.json`

## Add a motion clip

1. Put clip JSON files in:
   - `apps/motion-snippets/public/examples/motions/<robot>/motions/`
2. Register them in:
   - `apps/motion-snippets/public/examples/motions/<robot>/motions.json`

Supported clip keys:
- `joint_pos` or `jointPos`
- `root_pos` or `rootPos` (optional)
- `root_quat` or `rootQuat` (optional)

## Export from GT Dynamics

Use the exporter in the main repo:

```bash
python scripts/export_motion_clip.py \
  --input /path/to/trajectory.json \
  --output apps/motion-snippets/public/examples/motions/g1/motions/my_clip.json \
  --joint-names left_hip_pitch_joint right_hip_pitch_joint ... \
  --fps 60
```

Then add `my_clip.json` to `apps/motion-snippets/public/examples/motions/g1/motions.json`.

## Attribution

- Direct code structure origin: [Axellwppr/humanoid-policy-viewer](https://github.com/Axellwppr/humanoid-policy-viewer)
  Qingzhou Lu (Axellwppr on Github) said: "The framework code will be MIT-licensed (I’ll update the repo shortly). Mjlab and any motion datasets remain under their respective licenses."
- Runtime dependencies:
  - [mujoco-js](https://www.npmjs.com/package/mujoco-js) (MuJoCo WASM)
  - [three.js](https://threejs.org/)
  - [Vue 3](https://github.com/vuejs/core)
  - [Vite](https://github.com/vitejs/vite)
