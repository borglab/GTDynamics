# GTDynamics apps

The browser apps live here, with one shared tab bar and an overview as the default
tab. [registry.json](registry.json) supplies the tab labels, overview links, and
the list of apps to build. Each app keeps its own Vue/Vite project and can also
be opened directly.

- [URDF IK](urdf/README.md): move a UR10e or UR5 tool or a Unitree G1 palm or sole with live browser IK. [Local mode](#python-backed-apps) adds the GTDynamics factor graph and full model catalogue.
- [Contact Labels](contact-labels/README.md): inspect privileged stationarity targets derived from SSIO reference-world sole motion.
- [Contact Initialization](contact-initialization/README.md): inspect native pose-contact initialization and independently selectable priors on SSIO and additional G1 recordings.
- [Contact vs IK](contact-vs-ik/README.md): compare initialization and smoothing on three ten-second G1 walking snippets.
- [Kinematics](kinematics-inspector/README.md): inspect the full contact-initialized trajectory, contact weights, foot slip, factor counts, and error budgets.
- [Interpolation](interpolation/README.md): inspect Pose3 and joint-vector Gaussian process interpolation, dense encoder measurements, and estimated velocities.
- [Dynamics](dynamics/README.md): CMC-Opt inference of sole-frame contact wrenches and all 29 G1 joint torques on the SSIO 78–98 s clip.
- [Kinodynamics](kinodynamics/README.md): a coupled 50 Hz solve of motion, accelerations, contact wrenches and joint torques from contact initialization and zero dynamic variables.
- [Motion Planning](motion-planning/README.md): Franka trajectory optimization and playback.
- [SCATE](scate/ur10e-reactive/README.md): compare base costs in the reactive experiment or use **Experiment** to replay online obstacle avoidance against a blind run on a [UR5](scate/ur5/README.md) or [UR10e](scate/ur10e/README.md).

Each app starts with a short technical abstract and its interactive demo, then
continues with motivation, a factor graph formulation and factor definitions,
numerical results, and a discussion. URDF IK introduces the factor graph for
one robot configuration. The walking examples then use inferred contact phases
for initialization, initialization for smoothing, and
Gaussian-process interpolation supplies velocities for the dynamics example.
Kinodynamics then estimates motion and forces together. Motion Planning illustrates a
separate desired-motion problem. It does not currently plan from the G1 estimate.
The shared paper components live in `shared/paper/`; solver code and recorded
numerical results remain owned by their apps.

## Appearance

The Light/Dark button in the shared header switches every app, including robot
stages and numerical plots. Light is the initial default; the browser remembers
an explicit choice across reloads and app changes. Direct app URLs on the same
origin use that preference too. Changing themes preserves playback and camera
state. The preference remains usable for the current page if browser storage is
disabled.

## Local preview

Use **Node.js 22+**, npm, Python 3, Git, and [Git LFS](https://git-lfs.com/).
Recordings and compiled display geometry are stored in LFS. Download and install Git LFS (see [Git LFS Install Instructions](https://docs.github.com/en/repositories/working-with-files/managing-large-files/installing-git-large-file-storage)),
then hydrate the app assets from the repository root before running the apps:

```bash
git lfs install
git lfs pull --include="apps/**" --exclude=""
```

Developers who do not need the apps can skip LFS downloads when cloning:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/borglab/GTDynamics.git
```

Run the LFS pull above later to obtain the current app assets. This avoids
downloading their contents during checkout; ordinary Git history still includes
large assets committed before the LFS migration. Pages CI downloads LFS assets
before testing and building, so the deployed site contains real files.

From the repository root:

```bash
cd apps
npm ci
python -m pip install -r ../examples/g1_mjlab/requirements-model.txt
npm run models
npm run dev
```

Open **http://127.0.0.1:3000/**. The overview is the leftmost, default tab.
The launcher starts the shared page, each app's Vite server, the native
planner bridge when the optimizer has been built, and the URDF IK Python backend
when GTDynamics' Python bindings import (see [Python-backed apps](#python-backed-apps)).
Ctrl-C stops the servers.
Only port 3000 needs forwarding if previewing on a remote machine.

To connect directly over Tailscale, bind the shared server to this machine's
Tailscale IPv4 address and allow that exact browser origin for live planning:

```bash
# Replace 100.x.y.z with this machine's address from tailscale ip -4.
GTDYNAMICS_APPS_HOST=100.x.y.z \
MOTION_PLANNING_ALLOWED_ORIGINS=http://100.x.y.z:3000 npm run dev
```

Open `http://100.x.y.z:3000/` from another device on the same tailnet. The app
servers, optimizer bridge, and URDF IK backend (port 8767) continue listening on
loopback; the shared server proxies them. Without these overrides, the preview accepts local connections.

To use the machine's Tailscale hostname instead of its IP, also set
`GTDYNAMICS_APPS_ALLOWED_HOSTS` to that hostname (for example, `cyber`) and add
its exact origin (for example, `http://cyber:3000`) to the comma-separated
`MOTION_PLANNING_ALLOWED_ORIGINS` list. Then open `http://cyber:3000/` directly.

The first model bootstrap downloads the pinned MuJoCo Menagerie models.
Meshes and generated asset manifests stay out of Git; the production build
copies the actual model files into the site. Model licenses are included in
their asset directories. The G1 cache lives under `models/mujoco/.cache/`;
the bootstrap maintains its model link under `models/mujoco/`.

To preview exactly what GitHub Pages will serve, including recorded planning:

```bash
npm run build
npm run preview
```

Both commands use port 3000, so stop the dev server before starting preview.
The static site is assembled in `apps/dist/`, including all registered apps, model assets,
and the shared navigation. Asset URLs are relative, so the same build works at
`/`, `/GTDynamics/`, and on a custom domain.

## Python-backed apps

URDF IK runs damped least squares in a browser worker on the static site.
Local development uses the GTDynamics Python bindings when available. For a registry entry with
`"backend": "python"`, the launcher also starts the app's
`scripts/backend.mjs`. That script runs `scripts/server.py`, and the app's Vite
config proxies the page's API requests to it on port 8767 (`URDF_BACKEND_PORT`
overrides it). Build and install the bindings as
described in [`python/README.md`](../python/README.md), then check that they
import:

```bash
python -c "import gtdynamics"
```

`npm run dev` uses the repository's `.venv/bin/python` when it exists and
`python3` otherwise. To use another interpreter, set `GTDYNAMICS_PYTHON`:

```bash
GTDYNAMICS_PYTHON=/absolute/path/to/python npm run dev
```

If that interpreter cannot import `gtdynamics`, the app uses the same browser solver
as the static build for the UR10e, UR5, and G1. To run the app on its own, see
[the URDF IK README](urdf/README.md#running).

## Live motion planning

GitHub Pages hosts static files and cannot run the native C++ optimizer or its
WebSocket bridge. The hosted Motion Planning app loads the checked-in
`motion-planning/public/recording.json`, captured from a real GTDynamics/GTSAM
solve. Arm playback, recorded iteration replay, the factor graph, and joint plots
work in the browser. The UI labels this mode and links to these local instructions.

To edit endpoints and optimize new plans, first build the native targets from
the repository root using your normal GTDynamics dependencies:

```bash
cmake -S . -B build -DGTDYNAMICS_BUILD_MOTION_PLANNING_WEBDEMO=ON
cmake --build build --target motion_planning_optimizer motion_planning_factor_test -j2
cd apps
npm run dev
```

The launcher detects `build/apps/motion-planning/backend/motion_planning_optimizer`.
Set `MOTION_PLANNING_BACKEND=/absolute/path/to/motion_planning_optimizer` to use
another build. If no backend is available, the app uses the recording instead.
See [Motion Planning setup](motion-planning/README.md#setup) for CMake dependency
options. Add `?recording` to an app's direct dev URL to inspect recorded mode.

Refresh the hosted recording after changing the planner or the default problem:

```bash
# From the repository root, after rebuilding the optimizer:
python3 apps/motion-planning/scripts/export_recording.py
```

Commit the resulting JSON. Publishing requires no C++ toolchain.

## GitHub Pages

[The Pages workflow](../.github/workflows/pages.yml) installs the npm workspace,
bootstraps the pinned models, runs the Node tests, builds all registered apps,
and uploads the combined static site. Pull requests build and validate the site;
pushes to `master` deploy it. A manual workflow run on `master` can also deploy.

In the repository's **Settings → Pages → Build and deployment**, select
**GitHub Actions** as the source. After the workflow reaches `master`, the
deployment job reports the published URL (normally
<https://borglab.github.io/GTDynamics/>). No `gh-pages` branch or checked-in
build artifacts are needed.

The workflow follows GitHub's [custom Pages workflow](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages).

## Add another app

1. Add a Vite project under `apps/<id>/`, with a `package.json` and `index.html`.
   Use relative runtime asset URLs and a relative Vite production `base`.
2. Add the directory to `workspaces` in `apps/package.json` and run `npm install`
   from `apps` to update the shared lockfile.
3. Add an entry to `registry.json` with `id` (the folder name), `title`,
   `description`, and `tag`. This adds its tab and overview link and includes it
   in the development launcher and Pages build.
4. If it needs downloaded assets, add its bootstrap to the `models` script.
   Native/server functionality needs an explicit static mode for Pages.
   An app whose computation runs in Python sets `"backend": "python"` in its
   registry entry and provides `scripts/backend.mjs` exporting
   `startBackend()`. That function returns the spawned server process, or
   `null` when Python is unavailable. The app's Vite config proxies relative API
   paths to the server, and the static build can use a browser solver
   (see [`urdf/`](urdf/README.md)). Use an app browser test to exercise live
   behavior alongside the shared paper check on the static build.
5. Start robot visualization with [the shared viewport and camera](shared/README.md).
   Use `RobotViewport` for canvas, controls, stage, resizing, animation, and
   cleanup; use `CameraController` for fit and follow views. Supply live MuJoCo
   poses or saved body transforms from the app. Add overlays to its scene.

The active app loads only when its tab is selected. Leaving that tab unloads it,
so inactive robot simulations do not keep running in the background. Browser
back/forward restores the selected tab; a URL with no tab hash opens Overview.

## Checks

`npm test` runs every app's Node tests, including URDF IK's, and needs no Python
bindings.

```bash
cd apps
npm test
npm run build
# From the repository root, with the native targets built:
ctest --test-dir build -R '^motion_planning_' --output-on-failure
```

The URDF IK backend tests need the GTDynamics Python bindings. Run them from `apps/urdf`:

```bash
python -m unittest discover -s tests -p 'test_*.py'
```

The shared site's browser smoke test accepts an external Playwright installation:

```bash
# After npm run build, from apps/ (starts its own temporary preview servers):
PLAYWRIGHT_MODULE=/absolute/path/to/playwright/index.mjs \
  node tests/site.browser.mjs
```

It checks navigation, keyboard tabs, mobile layout, physics playback, recorded
iteration scrubbing, and requests under a project-site prefix.

The presentation check covers all registered apps at 1440 × 900 and 1280 × 800,
including shared navigation, visible primary controls, diagram/time selection,
relocated results, next-tab links, and mobile width:

```bash
PLAYWRIGHT_MODULE=/absolute/path/to/playwright/index.mjs \
  node tests/paper.browser.mjs
```
