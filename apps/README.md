# GTDynamics apps

The browser apps live here, with one shared tab bar and an overview as the default
tab. [registry.json](registry.json) supplies the tab labels, overview links, and
the list of apps to build. Each app keeps its own Vue/Vite project and can also
be opened directly.

- [Motion Planning](motion-planning/README.md): Franka trajectory optimization and playback.
- [Contact Initialization](contact-initialization/README.md): replay C++ GTSAM walking initialization, inspect native timings, and switch between two G1 motion sequences.

- [Contact vs IK](contact-vs-ik/README.md): compare initialization and smoothing on three ten-second G1 walking snippets.
- [Kinematics](kinematics-inspector/README.md): inspect the full contact-initialized trajectory, contact weights, foot slip, factor counts, and error budgets. This is the last tab.

## Local preview

Use **Node.js 22+**, npm, Python 3, and Git. From the repository root:

```bash
cd apps
npm ci
npm run models
npm run dev
```

Open **http://127.0.0.1:3000/**. The overview is the leftmost, default tab.
The launcher starts the shared page, each app's Vite server, and the native
planner bridge when the optimizer has been built. Ctrl-C stops the servers.
Only port 3000 needs forwarding if previewing on a remote machine.

To connect directly over Tailscale, bind the shared server to this machine's
Tailscale IPv4 address and allow that exact browser origin for live planning:

```bash
# Replace 100.x.y.z with this machine's address from tailscale ip -4.
GTDYNAMICS_APPS_HOST=100.x.y.z \
MOTION_PLANNING_ALLOWED_ORIGINS=http://100.x.y.z:3000 npm run dev
```

Open `http://100.x.y.z:3000/` from another device on the same tailnet. The app
servers and optimizer bridge continue listening on loopback; the shared server
proxies them. Without these overrides, the preview accepts local connections.

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

The active app loads only when its tab is selected. Leaving that tab unloads it,
so inactive robot simulations do not keep running in the background. Browser
back/forward restores the selected tab; a URL with no tab hash opens Overview.

## Checks

```bash
cd apps
npm test
npm run build
# From the repository root, with the native targets built:
ctest --test-dir build -R '^motion_planning_' --output-on-failure
```

The shared site's browser smoke test accepts an external Playwright installation:

```bash
# After npm run build, from apps/ (starts its own temporary preview servers):
PLAYWRIGHT_MODULE=/absolute/path/to/playwright/index.mjs \
  node tests/site.browser.mjs
```

It checks navigation, keyboard tabs, mobile layout, physics playback,, recorded iteration scrubbing, and requests under a project-site prefix.
