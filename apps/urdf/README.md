# URDF IK Explorer

An interactive browser app for inverse kinematics on the robots in [`models/urdfs`](../../models/urdfs).
Pick a model and an end-effector frame, then drag the tool.
GitHub Pages runs live damped least squares on the UR10e, UR5, and Unitree G1 in a browser worker.
Local mode adds the GTDynamics factor graph, the other solvers, and the full URDF catalogue.
The app shows the resulting joint angles and end-effector pose.
If the target can't be reached, it explains why.
The paper below the demo describes the factor graph and shows solver settings, residuals, and diagnostics. [Using the app](#using-the-app) lists every control.

## Requirements

- For native mode, build and install the GTDynamics Python bindings as described in [`python/README.md`](../../python/README.md) (`make python-install`). Check that `import gtdynamics` works.
  `numpy` and `gtsam` come with them.
  No other Python packages are needed: the backend uses only the standard library.
- For local development, install the apps workspace with `npm ci` in `apps/` using Node.js 22+ (see [the apps README](../README.md#local-preview)).
- The browser worker uses the workspace’s MuJoCo WebAssembly package for forward kinematics and Jacobians. The build bundles the URDFs and visual meshes. Run `npm run models` from `apps/` to generate G1’s fixed-pelvis URDF from the pinned Menagerie/MJLab model.
- A modern web browser with WebGL.

## Running

The **URDF IK** tab is the first app after Overview on the [GTDynamics apps site](../README.md).
The static build always uses browser IK. In native mode, Vite proxies the page’s
`api/`, `files/` and `pkg/` requests to the Python backend on port 8767. The
launcher points it at the checkout’s `models/urdfs` directory.
The app follows the site's Light/Dark setting.

`npm run dev` looks for Python in this order: `GTDYNAMICS_PYTHON`, then the repository's `.venv/bin/python`, then `python3`.
If the backend is unavailable, the app uses browser IK.

To run the whole apps site, including this tab, start it from `apps/` and open <http://127.0.0.1:3000/#urdf>:

```sh
npm run dev
```

To run only this app, start it from `apps/` and open <http://127.0.0.1:3010/>:

```sh
npm run dev --workspace urdf
```

To use a specific interpreter, set `GTDYNAMICS_PYTHON`:

```sh
GTDYNAMICS_PYTHON=/absolute/path/to/python npm run dev --workspace urdf
```

`npm run server --workspace urdf` starts only the backend, and `npm run frontend --workspace urdf` only the Vite server.
To start the backend directly, run this from the repository root with the GTDynamics environment active:

```sh
python apps/urdf/scripts/server.py
```

Backend options:

| Option | Default | Meaning |
| --- | --- | --- |
| `--port` | `8767` | Port to listen on. If you change it, set `URDF_BACKEND_PORT` to the same value for the Vite server. |
| `--host` | `127.0.0.1` | Interface to bind. Browsers reach the backend through the Vite proxy, so the default is usually right. |
| `--urdf-root` | `models/urdfs` (`gtd.URDF_PATH`) | Directory searched recursively for `*.urdf` files. |

## Using the app

| Action | How |
| --- | --- |
| Load a robot | **Model** dropdown. Browser mode includes the UR10e, UR5, and G1. Native mode lists every `*.urdf` under `models/urdfs`. Both use a fixed base. |
| Pick what to move | Double-click a movable link or its skeleton point, or use the **End-effector frame** dropdown. The dropdown shows how many joints move each frame and includes tool frames such as `tool0`, G1 palms and soles, and A1 toes in native mode. |
| Pick the solver | **IK scheme** dropdown (see below). Changing it re-solves to the current target. |
| Move the end-effector | Click and hold the **orange handle**, then drag. It moves in the plane facing the camera. IK runs continuously while you drag, warm-started from the previous solution. |
| Change the end-effector pose | **Double-click the handle.** A rotation gizmo appears and the pose panel is highlighted: drag the rings to rotate, press <kbd>W</kbd> for the axis-arrow move gizmo and <kbd>E</kbd> for rotate, and <kbd>Esc</kbd> (or double-click again) when done. You can also type an exact target (x, y, z in meters; roll, pitch, yaw in degrees, URDF convention `R = Rz(yaw)·Ry(pitch)·Rx(roll)`) and press Enter or **Solve to target**. |
| Position only vs. full pose | **Constrain orientation**. Off: only the end-effector position is solved for. On: the full 6-DoF pose is. It defaults to on for chains with 6 or more joints. |
| Accuracy and effort | **Diagnostics and solver settings** opens the results section with **Pos tol**, **Rot tol** and **Max iters** (blank uses the scheme's default). |
| Set joints directly | The **Joints** sliders run forward kinematics, and the target snaps to the new end-effector pose. **Home** returns G1 to its starting stance and sets other models to zero, or to mid-range when zero is outside a joint’s limits. Joints at a limit are shown in red. |
| Camera | Drag the background to orbit, right-drag to pan, scroll to zoom. **Fit view** frames the robot. |

The viewport shows:

- The robot meshes.
- A blue stick-figure **skeleton** of the robot’s link frames.
- The **target**: the orange handle and its faded axes.
- The **achieved end-effector pose**: solid axes.

When a solve fails, the handle turns red and a dashed red line joins the achieved pose to the target.
The pose panel always shows the current end-effector position, roll/pitch/yaw and quaternion next to the target.

## IK schemes

| Scheme | What it does |
| --- | --- |
| **GTDynamics factor graph (LM)** | GTDynamics' own IK. It jointly optimizes every link pose and joint angle over the kinematic factor graph with Levenberg–Marquardt. Pose goals use `Kinematics.inverse` with a `PoseGoal`, joint priors at the current configuration, and GTDynamics' joint-limit factors. Position-only goals use the same kinematic factors (`DynamicsGraph.qFactors`) plus a `PointGoalFactor`, warm-started the same way. That variant does not enforce joint limits. |
| **Damped least squares** | Levenberg–Marquardt in joint space on the geometric Jacobian. The damping adapts each step, and joints are clamped to their limits. This is the most robust choice for interactive dragging. |
| **Jacobian pseudo-inverse** | Newton–Raphson with the undamped Moore–Penrose pseudo-inverse. It converges fast near a solution, but can jump around near singularities. |
| **Jacobian transpose** | Gradient descent (`Δq = α Jᵀe` with the optimal step length). Each step is cheap, but convergence is slow, especially for orientation. |
| **Cyclic coordinate descent (CCD)** | Solves for one joint at a time, tip to base, rotating each joint toward the target. It is robust for position goals and weaker for orientation. |
| **Panda IKFast (analytical)** | Appears in native mode when the Python bindings include `PandaIKFast` and the loaded arm matches its seven-joint geometry. Solves full poses for `link7` and rigidly attached tool frames such as `link8` and `hand`, accounting for their fixed offsets. It tries the current seventh-joint angle first, then samples that joint's limits if necessary. At the first successful sample it chooses the joint-limit-valid branch closest to the current configuration. |

Panda IKFast requires **Constrain orientation**. Selecting an unsupported frame,
another robot, or a position-only goal removes it from the menu and switches to
damped least squares. **Max iters** bounds the number of free-joint samples
(default 33); the timing includes sampling, branch filtering, and forward-kinematics
validation. IKFast omits singular solutions, and a finite sampling failure does
not establish that the target is unreachable. Building with
`GTDYNAMICS_BUILD_PANDA_ROBOT=ON` exposes the existing native solver to Python.

Browser mode offers damped least squares and uses MuJoCo for forward kinematics.
The UR arms start from a bent configuration for dragging. G1 starts with slightly bent knees so its soles can move vertically.
The native joint-space schemes use GTDynamics for forward kinematics.

## Why a target can't be solved

A solve is **Solved** when position error and any constrained orientation error are within tolerance.
Otherwise the status panel says **Not solved** and lists the reasons it found.
It still shows the best configuration the solver reached.
The native mode checks are:

| Reason | How it is detected |
| --- | --- |
| Out of reach | The target is farther from the first joint than the sum of the chain's link lengths plus the prismatic travel. That sum is a strict upper bound on reach. |
| Too few joints | The chain has fewer movable joints than the task needs: 6 for a pose, 3 for a position. An example is a 3-joint leg with *Constrain orientation* on. |
| Joint limits | Joints sit at a limit while the error gradient pushes them further out. The panel names the joints and the limits. Solutions that violate a limit are reported too. |
| Singularity | The Jacobian at the solution is rank-deficient (condition number below 1e-3), as when the arm is fully stretched or joint axes line up. |
| Singular start | The solve began at a singular configuration, such as the stretched zero pose of the UR arms. Bend the arm with the sliders first. |
| Inconsistent factor graph | GTDynamics only: the optimizer settled in a local minimum where its link poses no longer agree with its joint angles. |
| Iteration limit | The solver ran out of iterations while the error was still going down. |
| Local minimum | No small joint motion reduces the remaining error. |
| Solver error | The optimizer raised an exception. Its message is shown. |

Native mode can warn about singularities and joint limits even when a solve succeeds.
Browser mode reports reach, insufficient degrees of freedom, stalled solves, iteration limits, and joints at their bounds.

## Native model notes

- G1’s pelvis is fixed at the world origin in both modes. Its 29 joints and tool frames come from the pinned G1 used by the walking demos. The explorer renders the same compiled geometry from the solver’s link poses.
- Every model is test-parsed in a separate process before it is loaded, because some URDFs crash GTDynamics' native parser (for example, `test/four_bar_linkage.urdf`).
  Models that GTDynamics can't load still render in the viewport, with the parser's error in the status panel.
  They are marked *(not loadable)* in the dropdown.
  These currently include `fetch*.urdf`, `fanuc_lrmate200id.urdf` (links with invalid inertia), `plane.urdf`, `walls.urdf`, and most files in `test/` (closed loops and files sdformat rejects).
- GTDynamics loads URDFs through sdformat, which merges links attached by fixed joints into their parents and drops links without inertia.
  The app lists the merged frames (such as `tool0`) as end-effectors using the fixed offsets from the URDF.
  It also tells you which joints were dropped (for example, the Panda's finger joints).
- Meshes are served from `models/`.
  `package://<pkg>/<path>` URIs resolve to any file under `models/` whose path ends in `<pkg>/<path>` (or `<path>`).
  Several URDFs (atlas, nao, fetch, bar_lab, laikago, fanuc) reference ROS packages that aren't in this repository.
  For those, the app shows the skeleton and any primitive shapes, and the panel reports how many mesh files are missing.

## Tests

The backend tests check four things:

- Link frames against an independent URDF forward-kinematics implementation.
- The geometric Jacobian against finite differences.
- That every scheme solves reachable goals.
- That each failure reason is reported.

Run them from this directory with the GTDynamics environment active:

```sh
python -m unittest discover -s tests -p 'test_*.py' -v
```

The Node tests cover the pose, joint and status helpers and the request helpers in `src/`.
`npm test` in `apps/` runs them with the other apps' tests.
To run only these, from `apps/`:

```sh
npm test --workspace urdf
```

The browser test checks the three bundled models, pose solves, and forward kinematics on the static build at `/` and under a project prefix. It rejects backend requests on those pages.
When `gtdynamics` imports, it also exercises the native factor graph.
Build the app first, from `apps/`:

```sh
npm run build --workspace urdf
```

Then run the test from `apps/urdf`:

```sh
PLAYWRIGHT_MODULE=/absolute/path/to/playwright/index.mjs node tests/app.browser.mjs
```

## Files

| File | Contents |
| --- | --- |
| `index.html`, `src/main.js`, `src/App.vue` | Vite entry, page markup, and solver mode. |
| `src/explorer.js` | The three.js scene, dragging and pose editing, and the side-panel logic (`createExplorer`). |
| `src/pose.js`, `src/api.js` | Pose, joint and status helpers, and requests to the backend. |
| `src/browser-backend.js`, `src/browser-worker.js`, `src/browser-ik.js` | Bundled model assets, worker transport, MuJoCo kinematics, and browser IK. |
| `src/style.css` | Styles, with light and dark palettes keyed on the shared theme. |
| `vite.config.mjs` | Proxies `api/`, `files/` and `pkg/` to the backend, also under the site's `/urdf/` prefix. |
| `scripts/server.py` | Standard-library HTTP backend. It serves URDF and mesh files and a JSON API: `GET /api/models`, `GET /api/schemes`, `POST /api/load`, `POST /api/fk`, `POST /api/ik`. |
| `scripts/ik_backend.py` | Robot model (GTDynamics + URDF XML), the IK schemes and the failure diagnostics. |
| `scripts/backend.mjs`, `scripts/dev.mjs` | Start the backend for `npm run dev`, here and in the apps launcher. |
| `tests/test_ik_backend.py` | Backend unit tests. |
| `tests/pose.test.mjs` | Node tests for `src/pose.js` and `src/api.js`. |
| `tests/app.browser.mjs` | Static browser solves and the native mode when GTDynamics is available. |
