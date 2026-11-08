# URDF IK Explorer

An interactive browser app for inverse kinematics on the robots in [`models/urdfs`](../../models/urdfs).
Pick a URDF and an end-effector frame, then drag the end-effector around.
Each move runs the selected IK scheme on GTDynamics' kinematic model.
The app shows the resulting joint angles and end-effector pose.
If the target can't be reached, it explains why.
The paper below the demo describes the factor graph and shows solver settings, residuals, and diagnostics. [Using the app](#using-the-app) lists every control.

## Requirements

- The GTDynamics Python bindings (`import gtdynamics` must work), built and installed as described in [`python/README.md`](../../python/README.md) (`make python-install`).
  `numpy` and `gtsam` come with them.
  No other Python packages are needed: the backend uses only the standard library.
- Node.js 22+ and the apps workspace installed with `npm ci` in `apps/` (see [the apps README](../README.md#local-preview)).
  [three.js](https://threejs.org) 0.160.0 and [urdf-loader](https://github.com/gkjohnson/urdf-loaders) 0.12.3 are npm dependencies of this workspace.
- A modern web browser with WebGL.

## Running

The **URDF IK** tab is the first app after Overview on the [GTDynamics apps site](../README.md).
Like the other apps it is a Vite project, but every solve runs on a local Python backend, [`scripts/server.py`](scripts/server.py).
The Vite dev server proxies the page's `api/`, `files/` and `pkg/` requests to that backend on port 8767.
On GitHub Pages, the paper is available alongside instructions for running the interactive demo locally.
The app follows the site's Light/Dark setting.

`npm run dev` looks for Python in this order: `GTDYNAMICS_PYTHON`, then the repository's `.venv/bin/python`, then `python3`.
If that interpreter can't import `gtdynamics`, it says so and the page shows the same notice.

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
| Load a robot | **Model** dropdown. Every `*.urdf` under `models/urdfs` is listed. The root link of the URDF is fixed in the world. |
| Pick what to move | **End-effector frame** dropdown. It lists every GTDynamics link plus URDF frames rigidly attached to one (for example, `tool0` on the UR arms or the `*_toe` frames on the A1). It also shows how many joints move that frame. |
| Pick the solver | **IK scheme** dropdown (see below). Changing it re-solves to the current target. |
| Move the end-effector | Click and hold the **orange handle**, then drag. It moves in the plane facing the camera. IK runs continuously while you drag, warm-started from the previous solution. |
| Change the end-effector pose | **Double-click the handle.** A rotation gizmo appears and the pose panel is highlighted: drag the rings to rotate, press <kbd>W</kbd> for the axis-arrow move gizmo and <kbd>E</kbd> for rotate, and <kbd>Esc</kbd> (or double-click again) when done. You can also type an exact target (x, y, z in meters; roll, pitch, yaw in degrees, URDF convention `R = Rz(yaw)·Ry(pitch)·Rx(roll)`) and press Enter or **Solve to target**. |
| Position only vs. full pose | **Constrain orientation**. Off: only the end-effector position is solved for. On: the full 6-DoF pose is. It defaults to on for chains with 6 or more joints. |
| Accuracy and effort | **Diagnostics and solver settings** opens the results section with **Pos tol**, **Rot tol** and **Max iters** (blank uses the scheme's default). |
| Set joints directly | The **Joints** sliders run forward kinematics, and the target snaps to the new end-effector pose. **Home** sets all joints to zero, or to mid-range when zero is outside a joint's limits. Joints at a limit are shown in red. |
| Camera | Drag the background to orbit, right-drag to pan, scroll to zoom. **Fit view** frames the robot. |

The viewport shows:

- The robot meshes.
- A blue stick-figure **skeleton** of GTDynamics' link frames.
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

The Jacobian-based schemes and CCD only move the joints between the fixed base and the end-effector, and they use GTDynamics for all forward kinematics.

## Why a target can't be solved

A solve is **Solved** when both the position and orientation errors are within tolerance.
Otherwise the status panel says **Not solved** and lists the reasons it found.
It still shows the best configuration the solver reached.
The checks are:

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

When a solve succeeds, the panel still warns if the arm is near a singularity or if a joint sits at a limit.

## Model notes

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

The browser test checks the offline notice on the static build at `/` and under a project prefix.
When `gtdynamics` imports, it also starts the backend and drives a live solve.
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
| `index.html`, `src/main.js`, `src/App.vue` | Vite entry and the page markup, including the offline notice. |
| `src/explorer.js` | The three.js scene, dragging and pose editing, and the side-panel logic (`createExplorer`). |
| `src/pose.js`, `src/api.js` | Pose, joint and status helpers, and requests to the backend. |
| `src/style.css` | Styles, with light and dark palettes keyed on the shared theme. |
| `vite.config.mjs` | Proxies `api/`, `files/` and `pkg/` to the backend, also under the site's `/urdf/` prefix. |
| `scripts/server.py` | Standard-library HTTP backend. It serves URDF and mesh files and a JSON API: `GET /api/models`, `GET /api/schemes`, `POST /api/load`, `POST /api/fk`, `POST /api/ik`. |
| `scripts/ik_backend.py` | Robot model (GTDynamics + URDF XML), the IK schemes and the failure diagnostics. |
| `scripts/backend.mjs`, `scripts/dev.mjs` | Start the backend for `npm run dev`, here and in the apps launcher. |
| `tests/test_ik_backend.py` | Backend unit tests. |
| `tests/pose.test.mjs` | Node tests for `src/pose.js` and `src/api.js`. |
| `tests/app.browser.mjs` | Browser test: offline notice, and a live solve when GTDynamics is available. |
