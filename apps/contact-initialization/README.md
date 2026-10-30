# Contact Initialization

A reusable **C++ GTSAM alternative to inverse-kinematics initialization**,
illustrated with Unitree G1 motion. Measured joint geometry is frozen at contact
boundaries. GTSAM estimates only event root poses and shared stance landmarks.

The browser loads recorded native iterations and timings. It performs no graph
initialization or optimization. Both the 8-second policy walk from [PR #446](https://github.com/borglab/GTDynamics/pull/446)
and a downloaded 30-second [G1 Moves step-touch capture](https://huggingface.co/datasets/exptech/g1-moves)
are selectable. The latter is retargeted human motion, with explicitly heuristic
contact labels and three disconnected support components.

![G1 contact initialization app](docs/preview.png)

## Run the app

With Node.js 22 or newer, from the repository root:

```bash
cd apps
npm ci
npm run dev --workspace contact-initialization
```

Open <http://127.0.0.1:3003/>. For the shared app site, run `npm run dev` from
`apps` and open <http://127.0.0.1:3000/>. Checked-in data and native recordings
make this app independent of a native server, Python, model downloads, or a GPU
at playback time. All production asset URLs are relative.

## Reusable initialization API

The GTSAM-only core is
[`gtdynamics/utils/ContactInitializer.h`](../../gtdynamics/utils/ContactInitializer.h).
It accepts FK event observations with persistent stance IDs, returns poses,
landmarks, `gtsam::Values`, diagnostics, and phase timings, and supports caller
anchors and key conventions. Its implementation has no JSON or reference-data
dependency.

The GTDynamics adapter is:

```cpp
gtsam::Values seeds = initializer.InitializeSolutionContacts(
    robot, rootLinkName, initialRootCOMPose,
    measuredJoints, sampleIndices, contactEpisodes, dt, options, &diagnostics);
```

It matches the robot/root/initial-pose arguments and `Values` return type of
`InitializeSolutionInverseKinematics`. The additional measured-joint trajectory
and episode schedule are necessary inputs. The adapter freezes root-relative FK
at episode boundaries, runs the sparse contact initializer, interpolates root
seeds between events, and expands them through FK at requested sample indices.
It preserves measured joint angles and velocities and uses normal GTDynamics
`PoseKey(link_id, sample)` keys. Contact coordinates are relative to link COMs.

An estimator that accepts a root pose seed can consume those poses while keeping
its own IMU twists, velocities, and bias initialization. The C++ adapter rejects
disconnected schedules rather than silently joining unrelated gauges. This is
a compatible pose-seeding workflow, not a literal replacement for IK's desired
pose inputs. Python bindings for the new C++ API are not included.

See [the integration example and API contract](docs/initializer-api.md).

## Factor graph and native solve

For a material sole point `a_j`, FK at event `i` supplies a fixed measurement
`p_ij = root_T_foot(q_i) a_j`. Each touchdown creates a fresh stance landmark
set; all events within that stance share it. The objective is

```text
minimize sum_(i,j) || R_i p_ij + t_i - c_j ||²
```

Joint angles and individual link poses are not optimization variables. The
first pose of each connected component is held constant by excluding its key
from the optimized state; its contact factors become unary landmark factors.
No trajectory, attitude, IMU, or reference factors are added.

The initializer first traverses the contact graph, registering frames against
already placed contacts using a small rigid SVD fit. Then
`gtsam::LevenbergMarquardtOptimizer` jointly updates all free event `Pose3` and
contact variables, with analytic Jacobians and multifrontal Cholesky. Numerical
LM diagonal damping permits point-mode solves but adds no measured attitude
information. The recorded cap is 100 LM iterations; stopping tolerances are
relative objective improvement `1e-8` or absolute improvement `1e-12`.

Three explicit modes are recorded:

| Mode | Contact variables | Assumption |
| --- | --- | --- |
| Sole patch + level ground, default | Three `Point2` xy landmarks per stance, fixed common z | Stationary sole patch on known level ground |
| Contact-only sole patch | Three `Point3` landmarks per stance | Stationary rigid sole, with no terrain constraint |
| One material point | One `Point3` per stance | Point support; rotations remain ambiguous |

The plane is an exact parameterization, not a soft penalty. First-sole FK sets
the initial tilt and the plane height in the chosen coordinate system. Given
fixed world landmarks, one point provides pose rank 3, two distinct points rank
5, and three noncollinear points rank 6. Local ranks do not certify global graph
observability or physical validity of the no-slip assumption.

G1 rocks its feet about 11 degrees per stance on median in the original rollout.
That model mismatch accumulates through contact-only rigid soles. A small
contact residual is not a guarantee of trajectory accuracy.

## Native timings and accuracy

The checked-in measurements used **Intel Core i7-14700F, Linux x86_64, Clang 18,
Release, GTSAM 4.3.2**, and `MULTIFRONTAL_CHOLESKY`. Each model has 2 warmups and
11 fresh runs. The app reports medians, minima, p95, all raw samples, iteration
counts, build details, and exact source/data hashes. The p95 is the empirical
nearest-rank quantile; with 11 repetitions it is the maximum sample.

| Sequence / mode | Native LM median | Initializer total median | LM iterations | 3D contact RMS |
| --- | ---: | ---: | ---: | ---: |
| 8 s policy / patch + ground | 116.82 ms | 117.08 ms | 69 | 11.85 mm |
| 8 s policy / contact-only patch | 198.28 ms | 198.56 ms | 100, cap reached | 11.49 mm |
| 8 s policy / one point | 4.77 ms | 4.88 ms | 7 | 1.89 mm |
| 30 s capture / patch + ground | 4.91 ms | 5.03 ms | 10 | 21.30 mm |
| 30 s capture / contact-only patch | 11.69 ms | 11.83 ms | 23 | 18.57 mm |
| 30 s capture / one point | 2.76 ms | 2.81 ms | 8 | 15.86 mm |

The longer capture has fewer event frames (35 versus 75), different motion and
contact evidence, and disconnected components; its shorter solve time is not a
duration-scaling benchmark. These measurements do not establish a speedup over
an IK initializer.

The core timer separates rigid registration seed, graph construction, and LM.
Optimization includes LM construction, linearization, damping, and sparse solves.
Repeated timing runs disable history capture; a separate run records snapshots.
FK extraction, dataset adaptation, reference evaluation, dense adapter expansion,
file IO, JSON, network, and rendering are excluded. Total time covers the sparse
initializer, not an end-to-end estimator initialization. Individual phase
medians need not sum exactly to the total median.

The 8-second grounded result has root position RMSE 0.252 m and orientation RMSE
14.78 degrees, after aligning only the first event. Contact-only patches give
1.741 m / 96.42 degrees and reach the iteration cap. The point-mode displayed
solution gives 0.285 m / 7.58 degrees, but its attitude remains underconstrained.
Recorded root poses are entirely removed from the native request and used only
for display and evaluation.

For the 30-second capture, each support component is aligned independently at
its first event. The grounded component-relative RMSE is 0.152 m / 9.85 degrees;
this is not accuracy of one globally recovered 30-second trajectory. No factor
or rendered trajectory line connects the independent components.

## Build C++ and reproduce recordings

The benchmark builds on its own with GTSAM and Boost 1.75 or newer:

```bash
cmake -S apps/contact-initialization/backend -B /tmp/contact-init-build \
  -DCMAKE_BUILD_TYPE=Release -DGTSAM_DIR=/path/to/lib/cmake/GTSAM
cmake --build /tmp/contact-init-build -j2
ctest --test-dir /tmp/contact-init-build --output-on-failure
python3 apps/contact-initialization/scripts/export_native_recordings.py \
  --optimizer /tmp/contact-init-build/contact_initializer
```

On hosts with multiple CUDA installations, an installed GTSAM may require
`-DCUDAToolkit_ROOT=/matching/cuda/path` for its transitive dependencies. The
initializer and measured solves use the CPU.

Alternatively enable `GTDYNAMICS_BUILD_CONTACT_INITIALIZATION_WEBDEMO=ON` in a
normal GTDynamics build. The core and adapter are always included in the
GTDynamics library; this option adds the benchmark executable and app tests.
`testContactTrajectoryInitializer` exercises the installed-library API's COM
frames, original sample keys, measured joints/velocities, dense expansion,
input checks, and disconnected-schedule rejection.

The native executable reads one JSON request from stdin or an input filename.
It writes graph metadata, actual LM iterations, benchmark statistics, and build
details. The Python exporter only orchestrates native runs and packages them;
it does not solve the graph. It strips the `reference` and metadata fields from
the request and records exact C++ source, binary, and dataset SHA-256 hashes.

Dataset generation, pinned downloads, contact thresholds, attribution, and
licenses are documented in [DATA.md](DATA.md). The longer capture's height/speed
heuristic does not create force measurements or establish no-slip contact.

## App verification

```bash
cd apps
npm test
npm run build
PLAYWRIGHT_MODULE=/absolute/path/to/playwright/index.mjs \
  node contact-initialization/tests/app.browser.mjs
```

The app tests independently recompute recorded residuals, verify native source
and data hashes, anchors, hard floor constraints, history, and timing statistics.
Browser regression covers both sequences, all modes, replay, timing display,
JSON export, camera/overlays, mobile layout, and `/GTDynamics/` asset URLs.
