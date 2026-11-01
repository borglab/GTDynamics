# Kinematics

Inspect the complete physical G1 clip `10871_humanoid_20260520_223451` from
SSIO / Open-IO, sampled at 5 Hz: 1,100 slices over 219.8 seconds. The only
initializer is the native contact initializer. The separate **Contact vs IK**
app compares the two initialization methods on short snippets.

The default enables Gal3 IMU aiding and a soft return-to-start position prior.
The viewer fits the entire trajectory; Follow robot switches to a close view.
A single large canvas shows the estimate, the dashed SuperOdom LiDAR-inertial
reference, and optional foot-slip locations. Scrub time, contact phases, and
all saved CM-Opt iterations. Runs stop after at most ten iterations; the viewer
explicitly distinguishes the iteration limit from a solver stopping criterion.

## Contact weights and slip

Twelve native solves are retained: stationary-contact displacement sigmas
50, 20, and 5 mm, each with IMU aiding and loop closure enabled or disabled.
Only the stationary-contact weight changes across sigma settings. Sigma is
per world axis per adjacent 0.2-second slice interval, not a velocity sigma.
Height costs, joint measurements, motion regularization, and initializers use
the same settings in each comparison.

Slip is computed offline from continuing sole corners. Per foot and interval,
the recording stores the previous/current world positions of the corner with
the largest displacement. Yellow-to-red points mark intervals above the chosen
threshold; selected-frame arrows show direction, enlarged for visibility.
No-contact or newly touching feet have no continuing-contact displacement.

Tightening the contact cost reduces slip but can increase reference error.
For the IMU-aided, loop-closed runs at the final saved iteration:

| Contact sigma | Stance displacement RMS | Reference position RMS |
| --- | --- | --- |
| 50 mm | 23.44 mm | 14.43 m |
| 20 mm | 15.42 mm | 23.73 m |
| 5 mm | 7.10 mm | 24.11 m |

These runs retain substantial drift and are inspection examples.

## Factor graph and error budget

Below the viewer, the table lists the actual native cost factors by type:
joint measurements, contact heights, stationary contacts, root motion,
initial pose, Galilean IMU preintegration, shared bias, initial velocity, and
return-to-start position (the last four appear when enabled). It reports
counts, residual dimensions, noise settings, and each group's contribution to
`0.5 * ||whitened residual||²` at the selected iteration. Native graph errors
are checked to sum to the entire objective before exporting.

Kinematic equalities are enforced by CM-Opt, reported separately with count,
residual dimension, and violation L2; they are not soft objective costs.
There are no inequalities. The default has 44,059 soft factors and 31,900
six-dimensional kinematic equalities.

`PoseKey` is world_T_COM; velocities are world-frame linear velocities of the
root link COM. `COM_T_IMU` handles sensor mounting and the centripetal lever arm;
the GTSAM correction omits the angular-acceleration lever-arm term. One shared
sensor-frame bias is estimated. The gyro prior is the raw stationary mean over
0–25 s, with sigma 0.0001 rad/s. Accelerometer bias retains sigma 0.5 m/s²;
its large horizontal values are coupled to tilt and the flat-sole assumption,
and are not a validated physical calibration.

Loop closure uses `PoseTranslationPrior3D` at the final COM position, targeting
the tightly anchored start, with world XYZ sigmas `[0.30, 0.30, 0.05]` m and no
orientation prior. It is a manually supplied observation for this example;
source SuperOdom loop-closure settings are unspecified.

## Generate and preview

Use the native `py312` environment with matching GTDynamics/GTSAM bindings and
MuJoCo. From the repository root:

```bash
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_apps.py
```

`--only trajectory` or `--only snippets` regenerates one app; `--jobs` controls
export concurrency. Every run calls the existing example with CLI arguments.
The full app exports contact-only smoothing histories and never runs IK.
Factor errors, foot-slip vectors, and all robot transforms are computed offline.
Geometry and full-clip raw IMU samples are shared across variants. The browser
loads compressed replay metadata, caches recent results with a memory limit,
and performs no optimization or kinematics.

From `apps`, run `npm run dev` for the hub, or
`npm run dev --workspace kinematics-inspector` (port 3005). Kinematics is the last
tab. The stable app ID / URL remains `kinematics-inspector`.

## Sources and validation

The dataset is [Open-IO G1 Inertial Odometry](https://huggingface.co/datasets/Open-IO/humanoid-g1-odometry),
released with [SSIO](https://ssio-imu.github.io/). The website retains authors,
license links, source revision/checksums, and modification notices. SuperOdom
is a LiDAR-inertial odometry reference, not independent motion-capture ground
truth. Only one constant translation changes its world origin; its vertical
and time-dependent drift is retained. Contact labels are inferred with a
rolling floor for labeling only, while the smoother uses a fixed ground plane.

Data tests validate factor-budget sums, all variants' graphs, equality
feasibility, exact loading-bundle coordinates, and recorded slip vectors.
Browser tests cover both apps, toggles, time/iteration replay, slip overlays,
large and mobile layouts, project-prefix URLs, and hub order/navigation.
