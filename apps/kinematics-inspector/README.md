# Kinematics

Inspect the complete physical G1 clip `10871_humanoid_20260520_223451` from
SSIO / Open-IO, at 1,135 contact-selected slices from 0.02 to 219.92 seconds. The only
initializer is the native contact initializer. The separate **Contact vs IK**
app compares the two initialization methods on short snippets.

The default enables Gal3 IMU aiding and a soft return-to-start position prior.
The viewer fits the entire trajectory; Follow robot switches to a close view.
A single large canvas shows the estimate, the dashed SuperOdom LiDAR-inertial
reference, and optional foot-slip locations. Scrub time or jump between contact
phases while viewing either the initial estimate or the final CM-Opt result.
The Estimate selector switches the robot, path, slip markers, factor errors,
position plots, and bias display together. Initial means the contact initializer
output before CM-Opt, with finite-difference velocities and the bias prior mean.
Intermediate iteration histories are kept only in Contact vs IK. The offline
solver still uses a ten-iteration cap. The page displays completed iterations
and recorded wall time for CM-Opt manifold setup, optimization, and solution
extraction. Initialization, graph construction, diagnostics, and replay export
are excluded. Exports run without iteration callbacks when retaining only the
initial/final snapshots; timings are individual runs, not repeated benchmarks.

## Contact-selected slices and IMU intervals

The full clip is first loaded on the 50 Hz contact-detection grid. Native
`InitializeSolutionContacts` uses `contactBuffer=1`: each continuous stance
loses one source sample (20 ms) at each end. `contact_init.Result.slices()`
returns exactly the slices used by that solve. The example reuses their native
poses and joint measurements for the smoothing seed and estimates only at those
slices, reindexing solver keys while preserving source indices and timestamps.
No dense interpolation states enter the smoothing graph.

The detector requires four samples (80 ms) per stance so two interior samples
survive the buffer. This excludes one previous three-sample detection. Selection
also retains phase boundaries with trusted contacts and supplements long stances.
The resulting intervals range from 0.02 to 1 second. Three lift-off intervals
would look like continuing contact after reindexing; their stationary factors
and slip measurements are excluded. The smoother otherwise uses the source
contact specifications.

`timestamps` are seconds from the input clip start. `dt` is the source grid
period, not the spacing of the reduced trajectory. Each of the 1,134 Gal3 pims
integrates precisely between adjacent timestamps, including fractional native
IMU intervals. Velocity seeds, playback, plot axes, and plot selection use the
same times. The initial gauge anchors the first retained slice; the initial
20 ms and final trimmed samples are not extrapolated into the viewer.

## Contact weights and slip

Twelve native solves are retained: stationary-contact displacement sigmas
50, 20, and 5 mm, each with IMU aiding and loop closure enabled or disabled.
Only the stationary-contact weight changes across sigma settings. Sigma is
per world axis per adjacent selected slice interval, not a velocity sigma.
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
| 50 mm | 30.21 mm | 26.83 m |
| 20 mm | 18.01 mm | 31.54 m |
| 5 mm | 8.95 mm | 25.51 m |

These runs retain substantial drift and are inspection examples.

## Factor graph and error budget

Below the viewer, the table lists the actual native cost factors by type:
joint measurements, contact heights, stationary contacts, root motion,
initial pose, Galilean IMU preintegration, shared bias, initial velocity, and
return-to-start position (the last four appear when enabled). It reports
counts, residual dimensions, noise settings, and each group's contribution to
`0.5 * ||whitened residual||²` at the selected estimate. Two bar columns show total
error and mean error per factor (total divided by count, not residual dimension).
Each column has its own linear scale from zero to its maximum. Native graph
errors are checked to sum to the entire objective before exporting.

Kinematic equalities are enforced by CM-Opt, reported separately with count,
residual dimension, and violation L2; they are not soft objective costs.
There are no inequalities. The default has 45,345 soft factors and 32,915
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

## Position comparison

Four time plots appear below the error budget, each spanning the page width:

- Pelvis world z: the MuJoCo pelvis **body origin**, in metres, rather than
  the link COM used by the IMU factor.
- Foot body x/y/z: the ankle-roll body origin expressed in the **pelvis frame**.
  The estimate and reference each use their own pelvis frame. A left/right
  selector chooses the foot without adding more plots.

Solid curves are the selected estimate; dashed curves use SuperOdom LiDAR-inertial
poses and measured joints. These are kinematic reference foot positions, not
independent tracked foot measurements. Each plot has time/position axes, shares
its scale between estimate and reference, and shows the selected walking time.
Click a curve to scrub the viewer. Plotly supports drag-to-zoom, scroll zoom,
and toolbar panning. The time range is linked across all four plots; vertical
zoom is independent for each. Reset plot zoom restores both axes in every plot.
The basic Plotly bundle loads on demand after the replay data. Position series
are computed offline from saved MuJoCo display poses, quantized to 1 micrometre;
pelvis quaternions are normalized before transforming foot positions.

## Generate and preview

Use the native `py312` environment with matching GTDynamics/GTSAM bindings and
MuJoCo. From the repository root:

```bash
PYTHONPATH=../gtsam/build/python:build/python \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python apps/kinematics-inspector/scripts/export_apps.py
```

`--only trajectory` or `--only snippets` regenerates one app; `--jobs` controls
export concurrency. Use `--only trajectory --jobs 1` to refresh recorded wall
times without competing exports, as used for the checked-in snapshots.
Every run calls the existing example with CLI arguments.
The full app exports contact-initialized initial/final snapshots and never runs IK.
Factor errors, foot-slip vectors, and all robot transforms are computed offline.
`--final-only` retains the initial/final snapshots without recording intermediate
iterates. Repacking an older final-only archive cannot recover its missing initial
estimate; regeneration is required. Saved integer pose/joint/velocity/slip blocks and native diagnostics are preserved exactly.
Geometry is shared across variants. Raw IMU samples remain in the offline input
cache; they are not published. Metadata is stored only in the compressed bundles
the viewer reads and offers for download. The browser requests only
`final.json.gz`, `final.bin.gz`, and the shared model by default. Selecting Initial
loads `initial.json.gz` and `initial.bin.gz` on demand. It caches recent results,
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
Position curves are checked against independent quaternion transforms of the
saved estimate/reference body poses.
