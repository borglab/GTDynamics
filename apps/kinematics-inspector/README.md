# Kinematics

Inspect the complete physical G1 clip `10871_humanoid_20260520_223451` from
SSIO / Open-IO, at 715 double-stance slices from 0.02 to 219.92 seconds. The only
initializer is the native contact initializer. The separate **Contact vs IK**
app compares the two initialization methods on short snippets.

The default selects the saved configuration with the lowest SE(3)-aligned
pelvis-position RMS: foot-height prior, Gal3 IMU aiding, and return-to-start
position prior enabled; upright prior disabled (8.244 m aligned pelvis RMS).
Reference alignment starts on.
The comparison table ranks all 16 configurations and reports physical residuals,
total objective, and every factor type's total and mean cost. View selects that
row's four checkbox settings. A small offline `comparison.json` supplies the
table; other trajectories are loaded only when selected.
The viewer fits the entire trajectory; Follow robot switches to a close view.
A single large canvas shows the estimate, the dashed SuperOdom LiDAR-inertial
reference, and optional foot-slip locations. Scrub time or jump between contact
phases while viewing either the initial estimate or the final CM-Opt result.
The Estimate selector switches the robot, path, slip markers, factor errors,
position plots, and bias display together. Initial means the contact initializer
output before CM-Opt, with finite-difference velocities and the bias prior mean.
Intermediate robot-pose histories are kept only in Contact vs IK. Kinematics
retains small scalar error histories and uses a 50-iteration smoothing cap.
The page displays completed iterations and recorded wall time for CM-Opt manifold setup, optimization, solution
extraction, and per-iteration factor-error evaluation. Initialization, graph
construction, and robot replay serialization are excluded. Only initial/final
robot poses are retained; timings are individual runs, not repeated benchmarks.

The **Aligned reference (SE(3))** checkbox applies one rigid least-squares fit
from reference to the displayed estimate's pelvis body positions, with equal
weight per retained slice. Scale stays exactly one and reflections are excluded.
The fit is computed offline separately for initial/final estimates and each
configuration. It transforms the ghost robot (positions and orientations),
dashed path, and world-coordinate reference plots. Foot-in-pelvis curves are
unchanged because the common world transform cancels out. The reference RMS
readout uses pelvis body origins and switches between original and aligned
frames. Solver states, factor errors, IMU, and ground geometry are unchanged.
Toggling alignment does not reload data or reset the camera. A rank-deficient
path may have a nonunique optimal rotation; singular values are retained in
the exported fit metadata.

## Contact-selected slices and IMU intervals

The full clip is loaded at 50 Hz (10,998 source slices). The example runs the
same binary-foot max-product graph as Contact Labels: displacement factors,
measured joint-velocity unaries, duration priors, and hard support/transition
constraints. It produces 787 contiguous phases: left, right, or double stance.
World sole displacements use the privileged SuperOdom reference; the resulting
schedule is an offline target, not a proprioceptive contact detector.

`initialization_slices` reuses native `doubleSupportSlices` selection for the
394 double-stance phases. A short phase
(sample count × 20 ms ≤ 200 ms) contributes its central slice; a longer phase
contributes evenly spaced source slices between its first and last samples after
a one-slice inward buffer. `--max-slice-interval-ms` defaults to 200; intervals
within each retained phase never exceed that limit and differ by at most one
source sample. This trial gives 715 slices, compared with 421 using endpoints
alone. Single-stance phases still determine material-foothold identity:
the continuing foot shares its landmark, while lift-off/recontact starts a new
one. The native pose initializer receives the original phase sequence plus
explicit selected slices, with `interpolateOutput=False`. Its result supplies
the exact indices and poses reused by smoothing. No interpolated states are
created in either solve.

Contact initialization uses the same corrected preset as Contact Initialization:
1 cm / 90° observations, upright starting values, and soft priors for pelvis
upright (15°), contact height (10 cm), and weak zero motion (10 m / 180°).
Sole-normal alignment is off in initialization. The first retained source pose
fixes the gauge. A constant vertical shift, derived from its measured-joint sole
FK, makes the mean initial contact height zero in both estimation and reference
replay. No later reference pose determines this shift. Configuration uses degrees
and converts at the native API boundary. The initializer allows 200 iterations
and reports stalled separately from converged; CM-Opt allows 50 iterations.

The reduced trajectory retains `sourceIndices` and actual timestamps. Its 393
hidden lift-offs start new stance landmarks and are excluded from slip metrics.
The timeline displays the complete red/green/yellow source schedule; clicking
jumps to the nearest estimated slice. The viewer and plots show saved estimates
only; line segments between samples are visualization, not additional states.

Intervals now range from 0.06 to 0.88 seconds, rather than up to 25.58 seconds.
Gaps across omitted single-stance phases can exceed the within-phase 200 ms
limit. Each of the 714 Gal3 pims integrates precisely between adjacent retained
timestamps, including fractional native IMU intervals. Velocity seeds, playback,
and plots use those same times. `dt` describes the source grid, not the reduced
spacing. The first retained pose uses the reference gauge at that exact source slice; the
initial 20 ms and final trimmed samples are not extrapolated.

## Shared stance poses and optional priors

Sixteen native solves are retained: upright and height priors each on/off,
combined with IMU aiding and loop closure each on/off. All share exactly the
same contact initialization and 1 cm / 90° observation noise. Prior switches
change smoothing only; the initializer retains its upright, height, and weak
motion priors. Smoothing has no weak motion or sole-normal prior.

There are 395 free world-frame stance `Pose3` landmarks and 1,430
`ContactPoseFactor` observations. The residual is
`Log((world_T_footCOM * COM_T_sole)^-1 * world_T_stance)`, the same native factor
as initialization, but attached to optimized foot-link poses instead of frozen
root-relative FK. Kinematic equalities couple the feet to root poses and joint
angles. A stance landmark persists across phase changes until that foot lifts.
The optional height prior constrains each landmark's world Z once (10 cm);
optional root upright uses the initializer's `UprightFactor` at 15°, leaving yaw
and position free. Each landmark starts at its first predicted sole pose and
remains a free optimization variable.

The graph table reports these actual factors and costs. No adjacent
`ContactPoseEqualityFactor` remains in this example. The previous 5/20/50 mm
replay variants are obsolete and removed. They remain in the preceding commit.

Slip is still computed offline from continuing sole corners. Per foot and
interval, the recording stores the previous/current world positions of the
corner with the largest displacement. Yellow-to-red points mark intervals above
the chosen threshold; arrows show direction, enlarged for visibility. Newly
touching feet have no continuing-contact displacement. This diagnostic is
independent of the contact factor's observation residual.

The initializer has 17.83 m reference-position RMS over the denser selection and stops after 34
iterations without accepted decrease (**stalled**). Neither the new graph nor
the 50-iteration smoothing cap establishes an accurate global trajectory.

## Optimization error history

The full-width Plotly graph shows each factor group's objective and their total
at every accepted CM-Opt iteration, starting with the feasible iteration-zero
state. The horizontal axis switches between iteration and elapsed solve time in
seconds. Errors are sums of `0.5 * ||whitened residual||²` over all trajectory
slices in that group; equality constraints are enforced by CM-Opt and excluded.
The default logarithmic vertical scale omits exactly zero costs; switch to
linear to include zero. Legend entries toggle curves; drag/scroll zoom and the
Plotly toolbar remain available.

Both initial and final replay bundles include the same scalar history, without
intermediate body-pose arrays. Times include manifold setup and factor-error
evaluation and exclude initialization and final replay serialization. The
50-iteration limit applies to Kinematics; Contact vs IK remains at ten.

The IMU-plus-loop-closure run with both smoothing priors off reaches the
50-iteration limit. Its total
objective falls from 109,339.45 at the feasible start to 2,393.01 at iteration 50.
Reference-position RMS is 13.85 m over the 715 retained slices. On the original
421 slices, pelvis-position RMS changes from 16.54 to 16.12 m; the initializer's
RMS is essentially unchanged (20.17 versus 20.19 m). Comparing the all-slice RMS
alone overstates the improvement because additional stationary samples change
the weighting. Contact-origin slip RMS is 6.61 mm and sole-height RMS is 151.01 mm
with the height prior disabled. Large drift remains;
these are capped optimization results, not a convergence claim.

## Factor graph and error budget

Below the viewer, the table lists the actual native cost factors by type:
joint measurements, shared stance pose observations, optional landmark height
and root upright priors,
initial pose, Galilean IMU preintegration, shared bias, initial velocity, and
return-to-start position (the last four appear when enabled). It reports
counts, residual dimensions, noise settings, and each group's contribution to
`0.5 * ||whitened residual||²` at the selected estimate. Two bar columns show total
error and mean error per factor (total divided by count, not residual dimension).
Each column has its own linear scale from zero to its maximum. Native graph
errors are checked to sum to the entire objective before exporting.

Kinematic equalities are enforced by CM-Opt, reported separately with count,
residual dimension, and violation L2; they are not soft objective costs.
There are no inequalities. The default has 23,278 soft factors and 20,735
six-dimensional kinematic equalities.

`PoseKey` is world_T_COM; velocities are world-frame linear velocities of the
root link COM. `COM_T_IMU` handles sensor mounting and the centripetal lever arm;
the GTSAM correction omits the angular-acceleration lever-arm term. One shared
sensor-frame bias is estimated. The gyro prior is the raw stationary mean over
0–25 s, with sigma 0.0001 rad/s. Accelerometer bias retains sigma 0.5 m/s²;
its large horizontal values are coupled to tilt and orientation uncertainty,
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
times without competing exports, for an isolated timing run. Checked-in exports use three concurrent jobs.
Every run calls the existing example with CLI arguments.
The full app exports contact-initialized initial/final snapshots and never runs IK.
Factor errors, foot-slip vectors, and all robot transforms are computed offline.
`--final-only` retains the initial/final snapshots without recording intermediate
iterates. Repacking an older final-only archive cannot recover its missing initial
estimate; regeneration is required. Saved integer pose/joint/velocity/slip blocks and native diagnostics are preserved exactly.
Geometry is shared across variants. Raw IMU samples remain in the offline input
cache; they are not published. Metadata is stored only in the compressed bundles
the viewer reads and offers for download. The browser requests the 15 KB
comparison summary plus only the best variant's `final.json.gz`, `final.bin.gz`,
and the shared model by default. Selecting Initial
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

Recorded full-trajectory replay with the SE(3)-aligned reference:

![Kinematics trajectory and aligned reference](docs/kinematics.png)

Scalar objective histories through 50 CM-Opt iterations:

![Objective error by factor type and total](docs/convergence.png)
