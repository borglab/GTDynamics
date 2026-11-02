# Contact vs IK

Compare contact and inverse-kinematics initialization, then CM-Opt smoothing,
on three ten-second physical Unitree G1 snippets from the SSIO / Open-IO clip
`10871_humanoid_20260520_223451`:

| Source window | Motion | Slices |
| --- | --- | --- |
| 40–50 s | Walking | 29 retained |
| 100–110 s | Corner | 27 retained |
| 204–214 s | Turn | 27 retained |

Both methods solve the same phase/trajectory factor graph. Select Initialization
to inspect the sparse contact solve or each slice's IK iterations; select
CM-Opt smoothing to inspect the common objective. All native outer iterations
are retained, with a ten-iteration CM-Opt limit. IMU aiding is optional and uses
the same calibrated gyro prior for both methods. The initial 0–25 s stationary
interval supplies this prior even though it is outside these snippets.

Contact labels come from the binary-foot max-product graph on 50 Hz source
measurements, with joint-velocity unaries, duration priors, and walking constraints.
Short double-stance phases contribute a central slice; longer phases retain
buffered endpoints and evenly spaced interior slices at most 200 ms apart.
This is the default shared G1 policy, configurable with `--max-slice-interval-ms`.
Both initializers and smoothing use exactly these slices and actual timestamps.
No dense expansion or interpolated solve states are exported. Labels use
privileged reference motion, not force measurements. The reference is SuperOdom LiDAR-inertial odometry,
not independent motion capture. Data and attribution are licensed CC BY 4.0;
the pinned robot model has its separate license.

The contact initializer now shares Contact Initialization's corrected preset:
upright starting values, contact observations **1 cm / 90°**, and three soft
priors: pelvis upright **15°**, height **10 cm**, and weak motion **10 m / 180°**.
Sole-normal priors are off in this initializer. A constant world translation puts
the mean active sole height at the first fixed root pose at zero, using only the
supplied gauge and measured-joint FK. Both initializers, smoothing, and the
reference overlay use that same frame. IMU samples are unchanged. Native
no-decrease stops are labeled stalled, not converged. The contact solve allows
200 iterations; CM-Opt remains capped at ten.

The browser initially loads only smoothing snapshots. Initializer histories load
on demand, while model geometry and recent results are reused. All numerical
work, including transforms and residuals, is performed offline.

From `apps`: `npm run dev --workspace contact-vs-ik` (port 3004), or use the
shared hub. Regenerate with the configured native Python environment:

```bash
python apps/kinematics-inspector/scripts/export_apps.py --only snippets
```

The exporter calls `examples/g1_mjlab/kinematic_walk.py`; timing comparisons remain
in the example's existing timing driver. Shared rendering and loading code lives
in `apps/shared/kinematics`.

Both smoothing starts now use shared world stance-pose landmarks and the native
`ContactPoseFactor` (10 mm translation, 90° rotation). Kinematic equalities couple
foot poses to joint angles. Smoothing upright, height, normal, and weak motion
priors are off for these comparison recordings; the contact initializer retains
its corrected priors. The Kinematics app separately exposes upright/height
smoothing ablations on the full trajectory.

IMU pims cover the actual irregular intervals between selected slices. All
retained slices have both sole contacts; hidden single-stance phases still
preserve each material foothold's identity through lift-off and recontact.
