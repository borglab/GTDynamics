# Contact vs IK

Compare contact and inverse-kinematics initialization, then CM-Opt smoothing,
on three ten-second physical Unitree G1 snippets from the SSIO / Open-IO clip
`10871_humanoid_20260520_223451`:

| Source window | Motion | Slices |
| --- | --- | --- |
| 40–50 s | Walking | 500 at 50 Hz |
| 100–110 s | Corner | 500 at 50 Hz |
| 204–214 s | Turn | 500 at 50 Hz |

Both methods solve the same phase/trajectory factor graph. Select Initialization
to inspect the sparse contact solve or each slice's IK iterations; select
CM-Opt smoothing to inspect the common objective. All native outer iterations
are retained, with a ten-iteration CM-Opt limit. IMU aiding is optional and uses
the same calibrated gyro prior for both methods. The initial 0–25 s stationary
interval supplies this prior even though it is outside these snippets.

Contact labels use the full clip's rolling-floor detector at 50 Hz. Unsupported
samples and disconnected contact components remain explicit; weak native motion
priors connect components for contact initialization. Labels are heuristic,
not force measurements. The reference is SuperOdom LiDAR-inertial odometry,
not independent motion capture. Data and attribution are licensed CC BY 4.0;
the pinned robot model has its separate license.

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
