# Kinodynamics

The app replays a coupled 50 Hz CMC-Opt solve of the SSIO G1 mid-clip turns
(78.00–97.98 s, 1,000 slices). It estimates configurations, body/world velocities,
accelerations, sole wrenches, internal reaction wrenches, and all 29 actuator
torques. The contact initializer supplies starting configurations. All other
states start at zero, including the shared IMU bias.

The computation is the student-facing example
[`kinodynamic_walk.py`](../../examples/g1_mjlab/kinodynamic_walk.py). Run from the
repository root with the built GTDynamics/GTSAM Python packages and pinned G1
model packages on `PYTHONPATH`. Enable `GTDYNAMICS_BUILD_G1_MJLAB` and
`GTDYNAMICS_BUILD_PYTHON`, then build `_g1_kinodynamics`. Add
`build/examples/g1_mjlab` alongside `build/python` and the matching GTSAM Python
path. The private extension is built with the standard wrapper’s pybind11
provider and is not installed into the GTDynamics package:

```bash
python examples/g1_mjlab/kinodynamic_walk.py --output /tmp/kinodynamics
python apps/kinodynamics/scripts/export_recording.py --result /tmp/kinodynamics
cd apps
npm run build
```

The example defaults to five iterations and accepts `--max-iterations`, `--start`, `--end`, `--data`, `--labels`, and
`--retraction-threads` (8 by default; 1 runs projections serially).
Its main saves numerical arrays and a JSON report. The exporter converts those
arrays to offline browser assets and adds independently compiled model geometry.
No optimization runs in the browser. All recording/model gzip files use Git LFS.

Articulated q/v/a kinematics, Newton–Euler balance, motor armature, reaction
wrench transmission, sole transforms, zero swing wrenches, and rectangular wrench
cones are exact constraints. Encoder, stance-pose landmark, Gal3 IMU, stance
velocity, trapezoidal configuration/velocity collocation, effort/rate and broad
acceleration terms are costs. No point contacts or GP/spline warm start are used.
No upright or ground-height costs remain in the coupled solve. The initializer
uses its shared upright/motion/height priors. At the first selected double-support
slice, translation and wrapped-yaw costs choose the world frame with unit
standard deviations (1 m and 1 rad). All six root-pose coordinates remain free
in the articulated tangent basis, allowing gravity/IMU observations to correct
the initial roll and pitch.
Earlier dense output from the initializer is a held pose guess and stays free
in the coupled solve.

CMC-Opt relinearizes the full nonlinear equality basis. Each retraction recovers
link poses/twists with native FK, fixes those q/v values through ConstVarFactor,
and projects the remaining affine acceleration/wrench equations with Gaussian QR
and ReducedALRetractor. Time blocks are eliminated chronologically, retiring each
stance landmark after its last observation and the shared bias at the end.
Per-slice projections run in a bounded TBB arena when available; serial and
parallel results are checked against each other. One previous iteration is retained for diagnostics. Compact boundary residuals
and active sets track the mode independently of diagnostic-history retention. The app stores scalar per-factor error
histories instead.

Timing separates contact initialization, graph assembly, initial feasibility,
and coupled optimization. Input/model loading, array export,
compression and browser rendering are excluded from the reported solver total.
Measurement construction and IMU preintegration are included in the total.
Feasibility restoration changes the raw zero dynamics and is included in total.
A feasible result or an objective-change stopping condition does not establish
that the objective is globally optimal.

The regenerated recording (2026-11-03, Release build, eight retraction threads)
took **78.03 s** in coupled optimization and **87.89 s** from contact
initialization through the solve. It completed five iterations, reducing the
objective from 28,986,857.82 to 9,120.02; it stopped at the iteration limit.
The final equality/inequality violation norms are 4.16e-12 and
1.29e-07. Across 21 evenly spaced slices, independent MuJoCo inverse
dynamics agrees within 8.24e-13 N in root force, 1.85e-12 Nm in root moment,
and 2.83e-12 Nm in joint torque. Every slice passes the sole-cone check
(maximum violation 9.33e-08); swing wrenches are zero. These are equation
consistency checks. Full-recording stance-origin speed RMS is
0.185 m/s, so the finite-noise stance objective still permits slip.
Reproduce the oracle with `python examples/g1_mjlab/validate_kinodynamics.py
--result /tmp/kinodynamics`. The replay includes source/binary fingerprints,
the projection metric, gauge metadata, and the separate gauge error group.
The [screenshot](docs/kinodynamics.png) shows the regenerated replay.

The page follows the shared technical-paper layout. Its abstract and complete
replay controls fit together on a laptop screen. Motivation, a synchronized
factor-graph schematic and native factor definitions precede the recorded
timings, force/velocity/torque plots, error budget and numerical settings.
Equations use the shared build-time MathJax plugin and work offline. Parameter
tables read the selected recording, including the sole support rectangles,
collocation weights, IMU assumptions and gauge. The Discussion links to Motion
Planning, which remains the final tab.

The support schedule is privileged: it uses saved max-product labels derived
from SuperOdom LiDAR-inertial foot motion and measured joint velocities. Reference
poses after initialization do not enter the coupled estimation graph. The inferred
forces and torques are model-based estimates. Dataset/model attribution is
available in the app and accompanying notices.
