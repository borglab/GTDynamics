# Dynamics

The Dynamics app displays estimated accelerations, ground reaction wrenches, and
all 29 G1 joint torques for the SSIO 78–98 s walking/turn snippet. Native CMC-Opt
solves the estimation problem offline. The browser replays body transforms,
forces, and torques at the saved timestamps.

The development window contains **57 original double-support knots** from the
best 715-state Kinematics trajectory. The Gaussian-process fit uses all those
knots and 10,996 dense encoder samples before cropping the window. Knot poses
and joint configurations remain exact. The Pose3/vector WNOA model in
`FloatingBaseInterpolator` fits body twists and joint rates; saved world COM
velocities provide additional observations. The same reusable GP implementation
supplies the Interpolation app.

Dynamics keeps these configurations and velocities fixed through
`ConstVarFactor`. Its variables include every link's body-twist derivative,
every joint acceleration, reaction wrenches, one external wrench per sole, and
actuator torques. Native `TwistAccelFactor` constraints enforce articulated
acceleration kinematics exactly. Rotor armature multiplies the **estimated**
joint acceleration in the motor-torque equality. All remaining equations are
affine, so CMC-Opt retains the elimination basis and cached affine
`ReducedALRetractor`.

Acceleration observations constrain the fitted velocity signal locally. At a
knot, the prior mean is

```text
a_hat(t) = [v_GP(t + h) - v_GP(t - h)] / (2 h),  h = 20 ms by default.
```

The stencil uses the `[-1, 0, 1]` differentiation mask. Its queries stay inside
the complete GP fit; the full trajectory's endpoints use one-sided differences
and the actual elapsed time. The development-window endpoints retain context
from the complete fit. Root and joint acceleration priors are soft; exact
force/moment balance and wrench inequalities can adjust their values.

| Acceleration prior | Per-axis standard deviation |
| --- | --- |
| Root angular body-twist derivative | 90 °/s² |
| Root linear body-twist derivative | 3 m/s² |
| Revolute-joint acceleration | 100 °/s² |

The root pose maps its inertial COM frame into world. Twists and their
derivatives are angular-first and expressed in the moving COM axes. The linear
part of a twist derivative is `d(v_body)/dt`; physical COM acceleration in body
axes is `d(v_body)/dt + omega_body × v_body`. Rotational prior scales are
specified in degrees and converted explicitly at the native factor boundary.
The recording retains both the acceleration estimates and their prior means.

The contact model uses one torque-first sole-local wrench per foot. Exact
inequalities enforce compression, an inscribed friction pyramid, rectangular
center-of-pressure bounds, and coupled yaw bounds. The original source phase
sequence preserves lift-off and recontact across unsampled intervals. Wrench-rate
priors connect observations belonging to the same uninterrupted stance.
Torque-rate costs use the actual irregular time intervals. Effort limits are
reported as diagnostics; hard effort bounds remain available in the native API.

The regenerated recording satisfies exact floating-base balance and contact
wrench inequalities at all 57 knots. CMC-Opt converged in three iterations,
taking approximately 0.75 s for native graph construction and optimization.
GP factor construction and elimination took another 0.18 s, excluding data
loading, FK sampling, and export. The result has 2,337 independent coordinates.
The error budget includes the acceleration observation costs. Per-component
RMS corrections from the GP-derived priors are approximately 1.20 °/s² for root
angular acceleration, 0.50 m/s² for root linear acceleration, and 0.39 °/s² for
joint accelerations.

Independent MuJoCo inverse dynamics agrees with the native required force,
moment, and all joint torques within 2e-12 N or Nm. Resultant-wrench and
four-corner LP feasibility checks agree at every knot. These checks establish
equation and frame consistency. Contact stationarity remains an input-quality
issue: the labeled stance feet have 0.67 m/s RMS speed in the GP fit. Rigid
contact would require further pose/velocity consistency constraints.

## Regenerate

Use the pinned G1 packages and matching GTDynamics/GTSAM Python bindings:

```bash
conda activate py312
python -m pip install -r examples/g1_mjlab/requirements-dynamics.txt
make -C build -j6 gtdynamics_py
PYTHONPATH="$PWD/build/python:$HOME/git/gtsam/build/python:$PWD/examples/g1_mjlab" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
python apps/dynamics/scripts/export_recording.py \
  --kinematics apps/kinematics-inspector/public \
  --dense-data apps/kinematics-inspector/.cache/openio-contact-source \
  --start 78 --end 98 --friction 0.8 --max-iterations 50
```

The exporter selects `comparison.json`'s winner automatically. The
`--acceleration-step-ms`, `--base-angular-acceleration-sigma-degrees`,
`--base-linear-acceleration-sigma`, and `--joint-acceleration-sigma-degrees`
arguments configure the acceleration observations. GP noise assumptions,
source fingerprints, factor errors, corrections, and timing scopes are saved
with the replay.

The default solve enforces exact floating-base balance. The explicit
`--soft-base-balance` option permits a residual wrench and publishes its force
and moment. Its per-axis scales are controlled by `--base-force-sigma` and
`--base-moment-sigma`. Configuration and velocity inputs remain fixed.

## Validate

With the same Python environment and `PYTHONPATH`:

```bash
make -C build -j6 testFixedKinematicsDynamics.run testFloatingBaseInterpolator.run
python -m unittest discover -s examples/g1_mjlab -p test_fixed_kinematics.py -v
python apps/dynamics/scripts/validate_recording.py
cd apps
npm test --workspace dynamics
npm run build
```

The MuJoCo oracle uses the **published optimized accelerations**, disables
contact constraints and passive damping/friction, and subtracts sole-wrench
loading when comparing actuator torques. LPs serve only as feasibility checks.
The native CMC result supplies every published force and torque.

The browser selects the nearest retained knot during playback. Connecting lines
between force/torque samples guide the eye across unsampled intervals. Compiled
canonical-model geometry and the replay use Git LFS. Model and SSIO attribution
remain available below the viewer. Browser checks are
`node dynamics/tests/app.browser.mjs` and
`node dynamics/tests/navigation.browser.mjs`, with `PLAYWRIGHT_MODULE` set when
Playwright is installed outside the npm workspace.
