# Interpolation

This offline app follows Kinematics. It fits a floating-base Gaussian process to
the comparison winner and the matching dense encoder recording, then replays
native FK and velocities. It is independent of the Dynamics app.

`FloatingBaseInterpolator` is a reusable C++ class for floating-base robot trees
with scalar joints. It uses GTSAM's `WnoaMotionFactor<Pose3>` and
`Interpolator<Pose3>` for the base. GTSAM's current WNOA templates assume a fixed
dimension (`2 * traits<T>::dimension`), so the single runtime-sized joint-vector
GP reuses GTDynamics' existing `GPLinearPrior` and `GPLinearInterpolator`.
There is no G1 joint-count specialization or separate Python implementation.

Each problem `add…` call immediately adds its factors to a Gaussian graph; raw
measurement vectors are not kept separately. Its const `solve()` returns a separate
`FloatingBaseInterpolation` with interpolation, native FK, joint-order, knot-time,
and diagnostic queries:

```python
problem = gtd.FloatingBaseInterpolator(robot, base_link, base_psd, joint_psd, sigmas)
problem.addKnot(time, world_T_base_COM, joint_angles)
# Add further knots, dense encoders, and optional base-velocity observations.
result = problem.solve()
state = result.interpolate(query_time)
values = result.values(query_time)
```

Graph assembly time (successful add calls) and solve time (elimination, result
construction, and diagnostics) are reported separately.

Each result owns its numerical snapshot and remains valid after adding inputs
to, solving again, or destroying the problem. Measurements and solver factors
are not retained in the result. The fixed robot model is shared for FK, without
an additional cached joint list; its links, joints, IDs, names, and geometry
must not be changed while a problem or result uses it.

The original irregular knot configurations are held **exactly fixed**. The
remaining velocity problem is Gaussian: the implementation substitutes zero
configuration increments into the existing GP factors and solves once. Dense
joint-angle measurements connect the bordering joint-rate variables through
the GP conditional mean. Measurement covariance includes the conditional WNOA
position covariance, as in GTSAM's interpolation wrappers; cross-time bridge
correlations are approximated by separate measurement factors.

The base GP also uses the Kinematics result's COM world linear velocities as
observations. Dense recorded root/reference poses are not fit measurements.
The current recording uses 715 knots and the 50 Hz joint observations between
the first and last knots. The initial view is the 78–98 second window; the
entire trajectory is selectable without another download.

The class exposes configurations, base twists, joint rates, and native link FK.
It does not impose stationary feet or estimate accelerations. The app reports
stance-foot translation/angular velocities so residual contact motion remains
visible. Sparse fixed configurations can limit the fit even with dense encoder
observations. Process noise is configurable; the initial values are assumptions,
not a calibrated sensor/process model.

Base poses are `world_T_base_COM`; base twists are angular-first, expressed in
base COM axes. Sole twists are angular-first, expressed in world axes at the
sole origin. Joint measurements are mapped by name. Storage is radians/SI;
rotational configuration parameters and plots use degrees. Rendering uses
world body xyz/xyzw transforms from native FK and canonical MuJoCo geometry.

## Page structure

The page follows a compact technical-paper sequence: abstract, complete replay
and controls, motivation, approach and a synchronized reduced factor graph,
factor definitions, recorded results/noise settings/plots, discussion, and
attribution/downloads. Clicking a displayed graph slice moves playback. Factor
scopes distinguish exact fixed configurations from the soft velocity factors;
encoder factors sharing a scope are explicitly grouped in the drawing.
Cross-app links derive their destination from `apps/registry.json` and preserve
standalone URLs, embedded navigation, and deployment prefixes.

## Regenerate

Build the direct `gtdynamics.i` Python bindings, activate `py312`, and expose the
matching GTDynamics and GTSAM build packages:

```bash
cmake -S . -B build -DGTSAM_DIR="$HOME/git/gtsam/build"
cmake --build build --target gtdynamics_py -j6
PYTHONPATH=build/python:$HOME/git/gtsam/build/python python \
  apps/interpolation/scripts/export_recording.py \
  --dense-data apps/kinematics-inspector/.cache/openio-contact-source
```

The ignored dense source is prepared by the existing Kinematics importer. The
exporter checks its NPZ/metadata hashes against the winning recording, chooses
`comparison.json`'s `bestVariant` automatically, and records source fingerprints,
noise assumptions, fit errors, and solve time. It uses the pinned shared G1
factory and model geometry, not another app's recorded geometry.

`--joint-process-sigma-degrees` and
`--base-angular-process-sigma-degrees` are process scales in
degrees/s^(3/2), squared after conversion to radians to form acceleration PSDs.
The encoder measurement scale is `--joint-measurement-sigma-degrees`.
The base linear process scale is m/s^(3/2), and `--base-velocity-sigma` is m/s.

Only the final replay, compiled geometry, metadata, and attribution notices are
published. Compressed recording/model files use Git LFS. The shared delta-coded
display codec quantizes values to 1e-6; diagnostics retain native precision.
The browser loads no native optimizer or MuJoCo runtime.

## Validate

```bash
make -C build -j6 testFloatingBaseInterpolator.run
PYTHONPATH=build/python:$HOME/git/gtsam/build/python python -m unittest \
  discover -s python/tests -p test_floating_base_interpolator.py -v
cd apps
npm test --workspace interpolation
npm run build
```

`tests/app.browser.mjs` checks playback, joint switching, overlays, fitting,
and both deployment prefixes against a built preview. Native tests cover
irregular times, noncontiguous IDs, constant motion, exact knot configurations,
Pose3 mean derivatives and twist continuity, dense measurements, world/body
velocity conversion, invalid inputs, independent repeated solves, and result
lifetime after problem destruction.
