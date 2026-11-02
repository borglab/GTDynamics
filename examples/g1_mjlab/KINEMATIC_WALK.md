# G1 kinematic estimation with CM-Opt

`kinematic_walk.py` is the driver. Read `load_walk`, phase inference, slice
selection, `build_problem`, initialization, `solve_cm`, and `evaluate` in order.
The driver imports focused modules:

- `contact_phases.py`: max-product inference, sampling, and stance identities.
- `kinematic_initialization.py`: native initialization and retained-slice mapping.
- `kinematic_smoothing.py`: graph construction, CM-Opt, and residual evaluation.
- `mujoco_io.py`: recordings, canonical model conversion, and FK validation.
- `imu_aiding.py`: Gal3 pims, world COM velocities, and one shared sensor bias.

## Default sampling

Use the original contact phases to retain only double-stance slices. A phase of
at most 200 ms contributes one central slice. A longer phase contributes its
one-sample-buffered endpoints plus evenly distributed interior slices, with gaps
no larger than 200 ms. `--max-slice-interval-ms` changes the limit. The source
sample period sets the attainable resolution; adjacent intervals within a phase
differ by at most one source sample. Skipped single-stance phases can leave
larger gaps between phases.

Contact initialization and full-state estimation use exactly these source slices
and actual timestamps. No interpolation introduces solve states. Original phases
preserve stance identity through omitted slices: a continuing sole retains its
world landmark, and lift-off/recontact starts another. The native initializer
returns the retained slices; Python renumbers solver keys while retaining
`source_indices`, `timestamps`, and hidden contact breaks.

The full SSIO recording has 10,998 source slices at 50 Hz and 787 inferred phases.
The default retains 715 slices, 395 stance landmarks, 1,430 contact-pose
observations, and 714 IMU intervals of 0.06–0.88 s. Contact Labels remains at the
source rate because it derives the labels before estimation-slice selection.

## Model and graph

The pinned `mujoco-menagerie`, MuJoCo, and MJLab pipeline selects `g1_mjlab`.
Each foot has one material sole frame, with +X forward, +Y left, and +Z up.
`PoseKey` stores world-to-link-COM poses; fixed COM-to-sole transforms account for
MuJoCo body/COM offsets. Geometry and joints come from the same compiled model.

Each uninterrupted stance has a free world `Pose3` landmark. Native
`ContactPoseFactor` measures the local-coordinate difference between that
landmark and the predicted sole pose. Initialization freezes encoder FK as a
root-relative observation; smoothing optimizes link poses coupled by hard
kinematic equalities. Contact observations use 10 mm translation and 90 degree
per-axis rotation standard deviations. Configuration specifies degrees and
converts to radians at native APIs.

The apps' contact initializer also uses soft upright (15 degrees), landmark
height (10 cm), and weak zero-motion (10 m / 180 degrees) priors. Sole-normal
alignment is off. Smoothing exposes upright and height priors separately and
omits weak motion regularization; optional IMU aiding connects adjacent retained
slices. Its pims integrate over the actual irregular intervals. One shared bias
and world-frame COM velocities accompany the Galilean IMU factors.

The initial root gauge and measured FK determine a common vertical shift placing
the initial mean sole height at zero. Later reference poses enter only evaluation;
SSIO contact labels themselves are privileged because their displacement cues
use SuperOdom LiDAR-inertial poses.

## Run

Use a Python environment containing the matching GTDynamics/GTSAM bindings,
NumPy, and the pinned model packages. From the repository root:

```bash
export PYTHONPATH="$PWD/build/python:/path/to/gtsam/build/python:$PYTHONPATH"
python examples/g1_mjlab/kinematic_walk.py --initializer contact
python examples/g1_mjlab/kinematic_walk.py --initializer ik
python examples/g1_mjlab/compare_kinematic_walk.py --repeats 3
python apps/kinematics-inspector/scripts/export_apps.py --only trajectory
python apps/kinematics-inspector/scripts/export_apps.py --only snippets
python apps/contact-initialization/scripts/export_native_recordings.py
python apps/contact-initialization/scripts/export_ssio_recordings.py
```

The example's default input is the local MJLab recording under `results`; see
[recording instructions](README.md#reproduce). Pass `--data` and `--contacts` for
another recording. SSIO uses max-product phase inference on its 50 Hz source;
other recordings retain their supplied schedules. The timing driver calls the
same example functions, alternates initializer order, and reports graph setup,
initialization, and CM-Opt timing separately.

The Kinematics app retains initial/final states and scalar error histories, with
50 CM-Opt iterations. Contact vs IK retains all native initializer and smoothing
iterations on three ten-second sources, with ten CM-Opt iterations. Contact
Initialization compares all combinations of its three soft priors on five clips.
All browser assets are offline recordings; large assets use Git LFS.

The Kinematics comparison table ranks all 16 configurations by SE(3)-aligned
pelvis RMS. The best saved result uses height, IMU, and loop closure, with upright
off (8.244 m aligned RMS). Substantial drift remains versus the SuperOdom
reference, and saved results reach the iteration cap. Alignment changes display
and reference error only; it adds no factors or scale changes.
