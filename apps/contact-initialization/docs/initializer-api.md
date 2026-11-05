# Pose-contact initialization

The G1 app and Kinematics use the same wrapped native API:

```python
options = gtd.pose_contact_init.Options()
options.addGroundFactors = True
options.addBetweenFactors = False
result = gtd.pose_contact_init.Result()
values = gtd.PoseContactInitializer().run(
    robot, "pelvis", initial_root_com_pose, measured_joints,
    trajectory, dt, options, result, contactBuffer=0,
)
```

The trajectory is a sequence of phases with `PoseContactConstraintSpec`, each
containing one `PoseOnLink` per active foot. `PoseOnLink.comTcontact` maps sole
coordinates into link COM coordinates. The canonical G1 body-to-COM conversion
lives in `examples/g1_mjlab/mujoco_io.py::contact_frames`. The ground role controls
height/normal priors; non-ground contacts still provide pose observations.

Only measured joint angles, the explicit phase contact schedule, and the initial
root gauge enter initialization. Frozen observations come from root-relative FK.
The initializer excludes the first selected root variable from optimization and
holds that gauge exactly. Disconnected contact graphs require explicitly enabled
weak motion priors. No reference trajectory supplies intermediate anchors.

`result.slices()` and `result.times` preserve the actual selected source indices
and timestamps. Optional buffering trims stance endpoints. Dense returned Values
interpolate sparse root poses and expand measured-joint FK at every source slice;
these dense samples are not additional optimization observations.

Native diagnostics expose:

- `poses` and `contactPoses`: world root-COM and world sole poses, respectively.
- `contactFrames`: the corresponding fixed `PoseOnLink` definitions, in stance
  order. A returning foot gets a new entry after lift-off.
- `observations`: `poseIndex`, `contactIndex`, and frozen `rootTcontact`. These
  indices address `poses` and `contactPoses`, not original source slice numbers.
- `graph` and `values`: the actual sparse native graph and final Values. The
  Values additionally include the fixed first root; it has no free graph key.
- `history`: native sparse iterations, full pose landmarks, objective and
  translational RMS. The objective is twice the GTSAM graph error.
- `rotationRms`, `iterations`, `converged`, and native phase timings.

The exporter uses this data directly for its bipartite graph and error budget.
It does not recreate stance identities in Python. Its frame visualization converts
world root-COM poses back to world pelvis-body poses before placing MuJoCo meshes.
Reference alignment and error evaluation occur only after the native solve.

The existing `PointOnLink`, `FootContactConstraintSpec`, `contact_init` measurement
and result types, and `InitializeSolutionContacts` are preserved for other clients.
This G1 migration does not change dynamics contact points or wrench models.
