# Using contact initialization instead of inverse kinematics

`Initializer::InitializeSolutionContacts` is the measured-joint alternative to
`Initializer::InitializeSolutionInverseKinematics`. Both accept a robot, a root
link name, and an initial world pose, and return ordinary GTDynamics
`gtsam::Values` suitable for seeding a later optimization.

The inputs differ because this initializer keeps measured joints fixed and
estimates floating-base poses from repeated ground contacts. It needs measured
joint angles and an explicit stance schedule instead of desired pose goals.
Each uninterrupted stance gets a new ID; a later touchdown of the same foot
must use a different ID. Contact points use link **COM coordinates**, as do
`PointOnLink` and `Robot::forwardKinematics` elsewhere in GTDynamics.

```cpp
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/values.h>

gtdynamics::Initializer initializer;
std::vector<size_t> samples = /* original graph indices, increasing */;
gtsam::Values measuredJoints;
// Populate JointAngleKey(joint_id, sample) for every joint and sample.
// Optional JointVelKey measurements are preserved; absent velocities are zero.

std::vector<gtdynamics::ContactEpisode> contacts = {
    {"left-stance-0", 0, 14, leftSolePoints},
    {"right-stance-0", 10, 28, rightSolePoints},
    {"left-stance-1", 24, 40, leftSolePoints}};

gtdynamics::contact_init::Options options;
options.model = gtdynamics::contact_init::Model::Ground;
gtdynamics::contact_init::Result diagnostics;
gtsam::Values seeds = initializer.InitializeSolutionContacts(
    robot, rootLinkName, initialRootCOMPose, measuredJoints, samples,
    contacts, samplePeriodSeconds, options, &diagnostics);

double solveMilliseconds = diagnostics.timings.optimizationMs;
```

Episode intervals include their first and last samples. Their boundaries must
occur in `samples`; the initial and final requested samples need active
contacts. The adapter retains contact boundaries and selects additional requested
samples whenever the next boundary is more than one second away. It uses the
latest measured sample within one second of the previous frame; requested
samples too sparse to meet that limit are rejected. The adapter freezes FK
geometry at all selected samples and builds one GTSAM graph whose variables
are root poses and shared stance points.
The first root pose fixes the world gauge. By default the adapter rejects disconnected
contact graphs: independent components have unrelated world gauges and cannot
safely be interpolated into one dense trajectory. The lower-level core exposes
those components for diagnosis and visualization.

The returned trajectory uses the original `PoseKey(link_id, sample)` and
`JointAngleKey(joint_id, sample)` conventions. Between event samples, the
adapter interpolates root poses and recomputes FK with that sample's measured
joint angles. Those dense values are initialization seeds; the sparse solve
does not enforce contact constraints at every intermediate sample. Remaining
dynamics variables receive zero seeds. Root twists are zero, so an estimator
with IMU-derived velocities and biases should keep its existing velocity and
bias initialization.

An estimator that already accepts `root_pose_init` before per-slice FK can use
`Pose(seeds, root_link_id, k)` in that argument. Alternatively, it can update
only its existing `PoseKey` values from `seeds` after constructing all slices,
preserving its own twist, velocity, and bias keys. This makes the pose seeds
compatible with that workflow while leaving the estimator's sensor factors
and state layout intact. This API is not a literal replacement for an IK call:
the measured joint trajectory and contact episode schedule are additional
required information.

`diagnostics.timings` separates native graph construction, initial placement,
and GTSAM Levenberg–Marquardt optimization. Those times cover the sparse core;
the adapter's robot FK and dense trajectory expansion are outside the core
timer. `dt` supplies event timestamps and does not change the spatial residuals.

For callers that already have FK contact geometry, the lower-level
`gtdynamics::contact_init::ContactInitializer` accepts `EventFrame` objects
directly and has no dependency on `Robot`. Its optional pose-key callback also
supports downstream applications with their own key conventions.


Optional priors are explicit native API settings:

```cpp
options.addBetweenFactors = true;  // Identity motion, between adjacent event poses.
options.betweenTranslationSigma = 10.0;  // metres
options.betweenRotationSigma = 3.141592653589793;  // radians
options.addUprightFactors = true;  // Body Z aligned to world Z; yaw stays free.
options.uprightSigma = 0.2617993877991494;  // 15 degrees
options.addGroundFactors = true;  // Scalar landmark-height prior; XY stays free.
options.groundSigma = 0.02;  // metres
// Optional known world height; otherwise use the first anchored stance mean.
options.groundPriorHeight = 0.0;
```

With BetweenFactors enabled, the graph connects separate contact components
and fixes only its first pose. The adapter can then expand that regularized
solution; the cross-gap motion comes from the explicit weak prior. Upright
and landmark-height factors alone do not connect separate components. `Graph::contactComponents`
preserves the measured support structure, while `Graph::components` describes
the complete graph. `Result` reports separate contact, between, upright and
ground objectives; `residualRms` excludes all prior costs. Soft ground factors
keep Point3 landmarks and penalize only their Z coordinate. The hard-ground
Point2 model already enforces the floor exactly and omits redundant height
factors. A supplied height that conflicts with its fixed plane is rejected.
