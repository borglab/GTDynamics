# Using contact initialization instead of inverse kinematics

`Initializer::InitializeSolutionContacts` is the measured-joint alternative to
`Initializer::InitializeSolutionInverseKinematics`. Both accept a robot, a root
link name, and an initial world pose, and return ordinary GTDynamics
`gtsam::Values` suitable for seeding a later optimization.

The inputs differ because this initializer keeps measured joints fixed and
estimates floating-base poses from repeated contacts. It takes a
`Trajectory` of `Phase` objects whose `ConstraintSpec`s expose `contactPoints()`.
`FootContactConstraintSpec` implements this interface. Contact points
use link **COM coordinates**, as do `PointOnLink` and `Robot::forwardKinematics`.
The same point on the same link shares a landmark across consecutive phases,
regardless of point ordering. A point that disappears
and later returns gets a fresh landmark. Callers do not supply touchdown IDs or
a second contact schedule.

```cpp
#include <gtdynamics/utils/Initializer.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/utils/values.h>

using namespace gtdynamics;
auto left = std::make_shared<FootContactConstraintSpec>(leftSolePoints);
auto right = std::make_shared<FootContactConstraintSpec>(rightSolePoints);
PointOnLinks bothPoints = leftSolePoints;
bothPoints.insert(bothPoints.end(), rightSolePoints.begin(), rightSolePoints.end());
auto both = std::make_shared<FootContactConstraintSpec>(bothPoints);
Trajectory trajectory(WalkCycle(std::vector<Phase>{
    Phase(0, 9, left), Phase(10, 14, both), Phase(15, 23, right),
    Phase(24, 28, both), Phase(29, 40, left)}), 1);

gtsam::Values measuredJoints;
// Populate JointAngleKey(joint_id, sample) for every joint and slice 0..40.
// Optional JointVelKey measurements are preserved; absent velocities are zero.
contact_init::Options options;
options.model = contact_init::Model::Ground;
contact_init::Result diagnostics;
gtsam::Values seeds = Initializer().InitializeSolutionContacts(
    robot, rootLinkName, initialRootCOMPose, measuredJoints,
    trajectory, samplePeriodSeconds, options, &diagnostics);

double solveMilliseconds = diagnostics.timings.optimizationMs;
```

Phases use original inclusive sample indices. They must be ordered and
contiguous; adjacent phases can be disjoint or share one endpoint. A shared
endpoint uses the union of the two contact sets and produces one state.
A specification with no contact points represents flight. Missing specs,
duplicate points within a phase, foreign robot links, and invalid intervals
are rejected. Each joint must be measured at every trajectory slice.

Each `PointOnLink` has an `isGround` flag, defaulting to `true` for existing
walking examples. For a contact away from the ground plane, use
`PointOnLink(link, localPoint, false)` (the same constructor is wrapped in Python).
`ConstraintSpec::groundContactPoints()` selects the flagged contacts for
kinematic height objectives. The initializer uses **all** `contactPoints()`;
changing `isGround` does not change which contact points share a landmark.
For mixed ground/wall contacts, select `Model::Patch` or `Model::Point` and leave
`addGroundFactors` disabled. `Model::Ground` still imposes its global level-ground
model; the flag does not change that solver option.

The initializer keeps supported phase boundaries and supplements long supported
phases with frames at most one second apart (every sample when `dt > 1`). It
freezes FK at those samples and solves root poses and shared contact landmarks.
The initial root pose anchors the first supported frame. Unsupported prefix
samples hold that pose; unsupported suffix samples hold the final solved pose.
Internal flight samples interpolate between surrounding solved frames. These
are explicit initialization assumptions, not observed motion.

By default, disconnected contact graphs are rejected: independent contact
components have unrelated world gauges. The optional motion prior can connect
them. The lower-level core retains component diagnostics. At least one supported
frame is required.

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
the measured joints and phase trajectory are additional required information.

`diagnostics.timings` separates native graph construction, initial placement,
and GTSAM Levenberg–Marquardt optimization. Those times cover the sparse core;
the adapter's robot FK and dense trajectory expansion are outside the core
timer. `dt` supplies event timestamps and does not change the spatial residuals.

For callers that already have FK contact geometry, the lower-level
`gtdynamics::contact_init::ContactInitializer` accepts `ContactMeasurements`
objects directly and has no dependency on `Robot`. Each object contains the
existing `gtdynamics::Slice` context, an actual timestamp in seconds, and frozen
contact observations. `measurements.slice.k` is the original source sample
index; its position in `Graph::measurements` is only a sparse vector ordinal.
For example, selected slices 42 and 103 retain those indices, rather than being
renumbered 0 and 1. The adapter uses `PoseKey(rootLinkId, slice.k)` for the sparse
root values. The native app JSON keeps its `frames` and `sampleIndex` field names.
The optional pose-key callback also supports applications with other key
conventions.


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
