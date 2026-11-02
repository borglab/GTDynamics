# Contact Labels

Inspect privileged stationary-contact training targets on three 20-second
windows of SSIO / Open-IO G1 clip `10871_humanoid_20260520_223451`:

- 24–44 s: first turn and walking onset.
- 78–98 s: mid-clip turns.
- 198–218 s: return turn.

Each recording contains 1,000 samples at 50 Hz, with additional source context
used during labeling. This app covers target construction. It does not train
or evaluate a proprioceptive classifier.

## Target definition

The additive native utility `contactPoseMotionLabels` accepts actual timestamps,
world poses of one material contact frame per foot, and `ContactPoseMotionParams`.
The frames come from the canonical Menagerie `g1_mjlab` model, assembled through
MJLab's public factory. Frame +X is forward, +Y left and +Z up; the origin lies on
the foot capsules' lower envelope. The existing point-based utility is retained.

In each 200 ms centered window, it measures:

- Contact-origin endpoint speed (m/s) and maximum excursion from the center (m).
- Endpoint SO(3) angular speed (rad/s) and maximum angular excursion from the
  center (rad). The excursion detects rotations that return to their start.

Default stationary bounds are 0.10 m/s, 0.010 m, 0.30 rad/s and 0.030 rad.
Moving bounds are 0.25 m/s, 0.025 m, 0.80 rad/s and 0.080 rad. All four metrics
must be below their stationary bounds for label 1. Any reaching its moving bound
gives label 0; intermediate or incomplete windows give -1 (uncertain). Scores
interpolate threshold margins; they are not calibrated probabilities. Strict and
tolerant presets expose sensitivity. Context outside each display interval keeps
its edge windows complete. Pure pivots and rocking can therefore be distinguished
from a stationary sole even when the contact origin barely translates.

The inputs are privileged SuperOdom LiDAR-inertial pelvis poses, measured joint
angles, and robot geometry. These are reference-derived **stationarity targets**,
not force measurements or proof of load-bearing contact. A stationary foot held
in the air passes a pure motion test. Reference error, encoder noise, and timing
also limit target quality. Constant world translations or rotations do not change
the labels. No gyro, torque, old contact labels, or initializer output enter them.

The additive `walkingContactLabels` utility couples the two feet under a walking
support assumption. When the other origin reaches either moving translation
bound and this origin stays below both, it promotes this foot to **2: likely
contact**, preserving any raw stationary label. Both windows must be complete.
Rocking of the inferred supporting foot is allowed; rotation alone in the other
foot does not trigger the rule. If both origins clearly translate, neither gets
promoted. The inputs are the original, aligned pose-motion samples, with the
same thresholds; call with the arrays swapped for the other foot.

This heuristic infers possible support, not sole stationarity or a measured
contact probability. It assumes walking without flight and can be wrong when
the robot is airborne or slides. Raw labels, angular diagnostics and scores are
retained, so a rocking support remains distinguishable from a stationary sole.

## Inspection

The viewer defaults to **Discrete FG · max-product**; select **Window model**
to replay the existing windowed labels with or without the walking support
prior. Three aligned timelines compare the aided graph, displacement-only graph,
and window model. Window
labels map to L/R only when one foot has support evidence (stationary or likely
contact) and the other is moving; both supported maps to double stance. Other
combinations remain unresolved. Agreement excludes those unresolved samples
and is not accuracy against measured contact.

## Simple discrete alternative

`contactSupportGraph` builds a native GTSAM `DiscreteFactorGraph`, exposed
directly in Python. Each 50 Hz **slice** has two binary variables: left key
`2*k`, right key `2*k+1`, with 0=swing and 1=stance. Four joint configurations
are possible: WW, WS, SW, SS (left foot first). Both-swing is allowed with a
within-slice penalty of 0.01 versus 1 for the other three configurations.

For each adjacent pair of slices, the graph contains:

1. One four-variable transition factor. The default 4 by 4 table is the product
   of independent per-foot transitions with persistence 0.97 and switching 0.03.
   Entries are 0.9409 for no switches, 0.0291 for one, and 0.0009 for two.
   A general positive row-stochastic table can be supplied through the API.
2. Two binary displacement factors, one per foot. The factor on slices `k,k+1`
   observes **both** incoming displacements `d[k],d[k+1]`, where
   `d[k] = ||world_p[k] - world_p[k-1]||` in metres. One preceding observed pose
   supplies the first displacement. Each observation gets a half-normal density
   with sigma=3 mm for stance and sigma=20 mm for swing.

The per-foot 2 by 2 table in W/S order is
`[[W[k]^w[k] W[k+1]^w[k+1], W[k]^w[k] S[k+1]^w[k+1]],
  [S[k]^w[k] W[k+1]^w[k+1], S[k]^w[k] S[k+1]^w[k+1]]]`.
Here S and W are narrow/broad **density values**, not state probabilities.
Weights are 1 at sequence endpoints and 0.5 elsewhere: overlapping factors
count each measurement once. This initial table is separable; it adds no assumed
correlation between displacements. Large/large now favors swing/swing, rather
than giving a support exchange the broad likelihood automatically.

The first left/right pair has a uniform prior. Factor tables are max-rescaled
with a numerical floor. The model uses uncalibrated assumptions at 20 ms spacing;
pure pivots remain unobservable. Incoming displacement is associated with its
ending slice, so within-interval transitions introduce timing ambiguity.

The exporter calls `graph.optimize(gtsam.Ordering.OrderingType.NATURAL)` for
GTSAM max-product elimination and traceback of the **joint MAP sequence**.
Ten context slices per end and one preceding pose yield 1,020 slices, 2,040
binary variables, and 6,114 factors without velocity. Solve timings exclude
construction, loading and export; all inference is offline. Plotted displacement
is aligned with its ending slice. Window thresholds do not change graph results.

### Three-slice duration prior

Each foot has a factor on `(c[k-1], c[k], c[k+1])` at every interior slice.
It assigns weight `singleSliceWeight=0.1` to WSW or SWS and 1 to all other
triples. Thus an isolated stance or swing slice receives an additional
negative-log cost of `-log(0.1) = 2.302585`. The prior is soft: evidence can
outweigh it. At 50 Hz it discourages one-slice (20 ms) events, without imposing
a hard minimum duration or penalizing ordinary sustained transitions.

There are `2*(N-2)` such factors; no padding or artificial boundary states are
introduced. Both graph variants include them. The native/Python parameter
`ContactSupportParams.singleSliceWeight` accepts values in `(0,1]`; 1 makes
the factors neutral. Inference remains exact GTSAM max-product, now with
three-slice factors alongside the existing adjacent-slice factors.

### Optional hard walking constraints (default on)

The app checkbox **Require support and double-stance transitions** selects
precomputed results with `graph.push_back(gtd.contactWalkingConstraints(N))`.
This adds N binary factors forbidding simultaneous swing and N-1 four-variable
factors forbidding left-only/right-only exchanges. The new factors contain
exact zeros and ones. Existing measurements, transitions, and duration priors
remain unchanged, and no numerical floor is applied to the constraint masks.

Every support exchange must pass through a double-stance slice. This guarantees
an intervening slice label, not a physical minimum contact duration. Turning
the option off selects the unconstrained solution. All four combinations with
velocity aiding on/off are computed offline. No new downloads are needed when
toggling. For N=1,020 the constraints add 2,039 factors: 10,193 total with
velocity aiding, or 8,153 without. The counts below exclude these masks.

### Joint-velocity unary factors

The **Joint-velocity factors** checkbox defaults on and adds two unary factors
per sample with `contactVelocityFactors`, directly wrapped in Python. The input
is the measured joint `dq` channel (rad/s), interpolated to the same timestamps
as `q` and mapped by joint name to MuJoCo DOF addresses. For each sole origin,
`point_velocity_in_frame` uses MuJoCo's point and pelvis Jacobians to compute

```
v_body = R_world,pelvis.T * ((J_point - J_pelvis) * qdot
         - cross(J_angular,pelvis * qdot, p_point - p_pelvis))
```

This is the derivative of the sole origin's coordinates in the pelvis frame,
in m/s, with +X forward. Body-to-sole offsets are included. Subtracting rotational
transport matters: rotating a world linear velocity into pelvis axes alone is
not the same quantity. Free-base velocity is zeroed; common base motion cancels
in this relative expression. No LiDAR velocity or finite differencing enters it.

With `v = v_body.x`, the stance weight is
`c(v) = clamp(1/(1+exp(v/0.1)), 0.02, 0.98)`. Each foot has unary weights `[1-c, c]` in swing/stance order, independent of
the other foot's contact. At -0.3/0/+0.3 m/s the stance weights are approximately
0.9526/0.5/0.0474. Zero is neutral; the floor bounds the maximum odds at 49:1.
The sigmoid is a forward-walking heuristic, not learned calibration; backward
walking and moving supports can contradict it. The two cues are correlated.

Each complete graph has 1 initial pair prior, 1,020 support couplings, 1,019
four-variable transitions, 2,038 per-foot displacement factors, and 2,040 unary
velocity factors, plus 2,036 duration factors: **8,154 total**. Unchecking velocity replays the 6,114-factor
graph. The body-forward-velocity plot remains visible for comparison.

The app contains the mathematical model beside its factor graph: state definitions,
factorization, transition table, displacement densities and overlap weights,
relative-velocity Jacobian, duration and hard walking constraints. Numerical
examples and parameter tables use the selected recording. Pose-landmark equations
and initialization priors live in Contact Initialization; the shared smoothing
objective, IMU assumptions and reference alignment live in Kinematics and Contact
vs IK. There is no separate report to keep synchronized.

MathJax 3.2.2 typesets checked-in equation fragments to inline SVG during the Vite
build. The app needs no runtime math scripts or fonts and works offline. Shared
components and source equations live in `apps/shared/paper/`.

Joint-state colors follow navigation lights: L red, R green, double stance
yellow; neither is blue (unresolved window labels gray). When hard constraints are off, both-swing is an
inferred configuration, not proof of flight. Robot markers and trails use black for
stance, white for swing, gray for unresolved; white markers have a black outline.

## Playback

The app starts with a close robot-following view. Fit snippet frames the full
reference path. Colored origin markers and trails show the targets; RGB axes show contact-frame
orientation. Each foot has a label strip, class durations and motion metrics.
The **Walking support prior** checkbox defaults on for the window model.
Turning it off restores raw stationarity labels in markers, trails,
timelines and cards. Plots always show the unchanged raw motion evidence.

Seven full-width plots show scores, origin speed, origin excursion, angular speed,
angular excursion, consecutive displacement, and body-forward velocity. Time zoom is linked; vertical zoom remains independent. Drag/scroll to zoom,
use Plotly's pan toolbar, reset the view, or click a curve/label interval to scrub.
Threshold presets switch precomputed native labels without running a labeler or
solver in JavaScript. Only the selected snippet is downloaded. Real MuJoCo model
geometry is rendered through the shared RobotViewport; FK runs only offline.

## Reproduce

With matching GTSAM/GTDynamics bindings and the pinned
`examples/g1_mjlab/requirements-model.txt` packages in Python 3.12:

```bash
cmake --build build --target testContactMotion.run python-stubs -j6
PYTHONPATH=../gtsam/build/python:build/python \
  python apps/contact-labels/scripts/export_recordings.py
cd apps
npm install
npm test --workspace gtdynamics-contact-labels
npm run build
npm run preview
```

Open the hub at `/#contact-labels`, or the standalone app at `/contact-labels/`.
For development, `npm run dev --workspace gtdynamics-contact-labels` uses port 3006.
The exporter reuses the existing pinned SSIO download, pose resampling, joint-order
and IMU-origin validation, sole geometry, and compiled-model export utilities.
The dataset is pinned to revision `db621c57833e08a812b2ec8491aa0ca81f0ecb33`.

The website includes SSIO/Open-IO CC BY 4.0 attribution and the robot model license.
Raw sensor arrays remain in ignored caches; only consumed compressed recordings,
compiled geometry, a snippet manifest, and licenses are published. Recordings and
geometry follow the app Git LFS rules. Native source/module fingerprints and raw
input checksums accompany every snippet. The report-removal update changes only
provenance metadata: previous exporter/report hashes and a hash of the unchanged
numerical content are retained in `provenance.maintenance`.

C++ tests cover stationary poses, pivots, nonuniform timestamps, world-frame
invariance, returning motion, uncertainty, and invalid inputs. Python tests exercise
the actual wrapper and default/custom options. App data tests independently check
saved contact-frame transforms and target consistency. Browser tests cover both deployment
paths, playback, all snippets/presets, plot zoom, mobile layout, and hub unload/reopen.

Factor graph diagrams remain interactive in each app: selecting a displayed slice
updates the demo. Mathematical definitions appear in the formulation section;
recorded numerical results follow in the results section.
