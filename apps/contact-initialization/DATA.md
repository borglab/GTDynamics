# Sequence data

[public/sequences.json](public/sequences.json) lists the available datasets and
their native-solver recording paths. The original 8-second policy walk remains
in [g1-walk.json](public/g1-walk.json), with its original force-based schedule.

## Original 8-second policy walk

The original 800 samples are the 100 Hz G1 rollout from [PR #446](https://github.com/borglab/GTDynamics/pull/446),
source revision `dba5c5eaea5d9bd19f0fa91191bbbbc9a4291941`. The recorder pins its
policy checkpoint and disables startup randomization, observation noise, and
pushes, using seed 42 and a 0.5 m/s command. Source hashes and policy/model
provenance are retained in the JSON metadata.

Native MuJoCo replay sums foot/terrain normal forces. Support turns on at 20 N
and off below 10 N; bridging bounded one-sample gaps joins 15 isolated 10 ms
dropouts. This yields 38 stance episodes and 75 event frames. The initial saved
sample has no support and is excluded, so the graph spans source samples 1–799.
It has one support component, 328 double-support samples and 471 single-support
samples. Local FK replaces the recorded root with the identity before computing
body and foot measurements. Recorded root poses are stored separately.

Timestamps are rebased to the first saved post-policy-step sample, so the graph
spans 0.01–7.99 s. Contact endpoints are first and last supported samples, with
10 ms resolution; the final stance is clipped by the recording boundary. The
derived FK was audited over all 24,000 body samples within 0.88 micrometers.
Regeneration uses `scripts/export_g1_contacts.py` with the original `model.mjb`
and `walk.npz` from the G1 PR; the raw input hashes are recorded in the JSON.

## Downloaded 30-second step-touch capture

The longer sequence is a continuous window of the public
[G1 Moves dataset](https://huggingface.co/datasets/exptech/g1-moves/blob/895d064b2385725e6aabf540461c219afc3e0916/README.md),
created by Mitch Chaiet / Experiential Technologies. Jasmine Coro performed the
`J_Dance0_StepTouch` dance. It is human motion retargeted to the Unitree G1's 29
joints, not recorded encoder data or a simulated policy rollout. The source is
licensed **CC BY 4.0**, with attribution and changes documented in
[G1_MOVES_ATTRIBUTION.txt](public/G1_MOVES_ATTRIBUTION.txt).

The downloaded CSV has 1,949 rows at 60 Hz, approximately 32.48 seconds. The app
uses its first 1,800 rows, a continuous 30-second window at the native sample
rate. The first-to-last timestamp span is 29.9833 seconds. No rows are repeated,
tiled, interpolated, or synthesized.

The pinned source revision is
`895d064b2385725e6aabf540461c219afc3e0916`. The
[original CSV](https://huggingface.co/datasets/exptech/g1-moves/resolve/895d064b2385725e6aabf540461c219afc3e0916/dance/J_Dance0_StepTouch/retarget/J_Dance0_StepTouch.csv)
has SHA-256
`d821d4ce0623242965c1f153fc8f8e0fb306294d1c084535ef27d9a3d36322a0`.
Its root quaternion is XYZW, followed by the standard G1 29-joint order. Export
normalizes the quaternions and converts them to WXYZ. The robot geometry matches
the G1 model used for the original sequence; comparison with the upstream
training NPZ's body FK agreed within a few micrometers in inspected states.

## Contact evidence and gaps

This capture contains no contact-force measurements. Its contact schedule is an
explicit **kinematic candidate-support heuristic**. It uses recorded-root FK
only to annotate contacts, then discards the root from the solver measurements.
The floating-base poses are stored separately for evaluation and display.

The detector uses the minimum height of the four fixed sole rectangle corners.
A robust floor estimate is the fifth percentile of the lower of both feet over
the window. To accommodate rocking, speed is the minimum horizontal speed among
the four sole corners, rather than the speed of the foot center. Candidate
support turns on below 4 cm and 0.5 m/s, and remains on up to 6 cm and 1.0 m/s.
Bounded gaps of at most two samples are bridged; episodes shorter than three
samples are removed. All settings and raw height/speed signals are exported.

A one-sample lookback now recovers direct handovers that acquisition hysteresis
otherwise separates. The incoming foot must already meet the height-on and
speed-hold thresholds at the outgoing foot's final supported sample, and its
support must be confirmed at the next sample. This refinement never crosses
an interval without support. It recovers the right-foot touchdown at source
sample 563 (9.383 s), creating the shared event frame missing from the earlier
schedule. The correction is recorded in `detector.handoverCorrections`.

The result has **19 candidate stance episodes, 55 selected frames, and 2 contact
components**. The frames comprise 34 contact boundaries and 21 supplemental
samples, so adjacent frames are at most one second apart. There are 1,496 double-support samples, 293 single-support samples,
and 11 unsupported samples (1357–1367, 22.617–22.783 s; next support at 22.800 s).
Both soles are above the inferred floor throughout that gap: the lower sole is
3.58–4.48 cm high. A quadratic fit to the model's mass-weighted centre of mass
in those samples gives vertical acceleration about -9.34 m/s², with 1.81 mm
fit RMS. This is consistent with a short hop, rather than a missed grounded
handover. It remains kinematic evidence from retargeted motion, without force
measurements or a claim of verified physical contact.

Contacts alone leave two independent world gauges. The optional weak
BetweenFactors connect them as an explicit motion assumption; the UI keeps the
measured contact-component count separate from the regularized graph count.
Contacts-only errors align each component separately, while connected variants
use one first-pose alignment. Candidate support still does not establish no-slip
or rigid-flat-foot contact.

Samples store `contactSignals.left/right.soleHeightAboveFloorMeters` and
`minimumSoleSpeedMetersPerSecond`. They have no `normalLoads` field. The dataset
has `metadata.contactEvidence = "kinematic"` and a documented detector type.
The shared frame/stance convention is `[start,endExclusive)`, with each graph
endpoint `end = endExclusive - 1` at the last supported sample.

## Frame sampling

Both exporters retain every contact start and final supported sample. Whenever
the next boundary is more than one second away, they select the latest original
sample within one second of the preceding frame and repeat until the boundary
is reached. Each supplemental frame uses that sample's measured FK and the
contacts active at that timestamp. Contact intervals and flight gaps are
unchanged. The native GTDynamics adapter applies the same rule to its requested
joint samples before solving.

The policy walk already has a maximum frame gap of 0.71 s and needs no additional
frames. The capture now has 55 frames instead of 34, with a maximum gap of 1 s.
`metadata.frameSampling` records the interval and counts; each frame's `selection`
is `contact` or `periodic`. The UI marks periodic ticks with dashed lines.

To resample an existing FK export without regenerating measurements, run in
`py312` from the repository root, then regenerate the native recordings:

```bash
python apps/contact-initialization/scripts/contact_discovery.py \
  apps/contact-initialization/public/g1-walk.json \
  apps/contact-initialization/public/g1-steptouch-30s.json
python apps/contact-initialization/scripts/export_native_recordings.py \
  --optimizer build/contact-initialization/contact_initializer
```

## Reproduce

Use the `py312` environment with NumPy and MuJoCo. Bootstrap the pinned G1
Menagerie model as described in [README.md](README.md), then run from the
repository root:

```bash
python apps/contact-initialization/scripts/export_g1_moves.py
```

The script downloads the pinned CSV into the ignored app `.cache/g1-moves/`
directory and checks its SHA-256 before parsing. It writes
`apps/contact-initialization/public/g1-steptouch-30s.json`. Optional window,
height/speed, gap, and minimum-duration arguments are explicit; a requested
window beyond the source recording raises an error instead of looping.

The original dynamic sequence uses
`scripts/export_g1_contacts.py` and its simulator-force detector. Its normal loads
must not be compared as if they were the same evidence as the longer capture's
height/speed labels.

## Visual model

The browser renders Unitree's 29-joint G1 meshes from MuJoCo Menagerie revision
`a03e87bf13502b0b48ebbf2808928fd96ebf9cf3`, shared with Motion Snippets.
`scripts/bootstrap_mujoco_models.py` downloads the pinned assets and generates
the contact app's manifest. The static build includes the model's BSD-3-Clause
license at `assets/unitree_g1/LICENSE`. Node tests compare its pelvis-relative
FK with the recorded body positions in both sequences. Rendering uses measured
joint angles and the recorded event root poses; it does not change the solver's
measurements or benchmark timings.
