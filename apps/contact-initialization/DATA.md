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
the four material corners, rather than the speed of the foot center. Candidate
support turns on below 4 cm and 0.5 m/s, and remains on up to 6 cm and 1.0 m/s.
Bounded gaps of at most two samples are bridged; episodes shorter than three
samples are removed. All settings and raw height/speed signals are exported.

The result has **19 candidate stance episodes, 35 event frames, and 3 support
components**. There are 1,495 double-support samples, 294 single-support samples,
and 11 samples without candidate support. Some contact-chain breaks reflect
rapid sliding or uncertain retargeted support: neither foot is ever more than
4.48 cm above the inferred floor, so these gaps should not be presented as
verified flight. The conservative speed threshold rejects those uncertain
states. No artificial motion factors reconnect the components.

Each component has its own gauge and cannot be positioned relative to the others
from these factors alone. Evaluation must respect the separate gauges; a single
first-pose alignment over the entire sequence would report a misleading global
trajectory error. A kinematic contact label also does not establish no-slip or
rigid-flat-foot contact: the inferred stances may still violate those assumptions.

Samples store `contactSignals.left/right.soleHeightAboveFloorMeters` and
`minimumSoleSpeedMetersPerSecond`. They have no `normalLoads` field. The dataset
has `metadata.contactEvidence = "kinematic"` and a documented detector type.
The shared frame/stance convention is `[start,endExclusive)`, with each graph
endpoint `end = endExclusive - 1` at the last supported sample.

## Reproduce

Use NumPy and MuJoCo 3.16.0 with the original G1 `model.mjb` described in
[README.md](README.md). From the repository root:

```bash
python apps/contact-initialization/scripts/export_g1_moves.py \
  --model /path/to/original/g1-recording/model.mjb
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
