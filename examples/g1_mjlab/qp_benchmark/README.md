# G1 sparse-support QP datasets

These local benchmarks freeze the **second global QP**, immediately before
GTSAM initializes its active set, after one accepted nonlinear LM step. Both
use `contact-gp-dynamics` initialization and the app's sparse selection of
single- and double-support slices, with original irregular timestamps.

| Dataset | Recording window | Selected slices | Single / double support | Reduced variables | Inequality rows |
| --- | --- | --- | --- | --- | --- |
| G1SP20 | 78–98 s | 171 of 1,000 | 114 / 57 | 18,549 | 230 |
| G1SPFULL | 0–219.94 s | 1,780 of 10,998 | 1,065 / 715 | 193,566 | 2,437 |

Each is the actual global problem passed to the QP solver, after the component
equality reduction through B, including shared contact/bias variables and
coupling between slices. The second QP was selected because the first has no
inequality rows. No constraints are added or removed during export. Inactive
nonlinear inequalities are absent from the captured tangent-cone model.
Twenty seconds describes the motion window, not a solver runtime limit.
The full schedule was inferred using the existing contact-phase algorithm;
its labels and provenance are included in `G1SPFULL/support-labels.json.gz`.
The 20-second clip uses the app's existing saved support labels.

The previous dense-sampling **G1CMCTWO** artifact remains separate. Its 106,836
variables describe a different problem; use these new datasets for the current
sparse-support application.

## Files and mathematical meaning

Original coefficients and noise scales are retained as float64. Sparse storage
drops only exact zeros. JSON key strings preserve uint64 precision.

| File | Contents |
| --- | --- |
| `J.npz` | Original **unwhitened** Jacobian, SciPy CSR |
| `G.npz` | Scalar inequality rows normalized to `G*x <= h`, SciPy CSR |
| `vectors.npz` | `r`, `sigmas`, `inverse_sigmas`, `h`, and zero `x0` |
| `metadata.json` | Ordered keys, offsets/dimensions, factor row ranges and ordered key lists, inequality row origins, QP parameters, native checks |
| `native_gradients.npz` | Native gradients at zero and the deterministic probe |
| `provenance.json` | Source/binary revisions and hashes, input/model provenance, sampling, actual solver options, TBB configuration |
| `source_changes.patch` | Uncommitted tracked example changes at capture time |
| `validation.json` | Lossless round-trip, native objective/gradient/constraint checks, and sparse MAT loader checks |
| `capture.log` | First accepted nonlinear step and both global QP calls |
| `problem.py` | Sparse factor loading, `qpbenchmark.Problem` construction, and MAT export |
| `IMPORT_GTSAM.md` | Prompt with absolute dataset locations and import instructions |
| `SHA256SUMS` | Checksums of every other packaged file |

With `W = diag(inverse_sigmas)`, the problem is

```
minimize  0.5 * ||W * (J*x - r)||^2
subject to G*x <= h
```

All h entries are zero. There are no explicit equality constraints or box
bounds; x0=0 is feasible. Factors starting at `damping_factor_start` already
contain the original diagonal LM damping at lambda=0.001. Keep them without
adding damping again. Additional QP regularization was zero.

For conventional solvers, `P=(W*J).T*(W*J)`, `q=-(W*J).T*(W*r)`.
The omitted objective constant is `0.5*||W*r||^2`, recorded in validation.json.
Forming P introduces normal-equation rounding. GTSAM can reconstruct the
original factors directly from the canonical package.

## Benchmark interoperability

The [free-for-all submission form](https://github.com/qpsolvers/free_for_all_qpbenchmark/blob/main/.github/ISSUE_TEMPLATE/new_problem.md)
allows generation code. A test-set iterator can yield
`problem.make_problem(dataset_directory)` with NumPy, SciPy, and qpbenchmark
installed. The result has sparse CSC P and G. No dense global matrix is formed.

The canonical package uses the documented `gtdynamics-sparse-qp-v1` schema;
it is not accepted by the existing `Problem.load()` NPZ path. Avoid the
inspected `Problem.save()` implementation, which densifies P and loses sparse G.
A standard sparse MAT export is also provided beside each ZIP. Regenerate it:

```bash
python problem.py /path/to/G1SP20 --mat /path/to/G1SP20.mat
python problem.py /path/to/G1SPFULL --mat /path/to/G1SPFULL.mat
```

The MAT files use the [existing free-for-all loader's convention](https://github.com/qpsolvers/free_for_all_qpbenchmark/blob/main/free_for_all.py):
`A=vstack([G,I])`, `l=-1e20`, `u=concatenate([h,+1e20])`, plus P, q, n and m.
The final n identity rows encode unrestricted bounds using its infinity sentinel.
Both exports were round-tripped through this unmodified loader, comparing P,
q, G and h exactly and checking absent equalities and unrestricted bounds.
Hessian conversion needs more memory and storage than the factor package.

## Validation and provenance

Portable coefficient arrays match the native capture exactly. Loaded costs,
gradients and constraint residuals agree with native evaluations at zero and
`x[j]=1e-4*((j%17)-8)` within the floating-point tolerances in pack.py. No
certified minimizer, reference optimum, or new solve-time claim is supplied.

Captures used GTDynamics `9b3d1e2a818f7251298b91c692e48bbcfae17738`
plus the recorded sparse-support Python changes, and GTSAM
`b20aa26730423e9dc46fe8c05ece7179a26332f2`, with TBB enabled in both.
That GTSAM revision includes diagnostic changes reported to regress other
datasets. It is capture provenance; do not import those solver changes.
The stored problems can be loaded without GTDynamics, MuJoCo, or source motion.

## Re-capture in the current local environment

`capture.cpp` is a Linux LD_PRELOAD diagnostic compiled against the captured
GTSAM ABI. It passes the first global call through unchanged, exports the
second, and exits with **86 on successful capture**. The default dimension
guard is 10,000 (`G1_QP_CAPTURE_MIN_DIM` can override it) to exclude small local
projection QPs. pack.py requires evidence of the first accepted nonlinear step.

```bash
GTSAM_SOURCE=/home/dellaert/.codex/worktrees/gtsam-qp2912/gtsam
c++ -std=c++17 -O2 -shared -fPIC capture.cpp \
  -I"$GTSAM_SOURCE" -I"$GTSAM_SOURCE/build" \
  -I"$GTSAM_SOURCE/gtsam/3rdparty/Eigen" \
  -L"$GTSAM_SOURCE/build/gtsam" -Wl,-rpath,"$GTSAM_SOURCE/build/gtsam" \
  -lgtsam -ltbb -ltbbmalloc -ldl -o capture.so
```

Put the matching built GTDynamics/GTSAM modules and g1_mjlab examples on
PYTHONPATH. Set `G1_QP_GTSAM_LIBRARY` to the pinned shared library,
`G1_QP_CAPTURE_DIR` to an existing OUTPUT/raw directory, and preload capture.so
and the TBB library recorded in provenance.json. Use py312 with
OPENBLAS_NUM_THREADS=1 and OMP_NUM_THREADS=1. Capture commands:

```bash
python capture_run.py --inputs /home/dellaert/git/GTDynamics \
  --output OUTPUT20 --name G1SP20 --start 78 --end 98
python capture_run.py --inputs /home/dellaert/git/GTDynamics \
  --output OUTPUTFULL --name G1SPFULL --full
```

Use a separate capture directory and environment for each command. Then run
`pack.py OUTPUT` without the preload. The temporary binary stream is native
little-endian data; the resulting NumPy/SciPy files are portable. The raw stream
is kept locally and excluded from the ZIPs.

Source/model attribution notices accompany the packages. No benchmark
submission or public upload has been made.
