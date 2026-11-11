# CMC example validation and timing

For a same-QP experiment comparing variable-splitting ADMM (point-local cone
projections with a reused global factorization) against ProxQP, see
[PointSplitADMM.md](PointSplitADMM.md). The new comparison is opt-in and does
not replace the historical baseline/candidate study below.

These drivers execute the repository's original optimization problems against
separate baseline and candidate builds. They save complete numerical outputs,
process failures, and timing samples. The native script mode exits before plots
and unrelated comparison solvers; it preserves optimizer settings and seeds.
It also skips optional post-solve optimization of saved intermediate iterates,
which the original helper uses only for projected-cost diagnostics.

`run.py` covers all fifteen `rss01`–`rss04` and `yetong01`–`yetong11` programs.
The RSS comparisons also run their CM penalty variants through the same LM engine.
The first four Yetong examples also run gradient descent. Yetong08 runs both
its multistage experiment and its active retractor comparison; a separate
`basis-retractor` case runs the second independent configuration even when the
first times out; Yetong09 runs
both fixed and varying projection metrics. Yetong10 and Yetong11 retain their
optional QP refinement. Commented-out experiments are not executable cases.

`g1_cases.py` covers all four friction formulations and all three full-dynamics
retractors at 24 recorded states. It also runs fixed-kinematics dynamics in hard
and soft base-balance modes, full 78–98 s coupled kinodynamics, a one-iteration
full-clip diagnostic, and a five-iteration 78–79 s diagnostic. The full case is
retained even if it fails. The diagnostics do not substitute for it.

The other search hits are callers or presentation code: `compare.py` and
`four_way_compare.py` invoke `g1_full_dynamics`; `kinodynamics_native.cpp` is the
backend of `kinodynamic_walk.py`; `fixed_kinematics.py` prepares inputs; report,
plotting and app exporters present the same solves. `main_connected_poses.cpp`
uses CM-Opt, not CMC-Opt. Notebook output text is not an additional example.

Build both versions in Release mode with the same compiler and GTSAM checkout.
The baseline is `f71c907d`. Each snapshot has `bin/`, `lib/libgtdynamics.so`,
`python/gtdynamics/`, and `native/_g1_kinodynamics*.so`. Verify `ldd` before
running: old executables with `DT_RPATH` can override `LD_LIBRARY_PATH`.

```bash
python examples/cmc_benchmarks/run.py \
  --binaries /tmp/cmc-baseline/bin --library /tmp/cmc-baseline/lib \
  --output /tmp/cmc-suite-before
python examples/cmc_benchmarks/g1_cases.py \
  --snapshot /tmp/cmc-baseline --requests /tmp/cmc-g1-requests.json \
  --walk-python-path /tmp/g1-pose-python/lib/python3.12/site-packages \
  --output /tmp/cmc-g1-before
```

Use `--paired-binaries`, `--paired-library`, and `--paired-output` with `run.py`,
or `--paired-snapshot` and `--paired-output` with `g1_cases.py`, to alternate
baseline/candidate order. `--resume` skips recorded trials. Each successful case
has one warmup and five measured repeats. A pair that times out in both
warmups is recorded as censored without repeating the same timeout five times.
The report retains those failed warmups, excludes them from timing statistics,
and does not claim a solution comparison. Full-dynamics trials have a 30-second
watchdog; trajectory and native-script timeouts are separately configurable. For G1, generate the immutable request JSON once with the
MuJoCo version in the recording metadata (the archived comparison uses 3.16.0).
The walking examples use their pinned 3.11.0 model environment. Both use Python
3.12. Save large arrays and logs outside the repository.

Native script timing is `IEResultSummary.optimization_time`; helper wall time
also includes result assessment, and process wall time includes construction
and export. G1 full-dynamics reports its solver and assembly times separately;
walking examples report their existing native timing scopes. The historical
kinodynamics cases explicitly select `--initialization contact` so that their
initializer choice is independent of future defaults. New runs use the shared
single/double-support sampler; historical reports retain their original sampling.
Paired comparisons must use the same sampling policy. Do not compare
unlike scopes or count warmup time. Timeout and failure are explicit outcomes,
never zero-time successes. Compare final objectives, individual factor errors,
constraint residuals, final coordinates/arrays, and iteration counts alongside
timing. Recheck apparent regressions above 5% with paired, isolated runs.

The controlled run pins native examples to logical CPU 2 and distributes G1 case pairs
over logical CPUs 4, 6, 8, and 10
(on separate physical cores). Baseline and candidate alternate order each repeat.
This avoids the TBB oversubscription observed in the exploratory batches;
`OMP_NUM_THREADS` alone does not limit TBB. Affinity is an explicit benchmark
condition, not a production default. Kinodynamics retains its configured eight
retraction workers under that affinity. Inspect `lscpu -e` before selecting CPUs
on another machine.


Generate the standalone report after both runs:

```bash
python examples/cmc_benchmarks/report.py \
  --before /tmp/cmc-suite-before /tmp/cmc-g1-before \
  --after /tmp/cmc-suite-after /tmp/cmc-g1-after \
  --g1-requests /tmp/cmc-g1-requests.json \
  --output /tmp/ComponentRefactorReport.html
```

The report contains an embedded JSON download and needs no server or CDN.
Pass `--notes`, `--provenance`, and repeatable `--recheck` JSON reports to retain
experimental qualifications, binary fingerprints, and repeated timing batches.
The benchmark instrumentation must be compiled into both snapshots from the
same sources, using each snapshot's own GTDynamics headers and library.

The snapshot Python package must include `sim.py` as well as `__init__.py` and
the extension module. Check imports before a timed run. Keep the baseline
library and extension ABI matched; copy the package's ordinary Python files
alongside the extension rather than synthesizing a replacement package.

For an expensive independent case, `run.py --trial N --cpu C --resume` runs
only measured pair N after a separate warmup. Multiple pairs may be placed on
distinct physical cores. Record that concurrency explicitly: those timings
measure a concurrent workload, even though each before/after pair shares a core.
The current report uses this for the standalone Yetong08 basis-retractor case.
