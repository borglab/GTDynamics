# G1 forward dynamics: four CMC friction formulations

[Open the current report](results/report.html). The experiment compares pyramidal,
elliptic norm, signed-square, and friction-penalty formulations on the full
GTDynamics constraint graph, with fresh MuJoCo CPU and MJLab/Warp GPU timings.

The repository keeps the current self-contained report and its six compressed
JSON inputs. Regenerate the HTML without rerunning timings with:

```bash
python examples/g1_mjlab/report.py examples/g1_mjlab/results
```

The renderer accepts compressed checked-in inputs or plain JSON from a fresh run;
plain JSON takes precedence. [Historical comparison](results/history.md) retains
aggregate results and provenance in [history.json](results/history.json).
Old reports, videos, profiling/diagnostic drivers, and per-case historical data
are omitted. Model, policy, and trajectory files remain local and can be recreated
with the commands below. Build and timing logs are not checked in.

## Whole-walk kinematics example

[The student walkthrough](KINEMATIC_WALK.md) accompanies
[`kinematic_walk.py`](kinematic_walk.py). It builds a single contact-aligned
`Trajectory` graph for all 800 walking samples and runs an equality-only CM-Opt
solve with `--initializer contact` or `ik`.
[`compare_kinematic_walk.py`](compare_kinematic_walk.py) calls those example
functions to compare the initialization and solve timings (`--repeats 3`).

## Full graph and input boundary

[`full_model.py`](full_model.py) exports static link/joint parameters, q/v,
applied forces, collision points/frames, and the explicit compliant-contact
coefficients R and reference acceleration. It exports **no mass matrix, bias
force, body/contact Jacobians, or reference force/acceleration solution**.
The importer supports this G1's floating root, 29 hinge joints, 30 massive links,
and 3D contacts against fixed terrain. Unsupported models/constraints raise errors.

[`full_dynamics.cpp`](full_dynamics.cpp) builds a GTDynamics `Robot`, runs
`FKTraversal`, and builds:

- `TwistAccelFactor` for each joint's acceleration relation;
- `WrenchFactor` for each link's Newton–Euler balance;
- GTDynamics joint wrench-equivalence expressions;
- joint-effort equations including rotor armature;
- contact-force to COM-wrench equations, including moment arms;
- friction inequalities or the friction penalty, as described below.

Positions and velocities are fixed forward-dynamics inputs. Accelerations and
wrenches remain unknowns. The inequality formulations use the same physical
objective; the penalty formulation adds a friction cost.
Initialization solves the full equality graph with zero pyramid-edge forces or
1 N circular normal forces, without using the reference solution.

CMC's `IEOptimizer::identifyManifolds` discovers the connected component.
`EliminationBasis` selects contact-force keys as coordinates and computes the
recovery map by eliminating GTDynamics' equality factors. The benchmark supplies
neither a recovery matrix nor a manually condensed objective. Each CMC result
records the actual ambient and reduced dimensions.

The four formulations use the shared `PyramidalFrictionConeConstraint`,
`EllipticFrictionConeConstraint`, and `SignedSquareFrictionConeConstraint` from
`gtdynamics/constraints`, or `ContactDynamicsFrictionConeFactor` from
`gtdynamics/factors`. The penalty factor retains a hard normal nonnegativity
constraint and adds a soft tangential-cone cost with sigma 0.001. It changes the
optimization problem; passing its solver termination test does not guarantee
physical cone feasibility. Pyramid contact
keys store `Vector4` edge coefficients; elliptic keys store `Vector3` forces in
(tangent 1, tangent 2, normal) order. The importer, wrench maps, objective, and
output explicitly convert MuJoCo's (normal, tangent 1, tangent 2) ordering.


With the fixed q/v inputs, the dynamics equalities are affine. The manifold explicitly
reuses their constant numeric basis and caches the projection metric. The
retractor skips AL for already-feasible trials or feasible equality projections;
required blocking faces are still enforced. CMC now uses
`ReducedALRetractor`: it obtains the GTDynamics equality basis, whitens the
stay-close metric, and solves a reduced nonlinear projection with GTSAM
augmented Lagrangian. It evaluates the original inequalities at recovered
variables and chains their Jacobians through the basis. There is no QCQP
construction or quadratic-model callback. Elliptic friction retains its original
norm-valued expression in both the retraction and outer CMC tangent QP. The
signed-square variant evaluates its original signed-square residual in both.
Reduced AL is an internal retraction subproblem; the current four-way report
compares only CMC, without an ambient AL column.

The retractor explicitly rejects unconverged subproblems and checks the original
whitened constraint residuals. The original cone is still nonsmooth at zero
force; using AL does not solve that issue in general. See the
[retractor documentation](../../gtdynamics/cmcopt/README.md#affine-equality-manifolds-reduced-al-retraction)
for applicability, metrics, and tolerances.

The objective is assembled from local least-squares factors:

```text
minimize  1/2 sum_links (A_i - b_i)' I_i (A_i - b_i)
        + 1/2 sum_joints armature_j qdd_j^2
        + 1/2 f' R f - aref' f
subject to the full dynamics equalities and friction inequalities.
The penalty variant replaces the tangential inequality with a friction cost.
```

Here `b_i` is the spatial acceleration at zero generalized acceleration,
computed by GTDynamics kinematic propagation. The first two terms represent
`1/2 qdd' M qdd`, without assembling M. On the dynamics equalities, this gives
the same compliant-contact objective as MuJoCo. Independent tests check this
identity as well as accelerations and wrench signs.

Collision detection, contact frames, R and reference acceleration, and
actuator/passive applied forces are shared inputs. In particular, R and reference
acceleration are supplied by MuJoCo to define the matched contact law. This does
not test independent collision detection or contact-law parameter generation.
Rigid/no-slip contact is not substituted for MuJoCo's compliant law.

## Reproduce

Requirements: GTDynamics' build dependencies, a current GTSAM constrained module,
Boost.JSON, and an existing MJLab Python environment. GPU timings require an
NVIDIA GPU. Commands run from the repository root.

```bash
MJLAB=/path/to/mjlab
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGTDYNAMICS_BUILD_G1_MJLAB=ON
cmake --build build --target g1_full_dynamics -j4
build/examples/g1_mjlab/g1_full_dynamics --build-info
build/examples/g1_mjlab/g1_full_dynamics --self-test

# Reuse model.mjb, walk.npz and walk.json if already recorded.
uv run --project "$MJLAB" --no-sync python examples/g1_mjlab/download_policy.py examples/g1_mjlab/assets
uv run --project "$MJLAB" --no-sync python examples/g1_mjlab/record_walk.py \
  --checkpoint examples/g1_mjlab/assets/model_final.pt \
  --output examples/g1_mjlab/results --seconds 8 --speed 0.5

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 uv run --project "$MJLAB" --no-sync python \
  examples/g1_mjlab/validate_full.py --data examples/g1_mjlab/results \
  --solver build/examples/g1_mjlab/g1_full_dynamics \
  --output examples/g1_mjlab/results/four-way-validation.json
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 uv run --project "$MJLAB" --no-sync python \
  examples/g1_mjlab/compare.py --data examples/g1_mjlab/results \
  --solver build/examples/g1_mjlab/g1_full_dynamics --gtsam-source /path/to/gtsam \
  --samples 24 --repeats 3 --timeout 30
uv run --project "$MJLAB" --no-sync python examples/g1_mjlab/report.py examples/g1_mjlab/results
```

Use `--skip-warp` for CPU-only work, `--output /tmp/g1-probe` to keep trial results
separate. The four-way run always covers both MuJoCo cone models. The report rejects
incomplete or duplicate sample sets. Solvers run in separate subprocesses with
individual deadlines; failures remain in the saved JSON.

The policy is pinned to
[robomotic/mjlab-policies, revision f277ca8](https://huggingface.co/robomotic/mjlab-policies/tree/f277ca8/g1_velocity/2026-04-26_20-11-37).
The downloader checks SHA-256
`b45a629ce3b6c279201be60362408cb0586fac077e577964bd66a50c32fd4763`.
The recording disables startup domain randomization, observation noise, and
random pushes. It supplies fixed states; it is not a GTDynamics walking rollout.

## Validation and timing

`validate_full.py` tests zero, random signed, and reference forces across three
states, all four formulations, and both cone encodings (36 checks). MuJoCo
matrices/Jacobians are **validation oracles only**. Tests cover FK, twists, acceleration maps/bias, force signs and moment
arms, armature, equality residuals, the physical objective, and the actual
friction penalty value. Reference force vectors
occur only in this prescribed-force test mode, never in optimization runs.

The comparison requires acceleration error `<1e-3`, equality L2 `<1e-5`,
canonical physical cone violation L2 `<1e-4`, and force-space projected-gradient infinity norm `<1e-3`.
Linear acceleration is in m/s²; angular/joint acceleration is in rad/s².
Equality residuals combine unscaled physical units. Solver termination alone is
not a pass. Cone violation is computed directly from forces, independent of
the constraint residual scaling. The projected-gradient check uses the original
physical constrained objective, including for the penalty variant. Failures are
retained.

GTDynamics elapsed time includes robot construction, its own FK, graph assembly,
feasible initialization, component/basis construction, and optimization with
retraction. Collision/contact-law generation, JSON, and process startup are
excluded. Component construction is included in CMC optimization time and also
reported separately. Each state gets three repetitions without a reference warm
start, with rotating formulation order. Each state contributes its median
latency. A state passes only if every repetition passes; returned-state medians
include inaccurate results. Validated-only medians and failure counts are shown
separately, and their successful-state populations can differ.

Native CPU timings cover complete `mj_forward`. MJLab/Warp timings cover
synchronized CUDA-graph forward calls, reporting one-world latency and batched
throughput. Both reset solver warm starts and use five warmups, 30 timed
repetitions, 100 solver iterations and tolerance `1e-12`. Warp uses float32;
native MuJoCo uses float64. Their scopes differ from GTDynamics, so timing ratios
are not end-to-end simulator speedups.

The example is rebased on master `8184aff7`, which includes the solver work from
[PR #449](https://github.com/borglab/GTDynamics/pull/449) and the shared friction
constraints from [PR #450](https://github.com/borglab/GTDynamics/pull/450).
The solver work includes `ReducedALRetractor`, affine equality-basis reuse,
the manifold-owned projection metric cache, and dense active-set solves for both
CMC QP paths.
Configuration records build details, binary/source hashes, model/trajectory
hashes, and working-tree state at measurement.

The October 31, 2026 dense-QP four-way rerun validates 24/24 pyramidal, 9/24 elliptic norm,
6/24 signed-square, and 4/24 penalty-factor states. Elliptic norm and
signed-square reject 15 and 18 states respectively. The penalty factor returns
all 24 states, but 20 fail physical accuracy checks. Median returned-state
latencies are 17.63, 52.29, 34.30, and 101.36 ms respectively; these medians use
different populations and are not direct speedup ratios. All 36 independent
prescribed-force checks pass. Fresh MuJoCo CPU and MJLab/Warp measurements and
accuracy checks are included in the report.
