# CMC-Opt with an unconstrained global solve

## Result

CMC-Opt now eliminates the selected inequality faces inside each component's
local coordinates. Every trajectory-wide Gaussian solve receives ordinary soft
cost factors. This applies to both the default feasible-step approximation and
the optional active-set refinement. The global constrained Gaussian and QP
fallbacks have been removed from the LM step.

On the G1 mid-clip turns, optimization takes **32.5–33.7 seconds**, compared with
**78.1 seconds** for the original implementation: **2.32–2.40× faster**.
Total time from contact initialization improves by **2.05–2.11×**. The three
architectural runs produce all 20 saved numerical arrays bit-for-bit identical
to each other and to the earlier experimental implementation.

## Like I'm an undergrad

### The robot problem has measurements and physical constraints

The unknown vector $x$ contains robot poses, velocities, accelerations, forces,
torques, contact landmarks, and IMU bias. Measurements and soft priors tell us
which states fit the data. Exact mechanics and contact constraints describe
which states are allowed:

$$
\min_x E(x)=\frac12\sum_f\|r_f(x)\|^2,
\qquad h(x)=0,\qquad g(x)\leq0.
$$

The residuals $r_f$ are divided by their noise standard deviations. An encoder
factor, for example, penalizes disagreement with a measured joint angle. The
equalities $h$ include articulated kinematics and force balance. Inequalities
$g$ include the sole-wrench cone: normal contact force must be nonnegative,
and tangential forces and moments must stay within their allowed bounds.

CMC-Opt first groups variables connected by hard constraints into components.
A component can be large, but many components can be handled independently
when restoring feasibility. Soft measurement and temporal factors can still
connect those components. This is why the objective needs a global solve.

### CM-Opt removes equalities by choosing allowed coordinates

At the current state, the linearized equality constraints on component $i$ are
$J_{h_i}\Delta x_i=0$. We construct a basis $B_i$ whose columns satisfy

$$
J_{h_i}B_i=0,\qquad \Delta x_i=B_i\delta_i.
$$

The smaller vector $\delta_i$ describes an allowed infinitesimal motion. The
global optimizer works with these coordinates. Every choice of $\delta_i$
satisfies the linearized equalities automatically.

This is ordinary substitution from algebra. Suppose the problem is

$$
\min_{x,y}\frac12[(x-1)^2+(y-2)^2],\qquad x+y=0.
$$

Set $x=-t$ and $y=t$. We now minimize the unconstrained function
$\frac12[(-t-1)^2+(t-2)^2]$. Its minimum is $t=1/2$, so the constrained solution
is $(x,y)=(-1/2,1/2)$. The equality disappears because the parameterization
already satisfies it.

### CMC-Opt must also decide which inequality faces block motion

An inequality at its boundary permits motion into the feasible interior. For
example, at normal force $f_z=0$, an increment $\Delta f_z\geq0$ is allowed.
We should not permanently fix every tight inequality to equality.

The working set selects faces to hold at equality while choosing a step.
Once that set is fixed, its local equations are simply

$$
A_i\delta_i=0.
$$

We can eliminate them exactly as CM-Opt eliminates the mechanics equalities:

$$
A_iN_i=0,\qquad \delta_i=N_i\eta_i,
\qquad \Delta x_i=B_iN_i\eta_i.
$$

Here $\eta_i$ is the remaining free step. The new matrix $N_i$ depends on the
selected faces. A component with no selected faces keeps its existing
coordinates; one with no remaining freedom receives an exact zero increment.
Dependent face rows do not remove extra degrees of freedom.

### The objective remains globally coupled

Let $r+J\delta$ approximate the measurement residuals in the equality-manifold
coordinates. Levenberg–Marquardt (LM) adds damping to discourage excessively
large steps. For a fixed working set the old subproblem was

$$
\min_\delta \frac12\|J\delta+r\|^2
 +\frac\lambda2\|D\delta\|^2,
\qquad A_i\delta_i=0\text{ for each component }i.
$$

With $N=\operatorname{diag}(N_1,\ldots,N_m)$, substitution gives

$$
\min_\eta \frac12\|JN\eta+r\|^2
 +\frac\lambda2\|DN\eta\|^2.
$$

This is an unconstrained least-squares problem that GTSAM can solve directly.
The factors connecting different components still connect them after the
substitution, so their coupling is fully retained. For a fixed working set,
this is the same optimization problem in different coordinates.

**Damping must also be transformed.** The local kernels need not be
orthonormal. Replacing $DN$ by identity would change the LM step. The code adds
damping first, then transforms all cost factors together, preserving the
original metric.

### Face selection and nonlinear retraction remain necessary

The complete optimization has the following stages:

1. **Restore the initial estimate to feasibility.** Contact initialization
   supplies the configurations. Dynamics variables initially set to zero
   generally violate force balance, so component retractions establish a
   feasible starting point before the outer iterations begin.
2. **Linearize and select faces.** Build the equality-manifold coordinates,
   linearize the soft costs, and project the downhill gradient into each local
   tangent cone to obtain initial blocking faces.
3. **Solve in the free coordinates.** Build each $N_i$, substitute it into
   the damped costs, solve the unconstrained global graph, and recover
   $\delta_i=N_i\eta_i$.
4. **Check the working set.** The default algorithm adds any violated active
   inequality blocks and repeats stage 3. Optional refinement also releases
   faces and selects individual rows, as explained below.
5. **Retract the finite step.** Lift through $B_i$ and restore the nonlinear
   constraints locally. A tangent approximation alone does not satisfy curved
   constraints after a finite movement.
6. **Accept or reject the trial.** Compare the actual nonlinear objective
   decrease with the predicted decrease from the undamped quadratic model.
   LM adjusts damping and retries or advances the outer iteration.

The five reported outer iterations can therefore contain several linear solves
and many inner retraction iterations.

Optional refinement uses standard primal active-set logic. It moves toward the
current working-set minimum until another inequality blocks the step. At a
working-set minimum, the coupled objective gradient is already known. The
multiplier equations are component-local:

$$
\nabla_i q(\delta)+A_i^T\mu_i=0.
$$

For the convention $A_i\delta_i\leq0$, nonnegative multipliers support keeping
the selected faces. A negative multiplier identifies a face to release. These
small local linear equations use rank-revealing Eigen factorization. The next
working-set subproblem again has an unconstrained global solve. Numerical
failure or exhaustion of the refinement budget rejects the trial.

The G1 timing experiment keeps `iqpMaxIterations=0`, matching the original
configuration. It measures the default feasible-step approximation, not the
optional fully refined QP solution. Optional refinement is validated separately
against analytic and exhaustive test problems.

## Architectural changes in the code

| Responsibility | Previous implementation | Current implementation |
| --- | --- | --- |
| Cone representation in an LM state | Global lists of both ambient and manifold constraint factors | Immutable component-local matrices, cached across damping trials |
| Selected faces | Exact constrained factors appended to the global cost graph | Local null-space bases substituted into the costs |
| Global numerical solve | Mixed soft and constrained Gaussian elimination | Ordinary Gaussian least squares |
| Optional refinement | Trajectory-wide GTSAM QP, with alternative constrained backends | Working-set orchestration around local bases and unconstrained global solves |
| Numerical fallback | Return to global constrained elimination | Reject an unsuccessful LM trial |
| Nonlinear feasibility | Component retractions | Component retractions |

[`WorkingFace`](../../gtdynamics/cmcopt/WorkingFace.h) owns one component's
selected-face basis. Stateless helpers transform Gaussian costs, solve the
coupled objective, and recover component increments.

[`IEManifoldLinearization`](../../gtdynamics/cmcopt/IEManifoldLinearization.h)
owns the local geometry and supplies projection, face charts, boundary steps,
and multiplier candidates. [`IELMWorkingSet`](../../gtdynamics/cmcopt/IELMWorkingSet.h)
coordinates optional row-level refinement. It promotes only fully tight original
blocks to equalities during retraction; partially tight multirow blocks retain
inequality semantics. No global cone object or flattened constraint indices are
needed.

[`IELMOptimizerState`](../../gtdynamics/cmcopt/IELMOptimizerState.cpp) orchestrates
these operations. It no longer constructs an ambient copy of every active
inequality, assembles a global face-constraint graph, or constructs a global
`QpProblem`. Cached local rows also avoid repeated ambient-variable recovery
and constraint linearization during working-set feasibility checks.

GTSAM's `ActiveSetSolver` currently provides no callback for replacing its
working-set subproblem with component-coordinate elimination. The small
working-set loop therefore lives in GTDynamics. GTSAM remains the Gaussian
solver, and its existing QP machinery remains in component-local gradient
projection. Eigen supplies local rank-revealing linear algebra, as it already
does elsewhere in GTDynamics. No SciPy dependency was introduced.

The simplification is a stricter architectural boundary. It does not eliminate
active-set complexity, and the added coordinate and refinement helpers mean
the change is not a net reduction in total source lines. Independent tests
can now verify the global solver's unconstrained-input contract directly.

## Measurements

These measurements predate the rebase onto master at `e836e474`. That merge
includes the extracted articulated retractor, an example-specific G1 solver,
and a revised gauge. The numbers below compare the earlier implementations;
they do not measure the rebased example.

Measured 2026-11-03 on an Intel Core i7-14700F, Release Clang `-O3`, eight
retraction workers, and `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`. Every run
uses the same 78–98 s SSIO clip, 1,000 original 50 Hz slices, five outer
iterations, measurements, initialization, priors, damping, and constraints.
The original implementation is PR #468 commit
`4de63c2606da2d5be485a5f4424a2f590995758c`.

| Version | Optimization | Total from contact initialization | Optimization speedup | Total speedup |
| --- | ---: | ---: | ---: | ---: |
| Original implementation | 78.092 s | 86.909 s | 1.00× | 1.00× |
| Preserved experiment, run 1 | 33.276 s | 42.118 s | 2.35× | 2.06× |
| Preserved experiment, run 2 | 34.463 s | 43.248 s | 2.27× | 2.01× |
| Component-coordinate architecture, run 1 | 32.634 s | 41.396 s | 2.39× | 2.10× |
| Component-coordinate architecture, run 2 | 33.685 s | 42.430 s | 2.32× | 2.05× |
| Component-coordinate architecture, final run 3 | 32.547 s | 41.236 s | 2.40× | 2.11× |

Total includes contact initialization, measurement preparation and IMU
preintegration, graph assembly, feasibility restoration, optimization, and
error-history recording. It excludes input/model loading, result-array
extraction, serialization, and browser loading. The numerical runs contain
no profiling hooks.

Most of the speedup was already established by the experiment: local face
elimination, cost-factor compaction, and cached feasibility checks. The new
architecture preserves that gain while removing the constrained fallback and
extending the contract to optional refinement. These timings do not establish
a substantial additional speedup over the experiment. They also do not imply
the same gain for every robot or constraint pattern.

## Numerical checks

All 20 arrays saved by each architectural run exactly match the corresponding
arrays from the preserved experiment. Compared with the original constrained
solver, the final objective changes from `9123.931172484721` to
`9123.931184057772`, a relative difference of approximately $1.27\times10^{-9}$.
Both implementations report feasible solutions after five iterations.

The new equality residual norm is $4.14\times10^{-12}$ and the inequality
violation norm is $1.13\times10^{-7}$. Against the original solver, the maximum
joint-angle difference is approximately $0.000084$ degrees, the maximum joint
torque difference is $0.000076$ Nm, the maximum sole-force vector difference is
$0.000595$ N, and the maximum sole-moment vector difference is $0.000077$ Nm.
The maximum link-position difference is $1.12\times10^{-7}$ m. Maximum joint acceleration
differences are $0.00571$ rad/s², against a baseline maximum magnitude of
$181.24$ rad/s². The comparison therefore covers dynamic variables as well as
the displayed poses.

Validation includes:

- Fixed-working-set optima against constrained Gaussian QR, with coupled
  factors, nonunit noise, transformed damping, redundant/scaled rows, tiny
  kernel coefficients, zero-DOF components, and ordering preservation.
- Direct inspection that reduced graphs contain soft cost factors only, plus
  equality of the original and reduced quadratic costs under recovery.
- Sixty-four coupled QP cases checked against exhaustive enumeration of all
  working sets using constrained QR, with both empty and fully active starts.
- Face release, opposing and redundant rows, partial multirow blocks, failure
  on iteration-budget exhaustion, and rejection of nonlocal/affine cone rows.
- Existing CMC manifold, tangent-cone, reduced AL retraction, fixed dynamics,
  and coupled kinodynamics tests: all eight relevant native suites pass.
- Four direct Python binding tests for coupled and fixed dynamics pass.

Optional refinement can exhaust its working-set budget on difficult or
degenerate cases. It reports failure and lets LM handle the unsuccessful trial;
it does not silently switch to a global constrained backend. The original G1
benchmark disables that optional refinement, so its timing and exact-array
agreement do not measure refinement performance on a full trajectory.

## Reproduction

The numerical example remains the student-facing entry point. The command
below now uses the revised G1 solver and gauge from master, so it will not
reproduce the historical numerical results above:

```bash
python examples/g1_mjlab/kinodynamic_walk.py \
  --start 78 --end 98 --max-iterations 5 --retraction-threads 8 \
  --output /tmp/kinodynamics-architecture
```

Use the configured `py312` environment, matching GTSAM/GTDynamics Python
bindings, and prepared G1 model/data cache. In this workspace,
`/tmp/run-kinodynamics` supplies those paths and the thread environment.
Measured outputs are in `/tmp/kinodynamics-architecture-{1,2,3}/`; the machine-
readable comparison is `/tmp/cmc-architecture-comparison.json`.


The subsequent component-ownership refactor has a separate
[downloadable HTML report](../../gtdynamics/cmcopt/ComponentRefactorReport.html),
with the full example inventory and matched before/after measurements.
