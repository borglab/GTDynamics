# Component coordinates for the CMC-Opt linear step

## Understanding of the previous implementation

This note describes the previous CMC-Opt linear step and its replacement by
component coordinates. The branch is rebased onto master after PR #468 merged
at `e836e474`. The performance measurements below were taken against the earlier
PR #468 commit `4de63c2606da2d5be485a5f4424a2f590995758c`, before the
articulated-retractor extraction and gauge changes. They are historical
measurements, not a benchmark of the rebased G1 example. The code convention is
$g(x)\leq0$.

CMC-Opt already discovers connected components of the hard equality and
inequality constraints. Each `IEConstraintManifold` owns one component,
an equality tangent basis, the active inequality boundaries, and its nonlinear
retractor. The objective is transformed to a factor graph over these manifold
components and any unconstrained variables. Objective factors can connect
different components.

For component $i$, let $x_i$ denote its original variables and let $B_i$ be the
existing equality tangent basis. A tangent increment has the form

$$
\Delta x_i=B_i\delta_i,\qquad J_{h_i}B_i=0.
$$

The basis accounts for the original mechanics equalities. Active inequalities
produce a local tangent cone $G_i\delta_i\leq0$, with
$G_i=J_{g_i}B_i$. A constraint involving several original variables still
belongs to one component, so its transformed Jacobian has only one component
key. `linearActiveManifoldInequalityConstraints()` constructs exactly these
unary, zero-right-hand-side factors.

The previous LM implementation performed the following operations:

1. Linearize the manifold objective and add the requested LM damping.
2. Project the negative gradient into each component's tangent cone to obtain
   an initial set of blocking inequality blocks.
3. Convert selected blocks to exact equality factors, append those factors
   to the global Gaussian objective, and call `SolveLinear`.
4. Lift the resulting increments to original variables, identify further
   blocking inequalities, and repeat until the step is directionally feasible.
5. If `iqpMaxIterations>0`, refine the feasible step with a trajectory-wide
   GTSAM QP. `robustQp` tried dense and sparse active-set backends and, for
   certain numerical failures, a native AL fallback.
6. Pass the component increments and blocking sets to the nonlinear
   retractors. Evaluate the original objective and perform the usual LM
   acceptance and damping update.

With `iqpMaxIterations=0`, step 4 is a feasible-step approximation; it does not
release faces to certify the minimum of the complete cone-constrained QP.
The proposed implementation retains that default behavior.

## Issue: local face equations reached the global solver

For a chosen working set, every added equality is local to one component.
Nevertheless, the previous representation gave those equations to a general
global constrained solver. This is a valid formulation, but it makes the
global solve enforce conditions that can already be eliminated locally.

In the 1,000-slice G1 benchmark, the original optimization took 78.1 seconds.
Profiling attributed approximately 45.8 seconds to constrained Gaussian solves
and another 9.8 seconds to repeated ambient-space feasibility checks. These
measurements motivate the change; they are not assumptions in the equivalence
argument below.

The objective's coupling between components must be preserved. Independently
projecting an unconstrained global solution afterward would generally change
the coupled quadratic minimizer. The proposed method instead parameterizes
the feasible working face before the global minimization.

## Architectural simplification

The change introduces a second, component-local coordinate map for selected
inequality faces, while retaining the existing equality basis and retractor:

$$
\eta_i \xrightarrow{N_i} \delta_i \xrightarrow{B_i} \Delta x_i.
$$

- `ComponentTangentSpace` constructs a basis $N_i$ for the kernel of each
  selected face matrix. It substitutes these bases into the already-damped
  soft costs, calls GTSAM's ordinary Gaussian solver, and recovers $\delta_i$.
- `TangentConeLinearization` caches the homogeneous inequality rows and their
  original component keys. It checks directional feasibility and constructs
  working-face parameterizations. Its optional refinement manages row-level
  working sets around the same unconstrained global solve.
- `IELMState` stores this immutable cone linearization across damping trials.
  It no longer stores a second ambient-space constraint graph.
- `IELMTrial` no longer appends constrained factors or constructs a global
  `QpProblem`. An invalid or unsuccessful reduced solve is rejected; it cannot
  fall back to global constrained elimination.

All trajectory-wide quadratic minimizations therefore receive soft Gaussian
costs. Local cone projection and nonlinear component retraction keep their
existing constrained numerical methods. Their constraints remain inside their
components.

The new $N_i$ is temporary working-set data. The existing $B_i$ and manifold
representation remain intact, and retraction still receives increments in the
original coordinates. This keeps face changes within the linear-step code and
preserves the retractor's interpretation of its input and metric.

## Mathematical equivalence for a fixed working set

### Definition of the two problems

Work in the existing equality-manifold coordinates $\delta$, after applying
$B=\operatorname{diag}(B_1,\ldots,B_m)$. The whitened, linearized measurement
residual is $J\delta+r$. At damping value $\lambda$, define

$$
q_\lambda(\delta)
=\frac12\|J\delta+r\|^2
 +\frac\lambda2\|D\delta\|^2.
\tag{1}
$$

$D$ is the damping matrix already chosen by the existing implementation.
It can represent isotropic or diagonal-Hessian damping.

For a fixed working set $W$, stack the selected rows on component $i$ into
$A_i$. Its equations are homogeneous because these are tangent directions at
the current boundary, rather than an affine correction to the nonlinear
constraint value. Write $A=\operatorname{diag}(A_1,\ldots,A_m)$, allowing
zero-row blocks for unconstrained components. The previous subproblem was

$$
\min_\delta q_\lambda(\delta)
\quad\text{subject to}\quad A\delta=0.
\tag{2}
$$

Choose a full-column-rank $N_i$ whose columns span $\ker A_i$, and set
$N=\operatorname{diag}(N_1,\ldots,N_m)$. The new subproblem is

$$
\min_\eta q_\lambda(N\eta)
=\min_\eta\left[
\frac12\|JN\eta+r\|^2
 +\frac\lambda2\|DN\eta\|^2\right].
\tag{3}
$$

### Proposition and proof

**Proposition.** The map $\eta\mapsto N\eta$ is a bijection from the reduced
coordinates to the feasible set of (2). Consequently, it maps the complete
set of minimizers of (3) to the complete set of minimizers of (2), without
changing any objective value.

**Proof.** Since $AN=0$, every reduced vector gives a feasible increment.
Conversely, if $A\delta=0$, each $\delta_i$ lies in $\ker A_i$ and therefore
has a representation $N_i\eta_i$. Full column rank makes that representation
unique. Finally, (3) evaluates the same function (1) at that increment.
Feasible points and their objective values thus correspond exactly. ∎

This argument does not require independent constraint rows or orthonormal
bases. If the quadratic is positive definite on $\ker A$, the recovered
increment is the unique minimizer. For a rank-deficient objective, the
minimizer sets still correspond, but different numerical solvers need not
choose the same member. The production solve retains GTSAM's numerical
failure handling for an underdetermined reduced system.

### Relation to the original KKT system

Let $H=J^TJ+\lambda D^TD$ and $c=J^Tr$. The old equality-constrained optimum
satisfies

$$
H\delta+c+A^T\mu=0,\qquad A\delta=0.
\tag{4}
$$

Substitute $\delta=N\eta$ and premultiply the first equation by $N^T$:

$$
N^THN\eta=-N^Tc.
\tag{5}
$$

Equation (5) is the normal equation of (3). Conversely, (5) implies that
$HN\eta+c$ is orthogonal to $\ker A$. Since
$(\ker A)^\perp=\operatorname{range}(A^T)$, a multiplier satisfying (4)
exists, including when $A$ has dependent rows. This explains why the global
solver can omit the multiplier and constraint equations.

### Cross-component coupling is retained

The block linking components $i$ and $j$ in the reduced Hessian is

$$
\widetilde H_{ij}=N_i^T H_{ij}N_j.
\tag{6}
$$

It is not discarded when $i\ne j$. All temporal, collocation, IMU, landmark,
and other cross-component objective factors remain in the global problem.
Local construction of $N_i$ therefore preserves the globally coupled optimum.

### Damping and constants are retained

The code constructs LM damping in the existing $\delta$ coordinates, then
reduces the complete cost graph. Thus the damping becomes $\|DN\eta\|^2$.
Using $\|\eta\|^2$, or recomputing a new Hessian-diagonal damping matrix after
the reduction, would define a different step. Neither is done here.

Factors having identical key scopes are stacked with their original noise
models. This preserves their summed quadratic, gradient, and Hessian. When
every variable in a factor has zero remaining freedom, that factor contributes
only a constant; it is omitted from the solve. Actual and predicted reductions
are evaluated using the original graphs, so these constants cannot alter LM
acceptance. The predicted reduction continues to use the **undamped** cost.

## Extension to face selection and nonlinear iteration

For the default add-blocks loop, equivalence follows by induction when both
implementations make the same numerical rank and boundary decisions. The
initial working set comes from the same component gradient projection. For
each fixed set, the proposition gives the same unique increment. Checking
$G_i\delta_i$ is algebraically the same as first recovering $B_i\delta_i$
and checking $J_{g_i}B_i\delta_i$. The same violated blocks are therefore
added, using the original raw directional threshold of $10^{-5}$.

When this loop terminates, it supplies the same component increments and
working blocks to the same nonlinear retractors. In exact arithmetic, under
the uniqueness and decision assumptions above, the nonlinear trial state,
predicted reduction, actual objective, and LM acceptance decision are the
same. This change does not add a measurement, drop a physical constraint,
alter a noise model, or introduce a new penalty objective.

Optional refinement still targets

$$
\min_\delta q_\lambda(\delta)
\quad\text{subject to}\quad G_i\delta_i\leq0\quad\forall i.
\tag{7}
$$

Its working-set subproblems use (3). A primal line search moves toward the
working-set minimizer until another inequality blocks it. At a working-set
minimum, the gradient of the coupled quadratic is known and the multiplier
equations separate by component:

$$
A_i^T\mu_i=-\nabla_i q_\lambda(\delta).
\tag{8}
$$

With the $G_i\delta_i\leq0$ convention, a negative selected-face multiplier
indicates a row to release. GTSAM's previous active-set implementation used
the opposite multiplier sign, $A_i^T\nu_i=\nabla_i q_\lambda$; hence its test
for a positive leaving multiplier is the same sign test with $\nu_i=-\mu_i$.
Feasible primal values, nonnegative active multipliers, and stationarity certify
an optimum of the convex QP. With a unique optimum, converged old and new
refinements recover the same increment.

GTSAM's current `ActiveSetSolver` has no hook for replacing its working-set
subproblem solve with these local parameterizations. This implementation keeps
the short primal working-set loop in GTDynamics, uses Eigen for the local
linear algebra, and uses GTSAM for every global Gaussian solve. Component-local
gradient projection continues to use the existing GTSAM QP machinery.

## Exact equivalence and numerical behavior

The proof establishes equality of the mathematical subproblems. It does not
claim identical floating-point arithmetic or identical finite-budget behavior
for every degenerate active set. The following differences are explicit:

| Detail | Scope of the claim |
| --- | --- |
| Kernel construction | Full-pivot LU with row normalization spans the same kernel in exact arithmetic; numerical rank decisions can differ from constrained elimination. |
| Sparse basis multiplication | Every nonzero coefficient is retained, including tiny entries; there is no sparsification threshold approximation. |
| Dependent selected rows | The primal feasible space is unchanged. Multipliers can be nonunique, so local pivoting can choose a different row to release. |
| Optional QP refinement | The same convex objective and cone are solved, but iteration order, degenerate face choices, and convergence within a finite budget can differ. |
| Retraction blocking sets | Only fully active blocks become nonlinear equalities. Different valid working sets at a degenerate optimum can lead to different subsequent retractions. |
| Numerical failure | The old optional global QP fallbacks are removed. Reduced-solve failure or iteration exhaustion rejects the trial. |
| Final feasibility check | A failed check now rejects the trial; the previous `recordSolution` called the check but ignored its return value. |
| Directional scaling | Default block discovery retains raw rows and its $10^{-5}$ tolerance. Optional row refinement retains whitened rows and $10^{-7}$ tolerances. |

The simplification is therefore an exact coordinate elimination, accompanied
by explicit changes to solver organization and numerical failure policy.
Reviewing those policies separately from equations (2)–(5) avoids treating an
empirical timing result as a proof of full algorithmic identity.

## Validation and performance

`testComponentTangentSpace` compares recovered solutions with the previous
constrained Gaussian QR formulation. It covers cross-component costs, nonunit
noise, anisotropic damping, redundant and scaled rows, tiny kernel coefficients, zero-DOF
components, ordering, native Hessian costs, and rejection of constrained noise
in the reduced objective. It also checks quadratic-cost equivalence and the
unconstrained-factor contract directly.

`testTangentConeLinearization` checks 64 coupled QP cases against an exhaustive
working-set oracle using constrained QR. Additional cases cover opposing and
dependent rows, row release, partially active multirow blocks, iteration
exhaustion, multiple blocks on one component, and invalid component assembly.
The native regression targets for the rebased implementation are:

```text
testComponentTangentSpace   testTangentConeLinearization
testIEManifoldOptimizer     testIEConstraintManifold
testTangentCone             testReducedALRetractor
testFixedKinematicsDynamics testArticulatedRetractor
testCMCImprovements        testKinodynamicGauge
```

The G1 benchmark uses 1,000 original 50 Hz slices from SSIO seconds 78–98,
five outer iterations, and eight retraction workers on an i7-14700F. The
default `iqpMaxIterations=0` is unchanged. No numerical profiling hooks are
present in these timing runs.

| Implementation | Optimization | Total from contact initialization |
| --- | ---: | ---: |
| Earlier PR #468 baseline (`4de63c26`) | 78.092 s | 86.909 s |
| Component coordinates, run 1 | 32.634 s | 41.396 s |
| Component coordinates, run 2 | 33.685 s | 42.430 s |
| Component coordinates, run 3 | 32.547 s | 41.236 s |

The measured optimization speedup is **2.32–2.40×**; the total speedup is
**2.05–2.11×**. Final objective values differ from the baseline by
$1.27\times10^{-9}$ relative, and the maximum link-position difference is
$1.12\times10^{-7}$ m. Every new run reports feasible mechanics and contact
constraints. All 20 saved arrays are bit-for-bit identical across the three
new runs and the preserved local-elimination experiment.

The benchmark measures the combined effects of local elimination, factor
compaction, and cached local checks. It does not isolate the gain of each
change, establish full convergence in five outer iterations, or benchmark
optional QP refinement on the complete trajectory.

The [undergraduate explanation and benchmark report](../../examples/g1_mjlab/cmcopt_architecture_undergrad.md)
gives the timing scopes, numerical comparisons, and reproduction command.
For review, the key correspondence is between equations (2)–(5),
[`ComponentTangentSpace`](ComponentTangentSpace.cpp), and the working-set
orchestration in [`TangentConeLinearization`](TangentConeLinearization.cpp).
