# Equality coordinates and the global tangent-cone QP

CMC separates [runtime types and points](IEConstraintManifold.md), numerical tangent
geometry, the global linear step, and nonlinear retraction. A `IEConstraintManifold`
defines constraint factors and operations; a `CMPoint` contains numerical state
and ambient bindings; its `CMTangentSpace` owns B and active inequality rows.
This type/value refactor builds on PR 476 and preserves the QP algorithms.

## Equality reduction

Each component supplies its existing equality basis $B_i(x)$, with
$Dh_i(x)B_i(x)=0$. Its increment is $\Delta x_i=B_i\xi_i$. Unconstrained
variables retain their original tangent coordinates. Nonlinear objective
factors are linearized in these coordinates, including all cross-component
and temporal couplings. Costs with identical key scopes are combined before
conversion to QP costs, retaining their square-root factors and compaction.
No global dense matrix is assembled.

## QP assembly and solve

At state construction, `IELMState::tangentQp` stores the undamped Gaussian
objective and the cached active inequality rows. Each constraint is
$Dg_i(x)B_i\xi_i\le0$, using GTSAM's nonpositive inequality convention and
applying its sigma once. Explicitly selected near-boundary modes keep the
same homogeneous constraint as before. Slack inequalities are omitted.

Each damping trial copies the QP and adds the existing identity or Hessian-
diagonal damping in $\xi$ coordinates. It calls
`ActiveSetSolver::optimizeWithState` with the sparse subproblem solver, zero
initial increments, and zero additional regularization. The QP has no equality
constraints: those were already eliminated by $B$. GTSAM selects its internal
elimination. When supplied, LM `ordering` is forwarded after removing
zero-dimensional component keys; GTSAM validates the remaining complete
permutation. An unset ordering selects automatic COLAMD. LM `linearSolverType`
does not select the global QP backend.

`IELMParams::qpBackend` can instead select the optional native `ProxQP` backend,
built with `GTDYNAMICS_WITH_PROXQP=ON` (see the root README for installation).
This backend requires full solves with projected-gradient initialization off.
It sparsely scatters the QP costs' existing local Hessian blocks, preserves the
inequality block/row order, and checks solved status plus original-factor KKT
residuals before lifting the step. ProxQP multipliers and tight row residuals
identify active rows. The supplied ordering controls scalar packing; ProxQP
chooses its own sparse elimination order. Numerical failure rejects the trial
without switching backends. Its `qpMaxIterations` controls outer iterations;
reported `qpIterations` is ProxQP's total inner iteration count.

The solver owns row additions, releases, and multipliers. GTDynamics no longer
constructs an active-face kernel $N$, transforms the objective through $N$,
or implements an active-set iteration. `qpMaxIterations` replaces
`iqpMaxIterations` and defaults to 1,000. Full convergence is the default.
Full solves use GTSAM's default `Automatic` formulation, which selects Dual for
cold QPs with inequalities and invertible reduced curvature. Equality-only QPs
use Primal. `qpUseProjectedGradientInitialization` maps to GTSAM's
`QpInitialization::ProjectedGradient` or `TightConstraints` (default); this
initialization choice applies to primal solves and is ignored by Dual.

With `qpUseFeasibleStep=true`, each trial calls
`ActiveSetSolver::computeFeasibleStep` instead. This API always uses Primal. It
honors the same projected-gradient initialization flag and returns at feasible
descent, without certifying
optimality or multiplier validity. `NoProgress` supplies the original zero
increment to the existing LM stopping logic. Numerical failures and iteration
limits reject the trial. This mode does not add a GTDynamics working-set solver.
With projection disabled, all tight rows enter the initial working set; feasible
descent does not perform multiplier-based releases or establish stationarity.

Zero-dimensional components are omitted from the QP variables and restored as
zero increments. An entirely fixed problem needs no QP call. Failures reject
the LM trial, with no dense or AL fallback for the global step. Programming
errors continue to propagate. Returned steps must be finite and pass the
existing component directional-feasibility and selected-face checks.

## Retraction and diagnostics

A contiguous QP-row mapping identifies the original component and nonlinear
constraint block. A block is passed to retraction as an equality only if every
row is active. Partial blocks remain inequalities. The step is lifted through
$B$ before the existing nonlinear retractors restore feasibility, including
inactive boundaries encountered by the finite step.

The original undamped Gaussian model determines predicted reduction. Retraction,
nonlinear objective evaluation, LM acceptance, and damping updates retain their
existing roles. `qpIterations` counts returned active-set iterations, not
factorizations. Local tangent-cone projection remains available to GD.

## Validation and historical measurements

`testIELMQp` checks the global path against analytic solutions and exhaustive
hard-constrained QR candidates, including partial blocks, free variables,
zero-dimensional components, damping, failures, and slack-boundary retraction.
`testIEManifoldOptimizer` checks boundary release and index/sign preservation.

The archived `ComponentRefactorReport.html` and `.json` describe earlier
implementations and remain historical measurements. They do not establish
performance or bit-identical results for this replacement: the former default
stopped at a feasible approximation, whereas the default replacement solves
the full tangent-cone QP. The G1 example now explicitly requests feasible steps
through GTSAM. Sparse-backend dependent-row robustness must be
validated before this replacement is considered ready. See
[SparseQpValidation.md](SparseQpValidation.md) for the current blocker and the
bounded G1 comparison.
