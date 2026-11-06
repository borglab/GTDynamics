# Equality coordinates and the global tangent-cone QP

CMC separates component geometry, the global linear step, and nonlinear
retraction. This solver replacement is based on PR 472; component/value APIs
and retraction algorithms remain unchanged.

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
ordering and elimination; LM `ordering` and `linearSolverType` no longer control
the global linear step.

The solver owns row additions, releases, and multipliers. GTDynamics no longer
constructs an active-face kernel $N$, transforms the objective through $N$,
or implements an active-set iteration. `qpMaxIterations` replaces
`iqpMaxIterations` and defaults to 1,000. Full convergence is the default.
`qpUseProjectedGradientInitialization` seeds this full solve with GTSAM's
componentwise projection of the negative gradient (default false).

With `qpUseFeasibleStep=true`, each trial calls
`ActiveSetSolver::computeFeasibleStep` instead. GTSAM always uses projected-gradient
initialization in this mode and returns at feasible descent, without certifying
optimality or multiplier validity. `NoProgress` supplies the original zero
increment to the existing LM stopping logic. Numerical failures and iteration
limits reject the trial. This mode does not add a GTDynamics working-set solver.

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
