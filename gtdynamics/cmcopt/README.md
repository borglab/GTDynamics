# CMC-Opt Math to Code Map

This note maps the main mathematical objects behind CMC-Opt to the implementation in `gtdynamics/cmcopt`.

CMC-Opt extends equality-only CM-Opt to manifolds with inequality boundaries and corners. The key extra ingredients are:

- an active set of tight inequalities,
- a tangent cone instead of only an equality tangent space,
- a projection step that respects that cone, and
- a retraction that can promote blocking inequalities to temporary equalities.

[Component coordinates for the linear step](ComponentCoordinates.md) explains
the equality reduction, global sparse QP, and nonlinear retraction boundary.

## Runtime types and values

[IEConstraintManifold, CMPoint, and CMTangentSpace](IEConstraintManifold.md) define the public
ownership boundary. The type owns factor definitions and operations; points own
ambient identities and numerical state; tangent spaces own point-dependent
geometry. Tangent increments are ordinary vectors. Automatic discovery remains
the default; manual sharing and compiled implementations use the same optimizer.

## Source Note

Equation references below are from Yetong Zhang's June 2024 thesis, *Constraint Manifold Optimization for Robotic Inference and Planning with Constraints*. The CMC-Opt material is primarily Chapter 4; metric-aware and approximate retraction variants are in Chapter 5.

## 1) Problem Statement

CMC-Opt solves problems of the form

$$
\min_x \sum_i \|f_i(x_i)\|^2
\quad \text{s.t.} \quad
h_j(x_j)=0,\; g_k(x_k) \ge 0.
$$

This is the factor-graph NLP form in thesis Eq. (4.1), using the thesis sign convention for feasible inequalities.

The implementation uses GTSAM's opposite convention: an inequality expression
`c(x)` is feasible when `c(x) <= 0`, so `c = -g` for the thesis notation used in
this note. `GeqZero(f)` constructs `c = -f`; `LeqZero(f)` constructs `c = f`.
Consequently, the implemented tangent cone uses `Dc(x) B xi <= 0`, and a positive
directional derivative identifies an outward step. The local gradient-projection QP uses
`LinearConstraint::LessEqual` with GTSAM's `ActiveSetSolver`.

The selected QP backend owns working-set selection and multiplier calculations.
CMC consumes its returned active rows and promotes only fully active nonlinear
blocks during retraction. The global LM QP uses GTSAM's sparse backend by default,
or the optional native ProxQP backend.

Constraint sigmas are applied once to the raw expression Jacobian. Reading an
already-whitened equality linearization and attaching the sigma again would
double the scaling. The sign and scaling regressions cover both bound senses,
non-unit sigmas, and a coupled QP whose initial blocking face must be released.

In code, the top-level optimizer API takes exactly those three pieces:

- cost graph,
- equality constraints,
- inequality constraints.

Pointers:

- [IEOptimizer](IEOptimizer.h#L31)
- [IEOptimizer::optimize](IEOptimizer.h#L43)
- [IEOptimizer::identifyUnconstrainedValues](IEOptimizer.h#L78)

The public `optimize(...)` entry point builds inequality/equality manifolds from the constraints, keeps unconstrained variables separate, and then hands the transformed problem to either the LM or GD variant.

## 2) Constraint-Connected Components with Inequalities

As in CM-Opt, the code first groups variables into constraint-connected components. For CMC-Opt, those components are built from both equality and inequality constraints.

Mathematically, each component carries a local feasible set

$$
\mathcal{M}_{IE} = \{x \mid h(x)=0,\; g(x)\ge 0\}.
$$

This is the local component version of the CMC state space in thesis Eq. (4.2), and matches the constraint-connected component construction in Eqs. (4.33)-(4.34).

Pointers:

- [identifyConnectedComponents](IEOptimizer.cpp#L87)
- [IEOptimizer::identifyComponents](IEOptimizer.cpp#L127)
- [CMPoint](CMPoint.h#L25)

`IEOptimizer::identifyComponents` builds one [CMPoint](CMPoint.h#L25) per connected component. Each point references a separate immutable `IEConstraintManifold` definition and owns the current numerical state and bindings.

## 3) CMPoint: Equality Basis + Active Set + Cone

The core CMC-Opt state for one component is:

$$
(x,\; \mathcal{A}(x),\; T_x^e\mathcal{M},\; \mathcal{C}_x),
$$

where:

- $x$ is the current component value,
- $\mathcal{A}(x)$ is the active set of tight inequalities,
- $T_x^e\mathcal{M}$ is the equality tangent space,
- $\mathcal{C}_x$ is the tangent cone induced by the active inequalities.

Pointers:

- [CMPoint](CMPoint.h#L25)
- [CMPoint::activeIndices](CMPoint.h#L98)
- [IEConstraintManifold::identifyActive](IEConstraintManifold.cpp)
- [IEConstraintManifold::linearize](IEConstraintManifold.cpp)
- [TangentCone](TangentCone.h#L25)

Implementation detail:

- The equality tangent basis belongs to the point’s `CMTangentSpace`; basis algorithms are shared with CM-Opt.
- The new CMC-Opt contribution is the `active_indices_` set plus the `TangentCone` built from linearized active inequalities.
- Thesis references: active/inactive inequality sets are Eqs. (4.3)-(4.4), and the tangent cone parameterized by the equality basis is Eqs. (4.14)-(4.16).

## 4) Active Set Formula

The active set is the set of tight inequalities,

$$
\mathcal{A}(x) = \{k \mid g_k(x) = 0 \text{ or is treated as active at } x\}.
$$

In code, active inequalities are either:

- detected from the current values, or
- explicitly supplied when a retraction already knows which indices must stay active.

Pointers:

- [IEConstraintManifold::identifyActive](IEConstraintManifold.cpp)
- [CMPoint constructor](CMPoint.h#L55)
- [CMPoint::createWithNewValues](CMPoint.h#L87)

That is the mechanism by which a trial step can enter a new boundary mode, keep an old mode, or rebuild the mode after retraction.

Thesis reference: Eq. (4.3) defines the active set, while Eq. (4.4) defines the inactive set.

## 5) Tangent Cone Formula

Given the equality tangent space and the active inequalities, CMC-Opt works in the cone

$$
\mathcal{C}_x = \{\xi \mid Dg_{\mathcal{A}}(x) B_x \xi \ge 0\},
$$

where $B_x$ is the equality tangent basis and $Dg_{\mathcal{A}}(x)$ is the Jacobian of the active inequalities.

Pointers:

- [IEConstraintManifold::linearize](IEConstraintManifold.cpp)

The manifold constructs one [immutable local linearization](CMTangentSpace.h) per state. Cone projection and global LM QP assembly share its active inequality derivatives.

Thesis reference: Eqs. (4.14)-(4.16) define the tangent cone as `v = B theta` with `K theta >= 0`, where `K = Dg_A(x) B`.

## 6) Tangent Cone Projection Formula

CMC-Opt projects a trial direction into the tangent cone by solving

$$
\xi^* = \arg\min_{\xi'} \|\xi' - \xi\|^2
\quad \text{s.t.} \quad A\xi' \ge 0.
$$

Pointers:

- [TangentCone::project](TangentCone.cpp#L23)
- [CMPoint::projectTangentCone](CMPoint.cpp#L79)
- [IEOptimizer::projectTangentCone](IEOptimizer.cpp#L185)

`TangentCone::project` solves the convex IQP. `CMTangentSpace::project` maps the cone-local blocking set back to original inequality indices for `CMPoint::projectTangentCone`. `IEOptimizer::projectTangentCone` applies that componentwise across the whole problem.

Thesis reference: Eq. (4.31) projects a descent direction into the cone for one CMC. Eq. (4.42) applies the same projection componentwise across multiple CMCs. The code generalizes this projection to any trial vector, not only a gradient vector.

## 7) Blocking Constraints

If a direction tries to move outside the cone along an already active boundary, CMC-Opt records the corresponding blocking indices.

Mathematically, these are the active inequalities whose directional derivative would violate feasibility.

Pointers:

- [CMPoint::blockingIndices](CMPoint.cpp#L121)
- [IELMTrial::LinearUpdate](IELMOptimizerState.cpp)

For LM, blocking sets come from the global QP solution, not a preliminary
gradient projection. Only blocks whose every row is active are passed to
retraction as equalities; partially active blocks retain inequality semantics.

Thesis reference: Eq. (4.45) defines the blocking set as active inequalities whose linearized face is tight for the computed update.

## 8) LM Linear Step

`IELMState::construct` assembles one undamped `gtsam::QpProblem` in the existing
component equality coordinates. For each LM trial, `LinearUpdate` copies it,
adds the usual damping factors, and calls GTSAM's sparse `ActiveSetSolver` or
the optional ProxQP adapter:

$$
\min_\xi \frac12\|JB\xi+r\|^2 + \frac\lambda2\|D\xi\|^2,
\qquad Dc_{\mathcal A}(x)B\xi\le0.
$$

Equalities are eliminated by $B$. The QP contains only the currently selected
boundary blocks, with homogeneous right-hand sides. Inactive inequalities are
handled by nonlinear retraction, not affine bounds in the QP. Cross-component
and temporal objective factors remain coupled and sparse.

Costs with matching variable scopes are stacked as Jacobian factors. GTSAM's
`QpCost` must retain that square-root representation, and its sparse solve must
preserve it through QR elimination. Converting these rank-deficient
least-squares factors to Hessians and back can lose weak directions before
active-set handling begins. See [SparseQpValidation.md](SparseQpValidation.md)
for the required dependency fixes and regression results.

Zero is a feasible QP starting point. `qpMaxIterations` defaults to 1,000;
exhaustion or a numerical failure rejects the LM trial. GTSAM can return either
a full QP optimum or an explicitly requested feasible descent step through
`qpUseFeasibleStep`. ProxQP supports full solves only, with projected-gradient
initialization disabled. There is no backend fallback. Extra GTSAM QP
regularization is disabled because damping is already represented in the costs.
The predicted reduction is measured using the undamped objective. `qpIterations`
reports GTSAM working-set iterations or ProxQP total inner iterations; these
count different operations and neither is a count of numerical factorizations.

The numerical backend owns the working-set solve. The former active-face basis $N$,
local face substitution, boundary search, and multiplier loop have been removed
from GTDynamics. The result is lifted through $B$ and sent to the existing
component retractors. A contiguous row mapping recovers original constraint
indices; partially active vector blocks are not promoted wholesale.

Component-local cone projection remains available to GD and other callers.
Changing from the old default feasible approximation to a converged QP can
change steps and nonlinear trajectories even though the tangent-cone model is
unchanged. See [ComponentCoordinates.md](ComponentCoordinates.md).

To enable ProxQP, install ProxSuite and configure `GTDYNAMICS_WITH_PROXQP=ON`,
then select `IELMParams::QpBackend::ProxQP` in `qpBackend`. See the
[installation instructions](../../README.md#optional-sparse-proxqp-backend).

## 9) Nonlinear Update and Retraction

After the linear step produces a tangent update and blocking set, CMC-Opt retracts back to a feasible manifold-with-boundary point.

Conceptually, the retraction solves a feasibility-restoring projection problem with two requirements:

- stay close to the trial point,
- satisfy equality constraints and all inequalities,
- force the blocking inequalities to remain active.

Pointers:

- [IELMTrial::NonlinearUpdate](IELMOptimizerState.cpp#L702)
- [IERetractor](IERetractor.h)
- [BarrierRetractor](BarrierRetractor.h)
- [RunPenaltyOptimization](BarrierRetractor.cpp)
- [BarrierRetractor::retract](BarrierRetractor.cpp)

The concrete barrier retractor does three things:

1. builds priors around the trial point,
2. promotes blocking inequalities to equalities,
3. runs a constrained penalty solve and then a cleanup LM solve.

The penalty solution can have a positive inequality residual smaller than the
feasibility tolerance. It is still on the outward side and must contribute an
active face to cleanup and the next tangent QP. Active-set collection therefore
uses `constraint->active(values)` rather than `!constraint->feasible(values)`.
The latter can return a boundary point with an empty active set, causing later
LM steps to omit that boundary from the tangent QP. A scalar unilateral-contact
regression checks both the returned boundary point and its tangent cone.

Thesis reference: Eq. (4.26) is the standard CMC metric-projection retraction. Eq. (4.46) is the on-corner retraction that additionally enforces blocking inequalities as equalities. Eq. (4.27) is the cleanup feasibility objective used after the penalty projection.

### Affine equality manifolds: reduced AL retraction

[`ReducedALRetractor`](ReducedALRetractor.h) applies when the equality
constraints are **exactly affine in Euclidean variables**. This is a documented
precondition. It uses the original scalar
inequalities directly, including nonlinear constraints involving multiple keys.
It does not require an `InequalityModel`, construct a QCQP, or approximate
nonlinear pose or trajectory equalities by a globally affine manifold.

The component owns the recovery basis and projection geometry. For a known
constant affine equality graph, set `IEConstraintManifold::Params::affineEqualities`
to `true`: successor states share the numeric equality basis, avoiding repeated
linearization and elimination. The default remains `false`, so nonlinear
manifolds rebuild their numeric basis at each state. This option is a caller
contract about fixed affine equations in Euclidean variables; it is not inferred
from one Jacobian and does not freeze active inequality faces.

`CMPoint::projectionMetric()` caches a `TangentSpaceMetric` containing
recovery maps and Cholesky whitening of `B' W B`. Affine successor states share
this immutable geometry. Changes to effective prior weights or selected keys
invalidate the cache; independent components do not share it. The metric follows
the existing prior sigma conventions, including optional varying sigmas and
partial priors. Only the target and common trial-length normalization change
per retraction. Equality factors must remain unchanged during an affine solve.

At an equality-feasible anchor `x0`, recovery is `x = x0 + B z`. Before AL, the
retractor checks whether the ambient trial already satisfies the original
constraints **and every requested blocking face**. If so, its projection cost
is zero and it is returned immediately. Otherwise it computes the unconstrained
metric projection onto the affine equalities; if that point passes the same
checks, it is the constrained minimizer and AL is also unnecessary. These are
optimality shortcuts, not acceptance of an arbitrary feasible AL iterate.
`IERetractionInfo` counts both shortcuts, AL calls, and inner LM iterations.

Each inequality evaluates its original expression at the recovered variables;
its reduced Jacobian is the sum of its original key Jacobians times the
corresponding recovery blocks. Only keys used by that constraint are recovered.
Original noise sigmas are applied once. A fixed positive scale, the maximum
reduced gradient norm at the anchor and target (floored at `1e-12`), normalizes
each row. Blocked inequalities become equalities using the same expression.
GTSAM's `AugmentedLagrangianOptimizer` solves this reduced nonlinear problem;
CMC's outer tangent-cone QP continues to use its existing solver.

For elliptic friction, AL now receives the original norm-valued cone expression.
There is no squared-cone conversion. Its nonsmooth apex and the original
factor's Jacobian convention remain; this is not a general apex solution.
For nonconvex problems, including forced curved boundary faces, stationarity
does not establish a global closest point.

The AL driver uses GTSAM's public BCL iteration and floors its shrinking inner
stationarity target at the requested final tolerance. Defaults retain `1e-7`
normalized stationarity, `1e-8` original whitened feasibility, and `1e-7`
active-face tolerance. The normalized feasibility target accounts for row
scaling; original equality and inequality residuals are checked after recovery.
The retractor throws if convergence or validation fails. It does not accept
feasibility alone or fall back to an ambient penalty solve. Settings live in
`ReducedALRetractor::Settings`; shared retractor parameters supply the metric
and initialization choice.

Regression tests are in [`testReducedALRetractor.cpp`](../../tests/testReducedALRetractor.cpp)
and [`testReducedALRetractionValidation.cpp`](../../tests/testReducedALRetractionValidation.cpp).
They cover projection signs, affine reduction, weighted metrics, original
quadratic and norm-cone factors, nonquadratic multi-key constraints, whitening,
forced boundaries, nonfinite evaluations, and explicit apex failures.

## 10) Boundary Forcing and Mode Changes

CMC-Opt does not only react to the current active set. It can also force a boundary switch when repeated failed trials show that a new boundary is being approached strongly enough.

Mathematically, this is a discrete mode change between different active sets.

Pointers:

- [IEOptimizer::isSameMode](IEOptimizer.cpp#L245)
- [IEOptimizer::identifyChangeIndices](IEOptimizer.h#L115)
- [IEOptimizer::identifyApproachingIndices](IEOptimizer.cpp#L284)
- [IELMOptimizer::checkModeChange](IELMOptimizer.cpp#L133)
- [CMPoint::moveToBoundary](CMPoint.cpp#L146)
- [movePointsToBoundaries](CMPoints.cpp)
- [BarrierRetractor::moveToBoundary](BarrierRetractor.cpp)

This is where CMC-Opt moves from a purely smooth local method to a boundary-mode method.

Thesis reference: Eq. (4.45) identifies blocking inequalities, and Eq. (4.46) describes the on-corner retraction onto the corresponding boundary face.

## 11) Full LM Iteration Loop

The full LM loop is:

1. linearize the cost and active inequalities,
2. compute a cone-feasible damped step,
3. retract with blocking constraints,
4. accept or reject by model fidelity,
5. optionally force a mode change if the boundary is being approached.

Pointers:

- [IELMOptimizer::optimize](IELMOptimizer.cpp#L31)
- [IELMOptimizer::iterate](IELMOptimizer.cpp#L73)
- [IELMOptimizer::checkModeChange](IELMOptimizer.cpp#L133)
- [IELMState::fromLastIteration](IELMOptimizerState.cpp#L36)
- [IELMOptimizer::checkConvergence](IELMOptimizer.cpp#L217)

Thesis reference: this is the implementation counterpart of Algorithm 3, with the linear subproblem from Eq. (4.44), retraction from Eq. (4.46), and model fidelity from Eq. (3.53).

## 12) Gradient-Descent Variant

The GD variant uses the same manifold-with-boundary model, but instead of an LM/QP step it projects the descent direction into the tangent cone and retracts that projected direction.

Conceptually,

$$
d = -\nabla f, \qquad
d_{proj} = \Pi_{\mathcal{C}_x}(d), \qquad
x^+ = R_x(d_{proj}).
$$

Pointers:

- [IEGDState](IEGDOptimizer.h#L46)
- [IEGDTrial](IEGDOptimizer.h#L83)
- [IEGDOptimizer](IEGDOptimizer.h#L124)
- [IEGDState::computeDescentDirection](IEGDOptimizer.cpp#L51)
- [IEGDTrial::computeNewPoints](IEGDOptimizer.cpp)
- [IEGDOptimizer::iterate](IEGDOptimizer.cpp#L175)
- [IEGDOptimizer::checkModeChange](IEGDOptimizer.cpp#L287)

Thesis reference: Algorithm 2 is the CMC gradient-descent variant. Its cone projection is Eq. (4.31)/(4.42), and its update/retraction step is Eq. (4.41).

## 13) Optional Metric-Aware Retraction

Some CMC-Opt runs use varying metric sigmas when building the retraction priors. That is the code path closest to a metric-weighted projection idea.

Pointers:

- [IERetractorParams](IERetractor.h)
- [IERetractorParams::createVaryingSigmas](IERetractor.h)
- [IELMState::computeMetricSigmas](IELMOptimizerState.cpp#L148)

Thesis reference: metric retractions are written generically in Eq. (5.12), with the cost-aware metric idea in Eqs. (5.18), (5.20), and (5.21). For CMCs, the projected gradient under the induced metric is Eq. (5.4).

## 14) Minimal File Roadmap

If you want to trace the math in code order, read these files in this order:

1. [IEOptimizer.h](IEOptimizer.h#L31) and [IEOptimizer.cpp](IEOptimizer.cpp#L85)
2. [CMPoint.h](CMPoint.h#L25) and [CMPoint.cpp](CMPoint.cpp)
3. [TangentCone.h](TangentCone.h#L25) and [TangentCone.cpp](TangentCone.cpp#L23)
4. [IELMOptimizerState.h](IELMOptimizerState.h#L35) and [IELMOptimizerState.cpp](IELMOptimizerState.cpp#L493)
5. [IERetractor.h](IERetractor.h) and [IERetractor.cpp](IERetractor.cpp)
6. [IELMOptimizer.h](IELMOptimizer.h#L40) and [IELMOptimizer.cpp](IELMOptimizer.cpp#L31)
7. [IEGDOptimizer.h](IEGDOptimizer.h#L124) and [IEGDOptimizer.cpp](IEGDOptimizer.cpp#L51)

## 15) Relation to CM-Opt

CMC-Opt reuses the equality-manifold machinery from CM-Opt and adds active sets, tangent cones, and boundary-aware retraction. For the equality-only side, see [../cmopt/README.md](../cmopt/README.md).


## Shared affine projection for AL and Barrier retraction

`CMTangentSpace` owns the equality basis. With
`IEConstraintManifold::Params::affineEqualities = true`, successive states reuse
that numeric basis and its projection-metric cache. This is an explicit contract
for affine equalities in Euclidean variables; nonlinear manifolds continue to
rebuild their bases.

`IERetractorParams::priorPrecisions()` defines the projection distance for both
ambient prior factors and reduced objectives. It handles constant or varying
sigmas, optional nominal scaling, selected keys, and repeated keys. The metric
cache changes when the effective weights, selected keys, or whitening option
change; those changes do not reconstruct the equality basis.

`AffineProjection` supplies shared recovery, original scalar inequality
pullbacks and derivatives, required blocking faces, and original-coordinate
validation. Constraints may involve any original key, including eliminated
variables. Both reduced solvers use it:

- `ReducedALRetractor` retains its AL policy, normalization, and explicit
  stationarity/feasibility convergence checks.
- `BarrierRetractor` uses it when `IERetractorParams::useReducedCoordinates` is
  true **and** the manifold has the affine contract. Otherwise it retains the
  existing ambient implementation. The reduced path uses the existing GTSAM
  penalty solver followed by active-face feasibility cleanup. It validates all
  original constraints and required faces before returning. Its validation
  threshold is `feasibleThreshold`, interpreted as an infinity norm of original
  whitened residuals in this opt-in path; failure throws. Cleanup establishes
  feasibility, not a stationarity certificate for the nearest point.

For a fair penalty comparison, the reduced Barrier path retains original
constraint scaling and projection-cost magnitude. AL's trial-length and row
normalizations are not applied to the penalty problem, where they would change
finite-penalty iterates. Eliminating the equalities does remove their penalty
terms, so ambient and reduced solver trajectories need not coincide.

`whitenAffineCoordinates` defaults to true. Setting it false retains the original
basis coordinates and the quadratic cost metric instead of absorbing that
metric into recovery maps. Both choices represent the same projection objective;
conditioning and finite solver tolerances can still affect numerical results.

Both reduced methods bypass the solver when the trial or its unconstrained
weighted equality projection is already feasible, including any requested
blocking faces. `numALSolveCalls`, `numPenaltySolveCalls`, `numFeasibleTrials`, and
`numFeasibleProjections` distinguish these cases.

The hierarchical CMC retractor and CM-only retractors are not converted to
reduced solves by this change. Nonlinear finite recovery remains future work.
