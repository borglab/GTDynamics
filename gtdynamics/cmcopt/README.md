# CMC-Opt Math to Code Map

This note maps the main mathematical objects behind CMC-Opt to the implementation in `gtdynamics/cmcopt`.

CMC-Opt extends equality-only CM-Opt to manifolds with inequality boundaries and corners. The key extra ingredients are:

- an active set of tight inequalities,
- a tangent cone instead of only an equality tangent space,
- a projection step that respects that cone, and
- a retraction that can promote blocking inequalities to temporary equalities.

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

In code, the top-level optimizer API takes exactly those three pieces:

- cost graph,
- equality constraints,
- inequality constraints.

Pointers:

- [IEOptimizer](IEOptimizer.h#L31)
- [IEOptimizer::optimize](IEOptimizer.h#L43)
- [IEOptimizer::IdentifyUnconstrainedValues](IEOptimizer.h#L78)

The public `optimize(...)` entry point builds inequality/equality manifolds from the constraints, keeps unconstrained variables separate, and then hands the transformed problem to either the LM or GD variant.

## 2) Constraint-Connected Components with Inequalities

As in CM-Opt, the code first groups variables into constraint-connected components. For CMC-Opt, those components are built from both equality and inequality constraints.

Mathematically, each component carries a local feasible set

$$
\mathcal{M}_{IE} = \{x \mid h(x)=0,\; g(x)\ge 0\}.
$$

This is the local component version of the CMC state space in thesis Eq. (4.2), and matches the constraint-connected component construction in Eqs. (4.33)-(4.34).

Pointers:

- [IdentifyConnectedComponents](IEOptimizer.cpp#L87)
- [IEOptimizer::IdentifyManifolds](IEOptimizer.cpp#L127)
- [IEConstraintManifold](IEConstraintManifold.h#L25)

`IEOptimizer::IdentifyManifolds` builds one [IEConstraintManifold](IEConstraintManifold.h#L25) per connected component. That object is the central mathematical container for the local manifold-with-boundary.

## 3) IEConstraintManifold: Equality Basis + Active Set + Cone

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

- [IEConstraintManifold](IEConstraintManifold.h#L25)
- [IEConstraintManifold::activeIndices](IEConstraintManifold.h#L98)
- [IEConstraintManifold::IdentifyActiveConstraints](IEConstraintManifold.cpp#L181)
- [IEConstraintManifold::ConstructTangentCone](IEConstraintManifold.cpp#L205)
- [TangentCone](TangentCone.h#L25)

Implementation detail:

- The equality tangent basis is still inherited from the CM-Opt machinery via `e_basis_`.
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

- [IEConstraintManifold::IdentifyActiveConstraints](IEConstraintManifold.cpp#L181)
- [IEConstraintManifold constructor](IEConstraintManifold.h#L55)
- [IEConstraintManifold::createWithNewValues](IEConstraintManifold.h#L87)

That is the mechanism by which a trial step can enter a new boundary mode, keep an old mode, or rebuild the mode after retraction.

Thesis reference: Eq. (4.3) defines the active set, while Eq. (4.4) defines the inactive set.

## 5) Tangent Cone Formula

Given the equality tangent space and the active inequalities, CMC-Opt works in the cone

$$
\mathcal{C}_x = \{\xi \mid Dg_{\mathcal{A}}(x) B_x \xi \ge 0\},
$$

where $B_x$ is the equality tangent basis and $Dg_{\mathcal{A}}(x)$ is the Jacobian of the active inequalities.

Pointers:

- [IEConstraintManifold::ConstructTangentCone](IEConstraintManifold.cpp#L205)
- [IEConstraintManifold::linearActiveManIConstraints](IEConstraintManifold.cpp#L235)
- [IEConstraintManifold::linearActiveBaseIConstraints](IEConstraintManifold.cpp#L263)

This is exactly where the code turns active nonlinear inequalities into linearized cone constraints in either manifold coordinates or base-variable coordinates.

Thesis reference: Eqs. (4.14)-(4.16) define the tangent cone as `v = B theta` with `K theta >= 0`, where `K = Dg_A(x) B`.

## 6) Tangent Cone Projection Formula

CMC-Opt projects a trial direction into the tangent cone by solving

$$
\xi^* = \arg\min_{\xi'} \|\xi' - \xi\|^2
\quad \text{s.t.} \quad A\xi' \ge 0.
$$

Pointers:

- [TangentCone::project](TangentCone.cpp#L23)
- [IEConstraintManifold::projectTangentCone](IEConstraintManifold.cpp#L79)
- [IEOptimizer::ProjectTangentCone](IEOptimizer.cpp#L185)

`TangentCone::project` solves the convex IQP. `IEConstraintManifold::projectTangentCone` maps the cone-local blocking set back to original inequality indices. `IEOptimizer::ProjectTangentCone` applies that componentwise across the whole problem.

Thesis reference: Eq. (4.31) projects a descent direction into the cone for one CMC. Eq. (4.42) applies the same projection componentwise across multiple CMCs. The code generalizes this projection to any trial vector, not only a gradient vector.

## 7) Blocking Constraints

If a direction tries to move outside the cone along an already active boundary, CMC-Opt records the corresponding blocking indices.

Mathematically, these are the active inequalities whose directional derivative would violate feasibility.

Pointers:

- [IEConstraintManifold::blockingIndices](IEConstraintManifold.cpp#L121)
- [IELMState::grad_blocking_indices_map](IELMOptimizerState.h#L46)
- [IELMState::computeGradient](IELMOptimizerState.cpp#L116)

Those blocking sets are later passed into the retraction, where they are enforced as equalities.

Thesis reference: Eq. (4.45) defines the blocking set as active inequalities whose linearized face is tight for the computed update.

## 8) LM Linear Step

The LM variant solves a damped local quadratic model with inequality-aware linear constraints. Conceptually it is

$$
\min_{\delta}
\frac{1}{2}\|J\delta + r\|^2 + \frac{\lambda}{2}\|D\delta\|^2
\quad \text{s.t.} \quad
\delta \in \mathcal{C}_x.
$$

Pointers:

- [IELMOptimizer](IELMOptimizer.h#L40)
- [IELMState](IELMOptimizerState.h#L35)
- [IELMState::linearizeIConstraints](IELMOptimizerState.cpp#L100)
- [IELMTrial::LinearUpdate](IELMOptimizerState.cpp#L493)
- [IELMTrial::LinearUpdate::InitEstimate](IELMOptimizerState.cpp#L555)
- [IELMTrial::LinearUpdate::CheckSolutionValid](IELMOptimizerState.cpp#L600)

How the code matches the math:

- `linearizeIConstraints()` builds the linearized inequality constraints at the current state.
- `LinearUpdate(...)` builds the damped LM system.
- `InitEstimate(...)` seeds the IQP with blocking constraints coming from the negative gradient.
- `SolveConvexIQP(...)` is then used to refine the constrained step.

Thesis reference: Eq. (4.44) is the CMC Levenberg-Marquardt quadratic subproblem, constrained by the cone coordinates for each CMC.

## 9) Nonlinear Update and Retraction

After the linear step produces a tangent update and blocking set, CMC-Opt retracts back to a feasible manifold-with-boundary point.

Conceptually, the retraction solves a feasibility-restoring projection problem with two requirements:

- stay close to the trial point,
- satisfy equality constraints and all inequalities,
- force the blocking inequalities to remain active.

Pointers:

- [IELMTrial::NonlinearUpdate](IELMOptimizerState.cpp#L702)
- [IERetractor](IERetractor.h#L92)
- [BarrierRetractor](IERetractor.h#L125)
- [RunPenaltyOptimization](IERetractor.cpp#L51)
- [BarrierRetractor::retract](IERetractor.cpp#L116)

The concrete barrier retractor does three things:

1. builds priors around the trial point,
2. promotes blocking inequalities to equalities,
3. runs a constrained penalty solve and then a cleanup LM solve.

Thesis reference: Eq. (4.26) is the standard CMC metric-projection retraction. Eq. (4.46) is the on-corner retraction that additionally enforces blocking inequalities as equalities. Eq. (4.27) is the cleanup feasibility objective used after the penalty projection.

## 10) Boundary Forcing and Mode Changes

CMC-Opt does not only react to the current active set. It can also force a boundary switch when repeated failed trials show that a new boundary is being approached strongly enough.

Mathematically, this is a discrete mode change between different active sets.

Pointers:

- [IEOptimizer::IsSameMode](IEOptimizer.cpp#L245)
- [IEOptimizer::IdentifyChangeIndices](IEOptimizer.h#L115)
- [IEOptimizer::IdentifyApproachingIndices](IEOptimizer.cpp#L284)
- [IELMOptimizer::checkModeChange](IELMOptimizer.cpp#L133)
- [IEConstraintManifold::moveToBoundary](IEConstraintManifold.cpp#L146)
- [IEManifoldValues::moveToBoundaries](IEConstraintManifoldUtils.cpp#L25)
- [BarrierRetractor::moveToBoundary](IERetractor.cpp#L78)

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

- [IELMOptimizer::optimizeManifolds](IELMOptimizer.cpp#L31)
- [IELMOptimizer::iterate](IELMOptimizer.cpp#L73)
- [IELMOptimizer::checkModeChange](IELMOptimizer.cpp#L133)
- [IELMState::FromLastIteration](IELMOptimizerState.cpp#L36)
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
- [IEGDTrial::computeNewManifolds](IEGDOptimizer.cpp#L93)
- [IEGDOptimizer::iterate](IEGDOptimizer.cpp#L175)
- [IEGDOptimizer::checkModeChange](IEGDOptimizer.cpp#L287)

Thesis reference: Algorithm 2 is the CMC gradient-descent variant. Its cone projection is Eq. (4.31)/(4.42), and its update/retraction step is Eq. (4.41).

## 13) Optional Metric-Aware Retraction

Some CMC-Opt runs use varying metric sigmas when building the retraction priors. That is the code path closest to a metric-weighted projection idea.

Pointers:

- [IERetractorParams](IERetractor.h#L31)
- [IERetractorParams::VarySigmas](IERetractor.h#L63)
- [IELMState::computeMetricSigmas](IELMOptimizerState.cpp#L148)

Thesis reference: metric retractions are written generically in Eq. (5.12), with the cost-aware metric idea in Eqs. (5.18), (5.20), and (5.21). For CMCs, the projected gradient under the induced metric is Eq. (5.4).

## 14) Minimal File Roadmap

If you want to trace the math in code order, read these files in this order:

1. [IEOptimizer.h](IEOptimizer.h#L31) and [IEOptimizer.cpp](IEOptimizer.cpp#L85)
2. [IEConstraintManifold.h](IEConstraintManifold.h#L25) and [IEConstraintManifold.cpp](IEConstraintManifold.cpp#L181)
3. [TangentCone.h](TangentCone.h#L25) and [TangentCone.cpp](TangentCone.cpp#L23)
4. [IELMOptimizerState.h](IELMOptimizerState.h#L35) and [IELMOptimizerState.cpp](IELMOptimizerState.cpp#L493)
5. [IERetractor.h](IERetractor.h#L90) and [IERetractor.cpp](IERetractor.cpp#L114)
6. [IELMOptimizer.h](IELMOptimizer.h#L40) and [IELMOptimizer.cpp](IELMOptimizer.cpp#L31)
7. [IEGDOptimizer.h](IEGDOptimizer.h#L124) and [IEGDOptimizer.cpp](IEGDOptimizer.cpp#L51)

## 15) Relation to CM-Opt

CMC-Opt reuses the equality-manifold machinery from CM-Opt and adds active sets, tangent cones, and boundary-aware retraction. For the equality-only side, see [../cmopt/README.md](../cmopt/README.md).
