# Constraint Manifold Optimization

<a id="overview"></a>
## Overview
This note maps CM-Opt as presented in Zhang et al., *Constraint Manifolds for
Robotic Inference and Planning* (ICRA 2023), to the GTDynamics implementation
in `gtdynamics/cmopt`. Equality-only and corner-aware optimization now share
runtime definitions, point values, discovery, recovery, and transformed factors.
`EConstraintManifold` is a runtime type and `ECMPoint` is its numerical value;
the former `ConstraintManifold` value implementation has been removed.
See the [type and migration guide](../constrained/README.md#migration-and-solver-behavior)
for the common hierarchy and API changes.

Equation references use the ICRA paper as the primary source. The June 2024
thesis, *Constraint Manifold Optimization for Robotic Inference and Planning
with Constraints*, expands the same construction in Chapter 3 and adds useful
implementation context in Chapter 5; thesis references are included only where
they clarify the code.

The paper starts from the equality-constrained factor-graph problem

\[
\arg\min_{X \in \mathcal{M}} \sum_i f_i(X_i^f)
\quad \text{s.t.} \quad h_j(X_j^h)=0,
\]

which is paper Eq. (11). CM-Opt transforms it into an unconstrained manifold
optimization problem

\[
\arg\min_{\Theta \in \mathcal{M}} \sum_i \bar f_i(\Theta_i^f),
\]

which is paper Eq. (16). Each constraint-connected component (CCC) becomes one
constraint-manifold variable, and cost factors touching constrained variables
are replaced by equivalent factors on those manifold variables.

In code, this transformation happens in three stages:

- Shared [component discovery](../cmcopt/IEOptimizer.cpp) groups the equality graph.
- [ManifoldOptimizer](ManifoldOptimizer.cpp) constructs immutable
  `EConstraintManifold` definitions and concrete `ECMPoint` values.
- [CMCostFactor](../constrained/CMCostFactor.h) transforms costs using
  the equality point category and the same recovery/Jacobian implementation as CMC.

Useful cross-reference:

| Concept | Paper | Thesis |
| --- | --- | --- |
| Original equality NLP | Eq. (11) | Eq. (3.1) |
| CCC, feasible component, recovery | Eqs. (12)-(14) | Eqs. (3.47)-(3.49), plus Eqs. (3.33)-(3.45) |
| Equivalent factors and transformed problem | Eqs. (15)-(16) | Eq. (3.46), Eq. (3.49) |
| Tangent basis | Eqs. (17)-(20) | Eqs. (3.5)-(3.14), Eqs. (3.41)-(3.44) |
| Retraction schemes | Eqs. (21)-(24) | Eqs. (3.15)-(3.17), Eq. (3.21), Eq. (3.45) |
| Equivalent-factor Jacobian | Eq. (25) | Eqs. (3.29)-(3.32) |
| Approximate retraction / approximate CM | Eq. (26) | Eqs. (5.22)-(5.27) |
| LM loop details | discussed around Eq. (16) | Eqs. (3.50)-(3.54), Algorithm 1 |

For transformed-factor construction, start with
[`ManifoldOptimizer::manifoldGraph`](ManifoldOptimizer.cpp) and
[`CMCostFactor`](../constrained/CMCostFactor.cpp).

<a id="ccc-transformation"></a>
## 1) CCC Identification and Problem Transformation (Paper Sec. III-A)
The paper defines a constraint-connected component as variables tied together by
equality constraints:

\[
C_c = (X_c^C, H_c(X_c^C)=0).
\]

For each CCC, CM-Opt constructs:
- the product embedding manifold \(\tilde{\mathcal{M}}_c\), paper Eq. (12),
- the constraint manifold \(\mathcal{M}_c\), paper Eq. (13),
- a recovery function \(r_k : \mathcal{M}_c \to \mathcal{M}_k\), paper Eq. (14),
- equivalent factors \(\bar f_i(\Theta_i^f)\), paper Eq. (15),
- the transformed unconstrained problem, paper Eq. (16).

Code mapping:

- Shared constraint-variable graph traversal:
  [`IEOptimizer::identifyConnectedComponents`](../cmcopt/IEOptimizer.cpp).
- Equality-only definition creation and requested initial projection:
  [`ManifoldOptimizer::constructManifoldOptimizationValues`](ManifoldOptimizer.cpp).
- Ambient-key ownership and transformed costs:
  [`ManifoldOptimizer::manifoldGraph`](ManifoldOptimizer.cpp).
- Shared recovery and chain rule:
  [`CMCostFactor`](../constrained/CMCostFactor.h).

Implementation detail:
- A CCC whose manifold dimension is positive becomes a free manifold-valued
  variable in `mopt_problem.values`.
- A fully constrained CCC becomes a fixed `ECMPoint` in
  `mopt_problem.fixedPoints`; equivalent factors can still recover values
  from it, but it is not optimized as a free variable.
- Variables untouched by equality constraints are copied as ordinary
  unconstrained variables.

The thesis makes this same transformation more explicit in Eqs. (3.46)-(3.49).

<a id="constraint-manifold"></a>
## 2) Constraint Manifold Object and Dimension (Paper Eqs. 12-13)
For one CCC, the paper first builds the product manifold
\(\tilde{\mathcal{M}}_c = \prod_{k \in I_c^C}\mathcal{M}_k\), then defines
the feasible subset

\[
\mathcal{M}_c =
\{X_c^C \in \tilde{\mathcal{M}}_c : H_c(X_c^C)=0\}.
\]

That is paper Eq. (13). The preliminaries give the submanifold assumption:
full-rank constraint differential, dimension reduction, and tangent space as a
kernel in paper Eqs. (8)-(10).

Code mapping:

- Runtime definition: [`EConstraintManifold`](EConstraintManifold.h).
- Numerical state and bindings: [`ECMPoint`](ECMPoint.h), with common
  recovery and dimension operations on [`CMPoint`](../constrained/CMPoint.h).
- Equality basis and point-dependent geometry:
  [`CMTangentSpace`](../constrained/CMTangentSpace.h).
- Initial projection before the first numerical basis:
  [`ManifoldOptimizer`](ManifoldOptimizer.cpp), through shared discovery's
  value-initialization hook.

The runtime type owns constraint factors, immutable conditioning, structural
preparation, and retraction policy. Points own ambient values and bindings.
Numerical bases are built per nonlinear point; explicitly certified affine
geometry is initialized once per runtime type and published read-only.
Conditioning constructs a distinct type with fresh preparation. Equality
geometry is independent of objective weights, damping, and retraction metrics.

Thesis context: Eq. (3.4) gives the simple dimension formula \(n=N-n_h\) for
Euclidean embedding spaces, while Eqs. (3.33)-(3.45) expand the product-manifold
version used by GTSAM values such as poses and rotations.

<a id="tangent-basis"></a>
## 3) Tangent Space Basis (Paper Sec. IV-A, Eqs. 17-20)
The paper computes the tangent space of the product manifold first:

\[
T_{X_c^C}\tilde{\mathcal{M}}_c =
\prod_{k \in I_c^C} T_{x_k}\mathcal{M}_k,
\]

paper Eq. (17), with product-basis construction in paper Eq. (18). The
constraint-manifold tangent space is then the kernel of the constraint
differential, paper Eq. (19), and its basis is

\[
B_{\theta_c}\mathcal{M}_c = B_{X_c^C}\tilde{\mathcal{M}}_c \, N,
\]

paper Eq. (20), where \(N\) is a null-space basis of the constraint Jacobian.

Code mapping:
- Abstract basis API: [`TangentSpaceBasis`](../constrained/TangentSpaceBasis.h)
- Orthonormal/null-space basis: [`OrthonormalBasis`](../constrained/TangentSpaceBasis.h)
- Constraint Jacobian: [`OrthonormalBasis::computeConstraintJacobian`](../constrained/TangentSpaceBasis.cpp)
- Dense null-space basis: [`OrthonormalBasis::constructDense`](../constrained/TangentSpaceBasis.cpp)
- Sparse null-space basis: [`OrthonormalBasis::constructSparse`](../constrained/TangentSpaceBasis.cpp)
- Map reduced coordinates \(\xi\) to ambient `VectorValues`: [`OrthonormalBasis::computeTangentVector`](../constrained/TangentSpaceBasis.cpp)

Parameterized/basis-variable path:
- Eliminate non-basis variables: [`EliminationBasis::construct`](../constrained/TangentSpaceBasis.cpp)
- Propagate Bayes-net Jacobians: [`computeBayesNetJacobian`](../constrained/MultiJacobian.cpp)
- Lift reduced coordinates to full tangent values: [`EliminationBasis::computeTangentVector`](../constrained/TangentSpaceBasis.cpp)

The thesis is useful here because it separates the two basis choices. Eqs.
(3.7)-(3.11) describe the parameterized/elimination basis; Eqs. (3.12)-(3.14)
describe the orthonormal basis. The code supports both through
`TangentSpaceBasisCreator`.

<a id="retraction"></a>
## 4) Retraction on Constraint Manifolds (Paper Sec. IV-B, Eqs. 21-24)
Paper Eq. (21) first applies each original variable's own retraction on the
product manifold. The CM-Opt retractor then projects or matches the result back
onto the CCC's constraint manifold.

The paper gives three practical retraction schemes:

| Scheme | Paper equation | Code |
| --- | --- | --- |
| Metric projection | Eq. (22) | [`ProjectionRetractor::retract`](ProjectionRetractor.cpp) |
| Approximate metric projection | Eq. (23) | [`UnconstrainedOptimizationRetractor::retractConstraints`](UnconstrainedOptimizationRetractor.cpp) |
| Retract basis variables | Eq. (24) | [`BasisRetractor::retractConstraints`](BasisRetractor.cpp) |

Shared flow:
- Reduced tangent coordinates \(\xi\) become an ambient tangent update through
  the basis: [`ECMPoint::retract`](ECMPoint.cpp)
- Base variables are retracted first: [`Retractor::retract`](Retractor.h)
- The retractor then enforces the equality constraints: [`Retractor::retractConstraints`](Retractor.h)

How the implementations differ:
- `UnconstrainedOptimizationRetractor` runs unconstrained LM on the penalty graph, matching the
  approximate projection idea in paper Eq. (23).
- `ProjectionRetractor` adds priors around the product-manifold trial point, then can
  run a cleanup solve without priors. This is the code path closest to the
  metric-projection objective in paper Eq. (22).
- `BasisRetractor` fixes the chosen basis variables at their retracted values
  and solves for the remaining variables, matching paper Eq. (24).
- `DynamicsRetractor` specializes the same idea for kinodynamic components by
  solving position, velocity, and acceleration/dynamics groups in sequence.

Thesis context:
- Eq. (3.21) explains the cleanup least-squares feasibility solve used after
  numerical projection.
- Eq. (5.12) writes metric projection with a general metric, and Eq. (5.21)
  gives the cost-aware variant used conceptually by later CM-Opt+ work.
- Eqs. (5.25)-(5.27) formalize approximate retractions that stop before full
  convergence.

<a id="equivalent-factors"></a>
## 5) Equivalent Cost Factors and Chain-Rule Jacobians (Paper Eqs. 15 and 25)
Paper Eq. (15) defines the equivalent factor by substituting every constrained
base variable with its recovery function:

\[
\bar f_i(\Theta_i^f)=f_i(X_i^f), \qquad
X_i^f = \{r_k(\theta_c)\}_{k \in I_i^f}.
\]

Paper Eq. (25) gives the Jacobian of an equivalent factor by chaining the
recover Jacobian with the original factor Jacobian:

\[
J_f(\theta_c)=
\sum_{k \in I_c^C \cap I_i^f} J_{r_k}(\theta_c)\,J_f(x_k).
\]

The shared [`CMCostFactor`](../constrained/CMCostFactor.cpp) compiles ordered
connections from original cost argument indices to point-local slots. The point
owns immutable slot bindings; the original cost retains its argument keys and
embedded expressions. Evaluation recovers only connected slots through
`CMPoint::recoverSlot` and sums the original Jacobian blocks times their current
recovery Jacobians. No per-factor ambient-key lookup map or point-wide scan is
needed. Zero-dimensional point arguments are captured as fixed ambient values;
ordinary free arguments pass through directly. Equality-only factors select
`PointType::Equality`, matching
`ECMPoint` values in `gtsam::Values`; general CMC factors select the IE category.
GTSAM extraction must use the exact stored concrete type despite public point
inheritance. Original noise models remain intact, with whitening applied once.
`pointCostGraph` builds connections from all points, including fixed points,
and rejects incompatible slot bindings during evaluation.

The thesis derives this chain rule through recovery functions in
Eqs. (3.29)-(3.32).

<a id="solvers"></a>
## 6) Solving the Transformed Problem
The paper's optimization target is the transformed unconstrained manifold
problem in Eq. (16). Both equality-only paths now accept `ECMPoints` and
ordinary free values, as well as the existing ambient cost/equality/value entry
point. They preserve their numerical loops:

- Generic path: [`NonlinearManifoldOptimizer`](NonlinearManifoldOptimizer.cpp) transforms
  the equality-constrained problem, then reuses standard GTSAM nonlinear
  optimizers on the manifold-valued variables.
- Custom LM path: [`LMManifoldOptimizer`](LMManifoldOptimizer.cpp) keeps
  explicit state/trial bookkeeping for CM-Opt updates on `ECMPoint` values.

`IELMOptimizer` can also optimize these equality points through explicit
widening to the general CMC point category, using direct Gaussian or QP linear
steps. The two custom LM implementations remain separate. Equality-only GTSAM
wrappers do not accept general inequality points; manifold traits do not provide
corner-aware optimization. Initial/final projection, callbacks, stopping rules,
and CM(F)/CM(I) retraction budgets retain their existing meanings.

Benchmark helpers:
- `ConstrainedOptBenchmark::OptimizeCmOpt` uses `NonlinearManifoldOptimizer`: [`ConstrainedOptBenchmark.cpp`](../constrained_optimizer/ConstrainedOptBenchmark.cpp)
- Dense/null-space plus approximate projection defaults: [`DefaultMoptParams`](../constrained_optimizer/ConstrainedOptBenchmark.cpp)
- Elimination basis plus basis-variable retraction defaults: [`DefaultMoptParamsSV`](../constrained_optimizer/ConstrainedOptBenchmark.cpp)

The thesis gives the LM details that are implicit in the paper: least-squares
model and quadratic approximation in Eqs. (3.50)-(3.51), LM update and fidelity
ratio in Eqs. (3.52)-(3.53), the manifold version in Eq. (3.54), and the full
LM-based CM-Opt loop in Algorithm 1.

<a id="approximate-retraction"></a>
## 7) Approximate Retraction and Symbolic Elimination
Paper Sec. V-B introduces approximate retraction because solving paper
Eqs. (23)-(24) exactly can dominate runtime. The approximate value may be
temporarily infeasible, and the paper models it as lying on an approximate
constraint manifold in Eq. (26).

The existing equality retractor factories implement `IERetractorCreator`
directly. They snapshot configuration into a type, bind ambient keys for each
point, and isolate mutable solver/factor work per call. `ECMPoint` construction
does not add a stricter projection after an approximate retraction.

Approximate behavior appears through retractor settings rather than a
separate mathematical type:
- limiting inner LM iterations in examples,
- using `UnconstrainedOptimizationRetractor` for approximate metric projection,
- using `BasisRetractor` for sparse/basis-variable updates,
- fast-pathing already feasible basis-retraction solves in
  [`BasisRetractor::retractConstraints`](BasisRetractor.cpp).

Paper Sec. V-C explains why the basis-variable version is closely related to
symbolic variable elimination: the chosen basis variables get direct tangent
updates, and the eliminated variables are recovered by satisfying constraints.
That is the conceptual model behind `EliminationBasis` plus `BasisRetractor`.

The thesis expands the approximate-retraction picture in Sec. 5.2, especially
Eqs. (5.22)-(5.27), and separates this from the standard feasible CM-Opt loop.

<a id="walkthrough"></a>
## 8) Quick Follow-One-Run Pointers
For a concrete end-to-end run, `examples/example_constraint_manifold/main_connected_poses.cpp` is a good example:
- Build equality constraints from a constraint graph: [`main_connected_poses.cpp`](../../examples/example_constraint_manifold/main_connected_poses.cpp)
- Build default CM params: [`main_connected_poses.cpp`](../../examples/example_constraint_manifold/main_connected_poses.cpp)
- Run CM optimization: [`main_connected_poses.cpp`](../../examples/example_constraint_manifold/main_connected_poses.cpp)

Read the core implementation in this order:
1. [`ManifoldOptimizer.h`](ManifoldOptimizer.h) and [`ManifoldOptimizer.cpp`](ManifoldOptimizer.cpp)
2. [`EConstraintManifold.h`](EConstraintManifold.h),
   [`ECMPoint.h`](ECMPoint.h), and shared
   [`IEConstraintManifold.cpp`](../cmcopt/IEConstraintManifold.cpp)
3. [`TangentSpaceBasis.h`](../constrained/TangentSpaceBasis.h), [`TangentSpaceBasis.cpp`](../constrained/TangentSpaceBasis.cpp), and [`MultiJacobian.cpp`](../constrained/MultiJacobian.cpp)
4. [`Retractor.h`](Retractor.h) and [`Retractor.cpp`](Retractor.cpp)
5. [`CMCostFactor.h`](../constrained/CMCostFactor.h) and
   [`CMCostFactor.cpp`](../constrained/CMCostFactor.cpp)
6. [`NonlinearManifoldOptimizer.h`](NonlinearManifoldOptimizer.h), [`NonlinearManifoldOptimizer.cpp`](NonlinearManifoldOptimizer.cpp), and optionally [`LMManifoldOptimizer.cpp`](LMManifoldOptimizer.cpp)

For inequality boundaries and corners, see the separate CMC-Opt documentation in
[`../cmcopt/README.md`](../cmcopt/README.md).

The retraction lifecycle is shared through
[`CMRetractor` and `CMRetractorCreator`](../constrained/CMRetractor.h).
The equality `Retractor`/`RetractorCreator` classes retain their IE-compatible
point wrappers and equality kernels; metric binding and workspace isolation live
in `constrained`. See the
[shared retraction contracts](../constrained/README.md#shared-retraction-policies-and-workspaces).
