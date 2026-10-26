# Constraint Manifold Optimization

<a id="overview"></a>
## Overview
This note maps CM-Opt as presented in Zhang et al., *Constraint Manifolds for
Robotic Inference and Planning* (ICRA 2023), to the GTDynamics implementation
in `gtdynamics/cmopt`.

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
- Find CCCs from equality constraints: [`ManifoldOptimizer::identifyConnectedComponents`](ManifoldOptimizer.cpp#L120)
- Build one `ConstraintManifold` per CCC: [`ManifoldOptimizer::constructManifoldValues`](ManifoldOptimizer.cpp#L184)
- Build equivalent cost factors on manifold variables: [`ManifoldOptimizer::constructManifoldOptimizationGraph`](ManifoldOptimizer.cpp#L232), using [`SubstituteFactor`](../factors/SubstituteFactor.h#L39)

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

If you are looking for where the transformed equivalent factors are created,
start at:
- [`ManifoldOptimizer.cpp#L251`](ManifoldOptimizer.cpp#L251)
- [`ManifoldOptimizer.cpp#L262`](ManifoldOptimizer.cpp#L262)
- [`SubstituteFactor.cpp#L82`](../factors/SubstituteFactor.cpp#L82)

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
- DFS over the constraint-variable bipartite graph: [`identifyConnectedComponent`](ManifoldOptimizer.cpp#L88)
- All components: [`identifyConnectedComponents`](ManifoldOptimizer.cpp#L120)
- Full transformation pipeline: [`transformProblem`](ManifoldOptimizer.cpp#L165)
- Map from base keys to manifold keys: [`constructManifoldOptimizationGraph`](ManifoldOptimizer.cpp#L232)
- Equivalent factor construction: [`SubstituteFactor`](../factors/SubstituteFactor.h#L39)

Implementation detail:
- A CCC whose manifold dimension is positive becomes a free manifold-valued
  variable in `mopt_problem.values`.
- A fully constrained CCC becomes a fixed manifold in
  `mopt_problem.fixedManifolds`; equivalent factors can still recover values
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
- Core type: [`ConstraintManifold`](ConstraintManifold.h#L59)
- Dimension: `embedding_dim - constraint_dim` in [`ConstraintManifold`](ConstraintManifold.h#L117)
- Optional initial feasibility projection: [`ConstraintManifold::constructValues`](ConstraintManifold.cpp#L23)
- Recover base variables for substituted factors: [`ConstraintManifold::recover`](ConstraintManifold.cpp#L34)

The `ConstraintManifold` object stores exactly the data needed by paper
Eqs. (13)-(14): the base values \(X_c^C\), equality constraints \(H_c\),
the tangent-space basis, and the retractor used to move from a tangent update
back to feasible values.

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
- Abstract basis API: [`TangentSpaceBasis`](TangentSpaceBasis.h#L90)
- Orthonormal/null-space basis: [`OrthonormalBasis`](TangentSpaceBasis.h#L164)
- Constraint Jacobian: [`OrthonormalBasis::computeConstraintJacobian`](TangentSpaceBasis.cpp#L57)
- Dense null-space basis: [`OrthonormalBasis::constructDense`](TangentSpaceBasis.cpp#L108)
- Sparse null-space basis: [`OrthonormalBasis::constructSparse`](TangentSpaceBasis.cpp#L115)
- Map reduced coordinates \(\xi\) to ambient `VectorValues`: [`OrthonormalBasis::computeTangentVector`](TangentSpaceBasis.cpp#L223)

Parameterized/basis-variable path:
- Eliminate non-basis variables: [`EliminationBasis::construct`](TangentSpaceBasis.cpp#L492)
- Propagate Bayes-net Jacobians: [`computeBayesNetJacobian`](MultiJacobian.cpp#L140)
- Lift reduced coordinates to full tangent values: [`EliminationBasis::computeTangentVector`](TangentSpaceBasis.cpp#L572)

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
| Metric projection | Eq. (22) | [`ProjectionRetractor::retract`](Retractor.cpp#L88) |
| Approximate metric projection | Eq. (23) | [`UnconstrainedOptimizationRetractor::retractConstraints`](Retractor.cpp#L56) |
| Retract basis variables | Eq. (24) | [`BasisRetractor::retractConstraints`](Retractor.cpp#L154) |

Shared flow:
- Reduced tangent coordinates \(\xi\) become an ambient tangent update through
  the basis: [`ConstraintManifold::retract`](ConstraintManifold.cpp#L45)
- Base variables are retracted first: [`Retractor::retract`](Retractor.h#L96)
- The retractor then enforces the equality constraints: [`Retractor::retractConstraints`](Retractor.h#L118)

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

Code mapping:
- Equivalent factor type: [`SubstituteFactor`](../factors/SubstituteFactor.h#L39)
- New key list after substitution: [`computeNewKeys`](../factors/SubstituteFactor.cpp#L20)
- Separate free, fixed, and unchanged keys: [`classifyKeys`](../factors/SubstituteFactor.cpp#L52)
- Recover base values and evaluate the original factor: [`unwhitenedError`](../factors/SubstituteFactor.cpp#L82)
- Accumulate chain-rule Jacobians: [`SubstituteFactor.cpp#L113`](../factors/SubstituteFactor.cpp#L113)

The thesis derives the same chain rule through recover functions in
Eqs. (3.29)-(3.32), which is the clearest way to read the `SubstituteFactor`
implementation.

<a id="solvers"></a>
## 6) Solving the Transformed Problem
The paper's optimization target is the transformed unconstrained manifold
problem in Eq. (16). In GTDynamics there are two optimizer paths:

- Generic path: [`NonlinearManifoldOptimizer`](NonlinearManifoldOptimizer.cpp#L22) transforms
  the equality-constrained problem, then reuses standard GTSAM nonlinear
  optimizers on the manifold-valued variables.
- Custom LM path: [`LMManifoldOptimizer`](LMManifoldOptimizer.cpp#L43) keeps
  explicit state/trial bookkeeping for CM-Opt updates.

Benchmark helpers:
- `ConstrainedOptBenchmark::OptimizeCmOpt` uses `NonlinearManifoldOptimizer`: [`ConstrainedOptBenchmark.cpp#L514`](../constrained_optimizer/ConstrainedOptBenchmark.cpp#L514)
- Dense/null-space plus approximate projection defaults: [`DefaultMoptParams`](../constrained_optimizer/ConstrainedOptBenchmark.cpp#L487)
- Elimination basis plus basis-variable retraction defaults: [`DefaultMoptParamsSV`](../constrained_optimizer/ConstrainedOptBenchmark.cpp#L499)

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

In code, approximate behavior appears through retractor settings rather than a
separate mathematical type:
- limiting inner LM iterations in examples,
- using `UnconstrainedOptimizationRetractor` for approximate metric projection,
- using `BasisRetractor` for sparse/basis-variable updates,
- fast-pathing already feasible basis-retraction solves in
  [`BasisRetractor::retractConstraints`](Retractor.cpp#L154).

Paper Sec. V-C explains why the basis-variable version is closely related to
symbolic variable elimination: the chosen basis variables get direct tangent
updates, and the eliminated variables are recovered by satisfying constraints.
That is the conceptual model behind `EliminationBasis` plus `BasisRetractor`.

The thesis expands the approximate-retraction picture in Sec. 5.2, especially
Eqs. (5.22)-(5.27), and separates this from the standard feasible CM-Opt loop.

<a id="walkthrough"></a>
## 8) Quick Follow-One-Run Pointers
For a concrete end-to-end run, `examples/example_constraint_manifold/main_connected_poses.cpp` is a good example:
- Build equality constraints from a constraint graph: [`main_connected_poses.cpp#L269`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L269)
- Build default CM params: [`main_connected_poses.cpp#L307`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L307)
- Run CM optimization: [`main_connected_poses.cpp#L317`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L317)

Read the core implementation in this order:
1. [`ManifoldOptimizer.h`](ManifoldOptimizer.h) and [`ManifoldOptimizer.cpp`](ManifoldOptimizer.cpp)
2. [`ConstraintManifold.h`](ConstraintManifold.h) and [`ConstraintManifold.cpp`](ConstraintManifold.cpp)
3. [`TangentSpaceBasis.h`](TangentSpaceBasis.h), [`TangentSpaceBasis.cpp`](TangentSpaceBasis.cpp), and [`MultiJacobian.cpp`](MultiJacobian.cpp)
4. [`Retractor.h`](Retractor.h) and [`Retractor.cpp`](Retractor.cpp)
5. [`SubstituteFactor.h`](../factors/SubstituteFactor.h) and [`SubstituteFactor.cpp`](../factors/SubstituteFactor.cpp)
6. [`NonlinearManifoldOptimizer.h`](NonlinearManifoldOptimizer.h), [`NonlinearManifoldOptimizer.cpp`](NonlinearManifoldOptimizer.cpp), and optionally [`LMManifoldOptimizer.cpp`](LMManifoldOptimizer.cpp)

For inequality boundaries and corners, see the separate CMC-Opt documentation in
[`../cmcopt/README.md`](../cmcopt/README.md).
