# Constraint Manifold Optimization

<a id="overview"></a>
## Overview
This note maps the math in *Constraint Manifolds for Robotic Inference and Planning* (ICRA 2023) to the GTDynamics implementation in `gtdynamics/cmopt`.

> **Note:** This document was created from the paper and code using GitHub Copilot (codex 5.3) on Feb 8, 2026.

The paper starts from a nonlinear equality-constrained problem:

\[
\min_{X \in \mathcal{M}} \sum_i f_i(X_i^f)
\quad \text{s.t.} \quad h_j(X_j^h)=0
\]

(paper Eq. 11), then transforms it into an unconstrained problem on new variables \(\Theta\):

\[
\min_{\Theta \in \bar{\mathcal{M}}} \sum_i \bar f_i(\Theta_i^f)
\]

(paper Eq. 16), where each constraint-connected component (CCC) is replaced by a constraint manifold variable.

In code, this transformation happens in three stages:
- Find CCCs from equality constraints: [`ManifoldOptimizer::IdentifyConnectedComponents`](ManifoldOptimizer.cpp#L119)
- Build one `ConstraintManifold` per CCC: [`ManifoldOptimizer::constructManifoldValues`](ManifoldOptimizer.cpp#L184)
- Build equivalent cost factors on manifold variables: [`ManifoldOptimizer::constructMoptGraph`](ManifoldOptimizer.cpp#L232), using [`SubstituteFactor`](../factors/SubstituteFactor.h#L39)
- Relevant headers/classes: [`ManifoldOptimizer`](ManifoldOptimizer.h), [`ConstraintManifold`](ConstraintManifold.h), [`SubstituteFactor`](../factors/SubstituteFactor.h)

If you are looking for where the paper’s “equivalent factors” are created, start at:
- [`ManifoldOptimizer.cpp#L252`](ManifoldOptimizer.cpp#L252)
- [`ManifoldOptimizer.cpp#L262`](ManifoldOptimizer.cpp#L262)
- [`SubstituteFactor.cpp#L82`](../factors/SubstituteFactor.cpp#L82)

<a id="ccc-transformation"></a>
## 1) CCC Identification and Problem Transformation (Paper Sec. III-A)
Paper definition:
- CCC \(C_c = (X_c^C, H_c(X_c^C)=0)\)
- Replace each CCC by \(\theta_c \in \mathcal{M}_c\)
- Replace each affected cost factor \(f_i\) by \(\bar f_i\) through substitution (paper Eq. 15)

Code mapping:
- DFS over constraint-variable bipartite structure: [`IdentifyConnectedComponent`](ManifoldOptimizer.cpp#L88)
- All components: [`IdentifyConnectedComponents`](ManifoldOptimizer.cpp#L119)
- Full transformation pipeline: [`problemTransform`](ManifoldOptimizer.cpp#L165)
- Build map from base keys to manifold keys: [`constructMoptGraph`](ManifoldOptimizer.cpp#L235)
- Relevant headers/classes: [`ManifoldOptimizer`](ManifoldOptimizer.h), [`ManifoldOptProblem`](ManifoldOptimizer.h), [`ConstraintManifold`](ConstraintManifold.h)

<a id="constraint-manifold"></a>
## 2) Constraint Manifold Object and Dimension (Paper Sec. IV)
Paper definition:

\[
\mathcal{M}_c = \{X_c^C \in \tilde{\mathcal{M}}_c \mid H_c(X_c^C)=0\}
\]

(paper Eq. 13).

Code mapping:
- Core type: [`ConstraintManifold`](ConstraintManifold.h#L46)
- Dimension set as `embedding_dim - constraint_dim`: [`ConstraintManifold.h#L90`](ConstraintManifold.h#L90)
- Optional initial feasibility projection on construction: [`ConstraintManifold::constructValues`](ConstraintManifold.cpp#L23)
- Recover base variable from manifold value (used by substituted factors): [`ConstraintManifold::recover`](ConstraintManifold.cpp#L34)
- Relevant headers/classes: [`ConstraintManifold`](ConstraintManifold.h), [`EManifoldValues`](ConstraintManifold.h)

<a id="tangent-basis"></a>
## 3) Tangent Space Basis \(T_{\theta_c}\mathcal{M}_c=\ker D H_c\) (Paper Eqs. 19–20)
Paper definition:

\[
T_{\theta_c}\mathcal{M}_c = \ker D H_c(X_c^C), \quad
\delta X = B_{\theta_c}\,\xi
\]

Code mapping (orthonormal/null-space basis):
- Build constraint Jacobian from linearized penalty graph: [`OrthonormalBasis::computeConstraintJacobian`](TspaceBasis.cpp#L57)
- Dense null-space basis \(N = \mathrm{kernel}(J_H)\): [`OrthonormalBasis::constructDense`](TspaceBasis.cpp#L108)
- Sparse basis construction: [`OrthonormalBasis::constructSparse`](TspaceBasis.cpp#L115)
- Map reduced coordinates \(\xi\) to ambient tangent vector \(\delta X\): [`OrthonormalBasis::computeTangentVector`](TspaceBasis.cpp#L223)

Code mapping (elimination/basis-variable parameterization):
- Eliminate non-basis vars and recover Jacobian dependencies: [`EliminationBasis::construct`](TspaceBasis.cpp#L492)
- Bayes-net Jacobian propagation (chain rule): [`ComputeBayesNetJacobian`](MultiJacobian.cpp#L140)
- Reduced-to-full tangent lift: [`EliminationBasis::computeTangentVector`](TspaceBasis.cpp#L572)
- Relevant headers/classes: [`TspaceBasis`](TspaceBasis.h), [`OrthonormalBasis`](TspaceBasis.h), [`EliminationBasis`](TspaceBasis.h), [`MultiJacobian`](MultiJacobian.h)

<a id="retraction"></a>
## 4) Retraction on Constraint Manifold (Paper Sec. IV-B, Eqs. 22–24)
Paper idea:
- Retraction is done by moving in tangent space, then projecting/matching back to satisfy constraints.
- Paper gives metric projection (Eq. 22), approximate projection (Eq. 23), and basis-variable retraction (Eq. 24).

Code mapping:
- Generic flow \( \xi \to \delta X \to R(\delta X)\): [`ConstraintManifold::retract`](ConstraintManifold.cpp#L45)
- Retraction base class, with “base retract then satisfy constraints”: [`Retractor::retract`](Retractor.h#L87)

Implementations:
- Approximate projection via unconstrained solve on penalty graph (close to Eq. 23): [`UoptRetractor::retractConstraints`](Retractor.cpp#L56)
- Projection with extra priors before optional cleanup solve: [`ProjRetractor::retract`](Retractor.cpp#L88)
- Basis-variable retraction (close to Eq. 24): [`BasisRetractor::retractConstraints`](Retractor.cpp#L154)
- Relevant headers/classes: [`Retractor`](Retractor.h), [`UoptRetractor`](Retractor.h), [`ProjRetractor`](Retractor.h), [`BasisRetractor`](Retractor.h), [`DynamicsRetractor`](Retractor.h)

Approximate retraction mode from the paper (Sec. V-B, Eq. 26) is used in examples by limiting inner retraction iterations:
- [`main_connected_poses.cpp#L323`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L323)
- [`main_cablerobot.cpp#L223`](../../examples/example_constraint_manifold/main_cablerobot.cpp#L223)

<a id="equivalent-factors"></a>
## 5) Equivalent Cost Factors and Chain Rule Jacobians (Paper Eq. 25)
Paper equation:

\[
J_{\bar f}(\theta_c)=\sum_{k \in I_c^C \cap I_i^f}
J_{r_k}(\theta_c)\,J_f(x_k)
\]

Code mapping:
- Equivalent factor type: [`SubstituteFactor`](../factors/SubstituteFactor.h#L39)
- Key substitution and de-duplication: [`computeNewKeys`](../factors/SubstituteFactor.cpp#L20)
- Handle fully constrained components (fixed manifolds): [`classifyKeys`](../factors/SubstituteFactor.cpp#L52)
- Build base values and evaluate base factor: [`unwhitenedError`](../factors/SubstituteFactor.cpp#L82)
- Chain-rule Jacobian accumulation with recover Jacobians: [`SubstituteFactor.cpp#L113`](../factors/SubstituteFactor.cpp#L113)
- Relevant headers/classes: [`ManifoldOptimizer`](ManifoldOptimizer.h), [`ConstraintManifold`](ConstraintManifold.h), [`SubstituteFactor`](../factors/SubstituteFactor.h)

This is the concrete implementation of paper Eq. 15 (factor substitution) + Eq. 25 (Jacobian chain rule).

<a id="solvers"></a>
## 6) Solving the Transformed Problem
There are two optimizer paths in code:
- Generic “reuse GTSAM nonlinear optimizers on manifold-valued variables”: [`NonlinearMOptimizer`](NonlinearMOptimizer.cpp#L22)
- Custom LM implementation with explicit trial/state bookkeeping: [`LMManifoldOptimizer`](LMManifoldOptimizer.cpp#L43)

In benchmark helpers, `ConstrainedOptBenchmark::OptimizeCmOpt` uses
`NonlinearMOptimizer`:
- [`ConstrainedOptBenchmark.cpp`](../constrained_optimizer/ConstrainedOptBenchmark.cpp)

Default CM parameter builders:
- Dense/null-space + unconstrained retraction:
  [`ConstrainedOptBenchmark::DefaultMoptParams`](../constrained_optimizer/ConstrainedOptBenchmark.cpp)
- Basis-variable elimination + basis retraction:
  [`ConstrainedOptBenchmark::DefaultMoptParamsSV`](../constrained_optimizer/ConstrainedOptBenchmark.cpp)
- Relevant headers/classes: [`NonlinearMOptimizer`](NonlinearMOptimizer.h), [`LMManifoldOptimizer`](LMManifoldOptimizer.h), [`LMState/LMTrial`](LMManifoldOptimizerState.h), [`ManifoldOptimizerParameters`](ManifoldOptimizer.h)

<a id="walkthrough"></a>
## 7) Quick “Follow One Run” Pointers
For a concrete end-to-end run, `examples/example_constraint_manifold/main_connected_poses.cpp` is a good example:
- Build equality constraints from a constraint graph: [`main_connected_poses.cpp#L269`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L269)
- Build default CM params: [`main_connected_poses.cpp#L307`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L307)
- Run CM optimization: [`main_connected_poses.cpp#L317`](../../examples/example_constraint_manifold/main_connected_poses.cpp#L317)
- Relevant headers/classes: [`ManifoldOptimizer`](ManifoldOptimizer.h), [`ConstraintManifold`](ConstraintManifold.h), [`NonlinearMOptimizer`](NonlinearMOptimizer.h)
