# Point-splitting ADMM versus ProxQP (experimental)

This experiment compares **the same** CMC-Opt global B-reduced tangent-cone QP
with the existing native ProxQP backend and a new **variable-splitting ADMM**
backend. It does **not** replace the default GTSAM solver or alter nonlinear
retraction. No speedup claim is made without running the benchmarks.

## Mathematical split

CMC's QP is

\[
\min_\xi \tfrac12 \|J\xi-r\|^2+\tfrac\lambda2\|D\xi\|^2
\quad\text{such that}\quad A_i\xi_i\le 0 .
\]

The equality constraints are already eliminated by the local bases \(B_i\).
Every active inequality row belongs to one reduced CMC component \(i\).
Introduce \(z_i=\xi_i\) with \(z_i\in C_i=\{z:A_i z\le0\}\).
The scaled ADMM iteration is

\[
\begin{aligned}
(H+\rho I)\xi^{k+1}&=-g+\rho(z^k-u^k),\\
z_i^{k+1}&=\Pi_{C_i}(\xi_i^{k+1}+u_i^k),\\
u_i^{k+1}&=u_i^k+\xi_i^{k+1}-z_i^{k+1}.
\end{aligned}
\]

Here \(H\) includes the original LM damping. Unconstrained keys have
\(C_i=\mathbb{R}^{d_i}\), so their projection is the identity. The first step
factorizes the sparse \(H+\rho I\) **once per QP** and reuses it at every ADMM
iteration. The second step uses independently maintained local dual variables
and Dykstra/coordinate-dual projections, parallelized using the shared
forIndependentComponents helper (TBB when available, serial otherwise).

For this first benchmark, **the global factorization is Eigen's sparse
SimplicialLDLT applied to the assembled Hessian**. It does not yet reuse GTSAM's
multifrontal square-root factors. This is important: it compares an actual
prototype, not an assumed cached GTSAM implementation. A GTSAM cached-factor
global step is a separate follow-up optimization.

The implementation:

- Requires homogeneous, point-local inequalities and zero explicit equalities,
  exactly the tangent QP constructed by CMC-Opt.
- Leaves active-set discovery and row-order mapping intact; only positive dual,
  tight rows may be promoted for nonlinear retraction.
- Does not soften constraints. It rejects failed projections, nonconvergence and
  failure of **original Gaussian-factor** stationarity, primal/dual feasibility
  and complementarity checks. ADMM residuals alone do not certify a result.
- Uses fixed \(\rho\) to enable factorization reuse. Adaptive \(\rho\), warm
  starts across LM trials and optimized closed-form cone projections are future
  work; they are not part of reported timings.

## Correctness

Build the usual GTDynamics tests and run:

~~~bash
ctest --test-dir build -R testPointSplitAdmm --output-on-failure
ctest --test-dir build -R testIELMQp --output-on-failure
~~~

The tests cover a constrained optimum, a globally coupled two-component QP,
dependent and opposing faces, serial/parallel agreement, invalid inputs and
(optional) direct ProxQP comparisons. GTDYNAMICS_WITH_PROXQP=ON is required
for the ProxQP assertions and timing benchmark.

## Paired QP benchmark (synthetic chain)

Build GTDynamics with examples and ProxQP enabled, using the same GTSAM and
Eigen dependency versions for both backends:

~~~bash
cmake -S . -B build -DGTDYNAMICS_WITH_PROXQP=ON -DGTDYNAMICS_WITH_TBB=ON
cmake --build build --target admm_vs_proxqp -j8
python examples/cmc_benchmarks/compare_admm_proxqp.py \
  --binary build/examples/cmc_benchmarks/admm_vs_proxqp \
  --output /tmp/cmc-admm-compare \
  --sizes 8 32 128 512 --rhos 0.1 1 10 --threads 1 4 --repeats 5
~~~

The QP uses 3D local nonnegative cones and pairwise factors coupling all
components into a sparse chain. Both solvers receive exactly the same
gtsam::QpProblem, including identical costs, constraints, key ordering and
iteration limits. Each pair has one warmup (repeat=-1, excluded from medians)
and five measured samples. The order alternates between backends.

Outputs include raw CSV, failure logs, solution objective, ADMM primal/dual/KKT
residuals and ADMM factorization/projection time. A speed ratio is reported
**only if both backends converge and objective values agree**. Failed or timed
out cases are retained as failures, not zero-duration successes. The runner
does not claim identical complexity to real G1 optimization.

For fair CPU comparisons, isolate the host, fix core affinity and match TBB
configuration. Compare both --threads 1 and parallel settings; in the serial
case, no TBB task arena is launched by the projection helper.

## Real G1 end-to-end comparison

On the existing G1 dataset and working Python/MuJoCo environment:

~~~bash
python examples/g1_mjlab/kinodynamic_walk.py \
  --qp-backend proxqp --qp-step full --qp-max-iterations 10000 \
  --initialization contact-gp-dynamics --output /tmp/g1-proxqp
python examples/g1_mjlab/kinodynamic_walk.py \
  --qp-backend admm --qp-step full --qp-max-iterations 10000 \
  --admm-rho 1 --admm-projection-threads 4 \
  --initialization contact-gp-dynamics --output /tmp/g1-admm
~~~

The two runs retain the same initialization, reduced equalities, nonlinear
factors, retraction, damping and convergence tests; only the QP backend differs.
Compare solver time, outer LM iterations, accepted/rejected trials, final
objective, constraint violations and result arrays from the saved summaries.
In particular, **do not compare ProxQP full solves with the default GTSAM
feasible-step setting**.

For large captured G1 tangent QPs, the repository already documents G1SP20
(18,549 variables) and G1SPFULL (193,566 variables) under
[qp_benchmark](../g1_mjlab/qp_benchmark/README.md). Those artifacts are not
included here and no timings for them are asserted. The G1 study should
report memory/factorization cost and convergence failures as well as solve time.

## Limitations to evaluate before promotion

A fixed \(\rho\) can make ADMM converge very slowly on stiff, mixed-scale
systems. Convergence to KKT accuracy and accurate active-face selection matter
more than a cheap individual iteration, since retraction consumes those
faces. Sparse normal equations can also lose accuracy relative to GTSAM's
square-root QR path. This is deliberately an **opt-in experimental backend**;
the PR should not change CMC-Opt's production default.
