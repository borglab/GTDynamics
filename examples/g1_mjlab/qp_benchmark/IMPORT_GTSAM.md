Import these two local **sparse-support G1 QP datasets** into this GTSAM QP
branch. Keep the work local; do not submit or publish the datasets.

Read this directory first:
`/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets`

Dataset folders (already extracted):
- `/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets/G1SP20`
- `/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets/G1SPFULL`

Portable archives:
- `/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets/G1SP20.zip`
- `/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets/G1SPFULL.zip`

Standard sparse MAT exports for the existing free-for-all loader:
- `/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets/G1SP20.mat`
- `/home/dellaert/.codex/worktrees/cmc-sparse-qp/GTDynamics/build/qp-datasets/G1SPFULL.mat`

| Dataset | Motion window | Sparse slices | Reduced scalar variables | Inequality rows |
| --- | --- | --- | --- | --- |
| G1SP20 | 78–98 s | 171 | 18,549 | 230 |
| G1SPFULL | 0–219.94 s | 1,780 | 193,566 | 2,437 |

Both contain the exact **second global QP after one accepted nonlinear LM
step**, using single- and double-support slices. The first QP has no inequality
rows. Equalities were already eliminated through B. This is the current sparse
application; do not substitute the older dense-sampling G1CMCTWO dataset.

1. Read each README.md, metadata.json, provenance.json, and validation.json.
   Verify SHA256SUMS from inside its folder. Copy the data to an appropriate
   local benchmark-data location following this branch's conventions. Do not
   automatically commit large binary files to ordinary Git.

2. Implement a data loader using the existing dataset infrastructure where
   appropriate. `problem.py::load_factors()` returns raw sparse J, sparse G,
   vectors, and metadata. The objective is
   `0.5*||diag(inverse_sigmas)*(J*x-r)||^2`, with `G*x<=h`.
   Reconstruct each original JacobianFactor using its row_start, row_count,
   ordered keys, and global key offsets/dimensions. Extract only that factor's
   small blocks. Use its original r and diagonal noise sigmas; check that
   reconstructed inverse sigmas match the stored values. Add factors in their
   original order. Factors at and after damping_factor_start already contain
   LM damping (lambda=0.001). Parse key strings directly as uint64. Reconstruct
   scalar linear inequalities in their recorded order and retain row origins.

3. Keep the native GTSAM benchmark in original Jacobian-factor form. Avoid
   global dense matrices and a dense Hessian-per-connected-component adapter:
   temporal/contact coupling makes this a large connected problem. Do not add
   equalities, bounds, damping, regularization, coefficient truncation, or
   dropped rows. Start at zero with no warm start, use the sparse QP backend,
   regularization=0, maxIterations=1000, and the four 1e-7 tolerances in
   metadata.native_checks. Let this branch choose its normal initial working
   set; do not import the capture branch's diagnostic solver modifications.

4. Check objective and gradient at x=0 and
   `x[j]=1e-4*((j%17)-8)` using the recorded global ordering. Native costs are in
   metadata, gradients in native_gradients.npz, and inequality probe residuals
   in inequality_row_origins. Check zero feasibility. Follow pack.py's numeric
   comparison tolerances. Add a loader/representation regression test; no
   certified reference optimum is supplied.

5. For conventional solvers, make_problem() constructs a sparse
   qpbenchmark.Problem; the MAT files can also be loaded with the existing
   free-for-all loader. Both paths were validated. Keep Hessian-input and
   original-factor runs distinguishable: forming P=Jw.T*Jw introduces normal-
   equation rounding. Standard QP objectives omit 0.5*||W*r||^2; add the constant
   in validation.json when comparing to the captured native least-squares cost.
   The canonical NPZ bundle needs its supplied loader, not Problem.load().

6. Run bounded cold solves, first G1SP20 and then G1SPFULL, with **TBB enabled**.
   Record revision/build, thread/affinity settings, assembly time, solve time,
   convergence, iterations, original-input objective, primal feasibility,
   stationarity, dual feasibility, and complementarity. Use explicit watchdog
   budgets and distinguish timeout, numerical failure, and iteration exhaustion.
   Keep the branch's usual benchmark budget for comparable runs; report longer
   diagnostics separately. Do not change the problem or silently switch
   backends to obtain a passing result. If solver fixes are needed, validate
   them against the existing standard datasets too.

The capture used GTSAM b20aa267, whose diagnostic changes reportedly regress
other datasets. **Do not cherry-pick those changes.** The stored QPs are the
inputs to test against the current branch. GTDynamics, MuJoCo, and the original
recording are not needed to load or solve them. No certified minimizer or
performance improvement is implied by the export.
