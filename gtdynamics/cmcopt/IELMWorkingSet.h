/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IELMWorkingSet.h
 * @brief Coordinate component-local working sets for the coupled LM objective.
 * @author GTDynamics contributors
 */
#pragma once

#include <gtdynamics/cmcopt/IEManifoldLinearization.h>
namespace gtdynamics {
namespace internal {
/// Borrow immutable component geometry for the duration of an LM trial.
using ComponentLinearizations =
    std::map<Key, std::shared_ptr<const IEManifoldLinearization>>;
/** Outcome of optional row-wise refinement of the coupled quadratic objective.
 * Only a converged result is eligible for nonlinear LM acceptance.
 */
struct WorkingSetResult {
  VectorValues delta;
  IndexSetMap blockingIndices;
  size_t numSolves = 0;
  bool converged = false;
};
/** Refine a feasible warm start with component-local charts and dual queries.
 * The optimizer coordinates row additions/releases using the coupled cost.
 * No global inequality graph, flat index space, or constrained solver exists.
 */
WorkingSetResult refineWorkingSet(const GaussianFactorGraph& cost,
                                  const ComponentLinearizations& components,
                                  const VectorValues& initial,
                                  const IndexSetMap& blockingIndices,
                                  const NonlinearOptimizerParams& params,
                                  size_t maxIterations);
}  // namespace internal
}  // namespace gtdynamics
