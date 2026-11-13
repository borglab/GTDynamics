/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file IndependentComponents.h
 * @brief Execute independent manifold projections with bounded concurrency.
 * @author GTDynamics contributors
 */
#pragma once
#include <gtsam/config.h>

#include <cstddef>
#ifdef GTSAM_USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#endif
namespace gtdynamics {
namespace internal {
/** Evaluate independent components, with a serial fallback in non-TBB builds.
 * The body must own distinct writable data per index. Exceptions propagate;
 * callers must discard the entire trial when any component fails. At most
 * threads tasks execute concurrently; one preserves the serial caller behavior.
 */
template <class Body>
void forIndependentComponents(size_t count, size_t threads, const Body& body) {
#ifdef GTSAM_USE_TBB
  if (threads > 1) {
    tbb::task_arena arena(static_cast<int>(threads));
    arena.execute([&] { tbb::parallel_for(size_t(0), count, body); });
    return;
  }
#endif
  for (size_t i = 0; i < count; ++i) body(i);
}
}  // namespace internal
}  // namespace gtdynamics
