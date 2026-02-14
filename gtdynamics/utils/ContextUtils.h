/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ContextUtils.h
 * @brief Helpers to iterate over context types as slices.
 */

#pragma once

#include <gtdynamics/utils/Interval.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtdynamics {

/// Execute `f(slice)` for each slice represented by context.
template <class F>
void forEachSlice(const Slice& context, F&& f) {
  f(context);
}

/// Execute `f(slice)` for each slice represented by context.
template <class F>
void forEachSlice(const Interval& context, F&& f) {
  for (size_t k = context.k_start; k <= context.k_end; k++) {
    f(Slice(k));
  }
}

/// Build a factor graph by accumulating per-slice graphs.
template <class CONTEXT, class F>
gtsam::NonlinearFactorGraph collectFactors(const CONTEXT& context, F&& f) {
  gtsam::NonlinearFactorGraph graph;
  forEachSlice(context, [&](const Slice& slice) { graph.add(f(slice)); });
  return graph;
}

/// Build values by accumulating per-slice values.
template <class CONTEXT, class F>
gtsam::Values collectValues(const CONTEXT& context, F&& f) {
  gtsam::Values values;
  forEachSlice(context, [&](const Slice& slice) { values.insert(f(slice)); });
  return values;
}

}  // namespace gtdynamics
