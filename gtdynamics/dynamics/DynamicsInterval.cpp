/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsInterval.cpp
 * @brief Dynamics factors for interval contexts.
 */

#include <gtdynamics/dynamics/Dynamics.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;

template <>
NonlinearFactorGraph Dynamics::aFactors<Interval>(
    const Interval& interval, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(aFactors(Slice(k), robot, contact_points));
  }
  return graph;
}

template <>
NonlinearFactorGraph Dynamics::graph<Interval>(
    const Interval& interval, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points,
    const std::optional<double>& mu) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(this->graph(Slice(k), robot, contact_points, mu));
  }
  return graph;
}

}  // namespace gtdynamics
