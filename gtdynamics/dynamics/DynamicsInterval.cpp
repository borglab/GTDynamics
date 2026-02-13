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
#include <gtdynamics/utils/ContextUtils.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;

template <>
NonlinearFactorGraph Dynamics::aFactors<Interval>(
    const Interval& interval, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return aFactors(slice, robot, contact_points);
  });
}

template <>
NonlinearFactorGraph Dynamics::graph<Interval>(
    const Interval& interval, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points,
    const std::optional<double>& mu) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return this->graph(slice, robot, contact_points, mu);
  });
}

}  // namespace gtdynamics
