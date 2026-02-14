/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  DynamicsPhase.cpp
 * @brief Dynamics factors for phase contexts.
 */

#include <gtdynamics/dynamics/Dynamics.h>
#include <gtdynamics/utils/Phase.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;

template <>
NonlinearFactorGraph Dynamics::aFactors<Phase>(
    const Phase& phase, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return aFactors(interval, robot, contact_points);
}

template <>
NonlinearFactorGraph Dynamics::graph<Phase>(
    const Phase& phase, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points,
    const std::optional<double>& mu) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return graph(interval, robot, contact_points, mu);
}

}  // namespace gtdynamics
