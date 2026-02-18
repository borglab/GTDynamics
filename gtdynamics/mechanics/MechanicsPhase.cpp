/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MechanicsPhase.cpp
 * @brief Mechanics factors over phase contexts.
 */

#include <gtdynamics/mechanics/Mechanics.h>
#include <gtdynamics/utils/Phase.h>

namespace gtdynamics {

template <>
gtsam::NonlinearFactorGraph Mechanics::wrenchEquivalenceFactors<Phase>(
    const Phase& phase, const Robot& robot) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return wrenchEquivalenceFactors(interval, robot);
}

template <>
gtsam::NonlinearFactorGraph Mechanics::torqueFactors<Phase>(
    const Phase& phase, const Robot& robot) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return torqueFactors(interval, robot);
}

template <>
gtsam::NonlinearFactorGraph Mechanics::wrenchPlanarFactors<Phase>(
    const Phase& phase, const Robot& robot) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return wrenchPlanarFactors(interval, robot);
}

}  // namespace gtdynamics
