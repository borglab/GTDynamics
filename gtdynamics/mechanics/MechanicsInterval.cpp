/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MechanicsInterval.cpp
 * @brief Mechanics factors over interval contexts.
 */

#include <gtdynamics/mechanics/Mechanics.h>
#include <gtdynamics/utils/ContextUtils.h>
#include <gtdynamics/utils/Interval.h>

namespace gtdynamics {

template <>
gtsam::NonlinearFactorGraph Mechanics::wrenchEquivalenceFactors<Interval>(
    const Interval& interval, const Robot& robot) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return wrenchEquivalenceFactors(slice, robot);
  });
}

template <>
gtsam::NonlinearFactorGraph Mechanics::torqueFactors<Interval>(
    const Interval& interval, const Robot& robot) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return torqueFactors(slice, robot);
  });
}

template <>
gtsam::NonlinearFactorGraph Mechanics::wrenchPlanarFactors<Interval>(
    const Interval& interval, const Robot& robot) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return wrenchPlanarFactors(slice, robot);
  });
}

}  // namespace gtdynamics
