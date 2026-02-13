/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  StaticsInterval.cpp
 * @brief Statics factors over interval contexts.
 */

#include <gtdynamics/statics/Statics.h>
#include <gtdynamics/utils/ContextUtils.h>
#include <gtdynamics/utils/Interval.h>

namespace gtdynamics {

template <>
gtsam::NonlinearFactorGraph Statics::graph<Interval>(
    const Interval& interval, const Robot& robot) const {
  return collectFactors(
      interval, [&](const Slice& slice) { return this->graph(slice, robot); });
}

template <>
gtsam::Values Statics::initialValues<Interval>(const Interval& interval,
                                               const Robot& robot,
                                               double gaussian_noise) const {
  return collectValues(interval, [&](const Slice& slice) {
    return initialValues(slice, robot, gaussian_noise);
  });
}

template <>
gtsam::Values Statics::solve<Interval>(const Interval& interval,
                                       const Robot& robot,
                                       const gtsam::Values& configuration,
                                       double gaussian_noise) const {
  return collectValues(interval, [&](const Slice& slice) {
    return solve(slice, robot, configuration, gaussian_noise);
  });
}

template <>
gtsam::Values Statics::minimizeTorques<Interval>(const Interval& interval,
                                                 const Robot& robot) const {
  return collectValues(
      interval, [&](const Slice& slice) { return minimizeTorques(slice, robot); });
}

}  // namespace gtdynamics
