/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  StaticsPhase.cpp
 * @brief Statics factors over phase contexts.
 */

#include <gtdynamics/statics/Statics.h>
#include <gtdynamics/utils/Phase.h>

namespace gtdynamics {

template <>
gtsam::NonlinearFactorGraph Statics::graph<Phase>(const Phase& phase,
                                                  const Robot& robot) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return graph(interval, robot);
}

template <>
gtsam::Values Statics::initialValues<Phase>(const Phase& phase,
                                            const Robot& robot,
                                            double gaussian_noise) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return initialValues(interval, robot, gaussian_noise);
}

template <>
gtsam::Values Statics::solve<Phase>(const Phase& phase, const Robot& robot,
                                    const gtsam::Values& configuration,
                                    double gaussian_noise) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return solve(interval, robot, configuration, gaussian_noise);
}

template <>
gtsam::Values Statics::minimizeTorques<Phase>(const Phase& phase,
                                              const Robot& robot) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return minimizeTorques(interval, robot);
}

}  // namespace gtdynamics
