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

}  // namespace gtdynamics
