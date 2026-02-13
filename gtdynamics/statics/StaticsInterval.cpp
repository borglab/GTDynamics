/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  StaticsInterval.cpp
 * @brief Statics factors over interval/phase contexts.
 */

#include <gtdynamics/statics/Statics.h>
#include <gtdynamics/utils/Phase.h>

namespace gtdynamics {

gtsam::NonlinearFactorGraph Statics::graph(const Interval& interval,
                                           const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(this->graph(Slice(k), robot));
  }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::graph(const Phase& phase,
                                           const Robot& robot) const {
  const Interval& interval = static_cast<const Interval&>(phase);
  return graph(interval, robot);
}

}  // namespace gtdynamics
