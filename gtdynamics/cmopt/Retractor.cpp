/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Retractor.cpp
 * @brief Equality-manifold retraction interfaces and common behavior.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/cmopt/Retractor.h>

#include <iostream>

namespace gtdynamics {
using namespace gtsam;

void Retractor::checkFeasible(const NonlinearFactorGraph &graph,
                              const Values &values) const {
  if (params_->checkFeasible) {
    if (graph.error(values) > params_->feasibleThreshold) {
      std::cout << "fail: " << graph.error(values) << "\n";
    }
  }
}


}  // namespace gtdynamics
